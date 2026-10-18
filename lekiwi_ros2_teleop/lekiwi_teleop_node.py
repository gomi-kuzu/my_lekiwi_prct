#!/usr/bin/env python3

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS2 Teleoperation Node for LeKiwi Robot

This node subscribes to ROS2 topics to receive commands and publishes observations.
Based on lekiwi_host.py but using ROS2 topic communication instead of ZMQ.
"""

import os
import shutil
import subprocess
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState, Image, CompressedImage
from std_msgs.msg import Float32MultiArray, Header

# Import LeKiwi classes from lerobot
# LEROBOT_PATH environment variable must be set
lerobot_path = os.environ.get('LEROBOT_PATH')
if lerobot_path is None:
    raise EnvironmentError(
        "LEROBOT_PATH environment variable is not set. "
        "Please set it to the path of your LeRobot installation's src directory. "
        "Example: export LEROBOT_PATH='/path/to/lerobot/src'"
    )
if lerobot_path not in sys.path:
    sys.path.append(lerobot_path)

from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig
from lerobot.robots.lekiwi.lekiwi import LeKiwi
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


class LeKiwiArmOnly(LeKiwi):
    """LeKiwi subclass that operates only the arm (motors 1-6), without the mobile base."""

    def __init__(self, config: LeKiwiConfig):
        # Call Robot.__init__ (grandparent), skipping LeKiwi.__init__ to avoid base motors
        from lerobot.robots.robot import Robot
        Robot.__init__(self, config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "arm_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.arm_motors = list(self.bus.motors.keys())
        self.base_motors = []
        self.cameras = make_cameras_from_configs(config.cameras)

    def configure(self):
        """Configure only arm motors (skip base motor velocity mode setup)."""
        self.bus.disable_torque()
        self.bus.configure_motors()
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", name, 16)
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 32)
        self.bus.enable_torque()

    def get_observation(self):
        """Read arm positions only (no base wheel velocities)."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        import time
        import logging
        logger = logging.getLogger(__name__)

        start = time.perf_counter()
        arm_pos = self.bus.sync_read("Present_Position", self.arm_motors)
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        obs_dict = {f"{k}.pos": v for k, v in arm_pos.items()}

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action):
        """Send arm position commands only (ignore base velocity keys)."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        arm_goal_pos = {k: v for k, v in action.items() if k.endswith(".pos")}

        if self.config.max_relative_target is not None:
            from lerobot.robots.utils import ensure_safe_goal_position
            present_pos = self.bus.sync_read("Present_Position", self.arm_motors)
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in arm_goal_pos.items()}
            arm_goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in arm_goal_pos.items()}
        self.bus.sync_write("Goal_Position", arm_goal_pos_raw)
        return arm_goal_pos

    def stop_base(self):
        """No-op: no base motors in arm-only mode."""
        pass

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()


class LeKiwiTeleopNode(Node):
    """
    ROS2 Node for LeKiwi robot teleoperation.
    
    Subscribes to:
        - /lekiwi/cmd_vel (geometry_msgs/Twist): Base velocity commands
        - /lekiwi/arm_joint_commands (sensor_msgs/JointState): Arm joint position commands
    
    Publishes:
        - /lekiwi/joint_states (sensor_msgs/JointState): Current robot state
        - /lekiwi/camera/front/image_raw/compressed (sensor_msgs/CompressedImage): Front camera
        - /lekiwi/camera/wrist/image_raw/compressed (sensor_msgs/CompressedImage): Wrist camera
    """
    
    def __init__(self):
        super().__init__('lekiwi_teleop_node')
        
        # Declare parameters
        self.declare_parameter('robot_port', '/dev/ttyACM0')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('watchdog_timeout_ms', 500)
        self.declare_parameter('use_degrees', False)
        self.declare_parameter('rotate_front_camera', True)  # Front camera mounted upside down
        # Use by-path for stable identification (based on USB port location)
        # This is better for identical camera models with same serial numbers
        # Note: Connect cameras to different USB ports to get different paths
        self.declare_parameter('front_camera_device', 
            '/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0,/dev/video2')  
        # NOTE: Hub port may enumerate differently depending on physical port.
        # Update this if `ls /dev/v4l/by-path/` shows a different suffix on your system.
        self.declare_parameter('wrist_camera_device', 
            '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1:1.0-video-index0,/dev/video0')
        
        ##### not use usb-hub #########
#         この辺は、USBケーブルを別の場所に差し替えると名前が変わる
#         調べる、コマンド
#         ls -la /dev/v4l/by-path/ 2>&1; echo "---by-id---"; ls -la /dev/v4l/by-id/ 2>&1; echo "---list-devices---"; v4l2-ctl --list-devices 2>&1
        ##---------------------------##

        # Camera settings
        self.declare_parameter('camera_fps', 30)  # Camera FPS (must match hardware capability)
        self.declare_parameter('front_camera_width', 640)  # Front camera width
        self.declare_parameter('front_camera_height', 480)  # Front camera height
        # NOTE: The wrist camera is physically mounted rotated 90°,
        # so width/height are swapped compared to the front camera.
        self.declare_parameter('wrist_camera_width', 480)  # Wrist camera width
        self.declare_parameter('wrist_camera_height', 640)  # Wrist camera height
        # Use MJPG (compressed) to drastically reduce USB bandwidth so two cameras
        # can run on the same USB controller/hub. Empty string = driver default (often
        # uncompressed YUYV), which frequently causes 'read failed (status=False)'.
        self.declare_parameter('camera_fourcc', 'MJPG')
        # Manual exposure. These icSpring cameras default to auto_exposure=3 (Aperture
        # Priority), which stretches exposure time in dim light and drops the effective
        # capture rate to ~13.5fps. Forcing manual exposure with a short exposure time
        # restores ~28-30fps. Set camera_manual_exposure=False to keep the driver default.
        # exposure_time ~200 (20ms) keeps >=28fps; increase for brighter images (but
        # values above ~300 start dropping below 30fps). gain compensates brightness
        # without costing frame rate (max 63 on these cameras).
        self.declare_parameter('camera_manual_exposure', True)
        self.declare_parameter('camera_exposure_time', 200)  # exposure_time_absolute (100us units)
        self.declare_parameter('camera_gain', 63)  # 0..63; higher = brighter (adds noise)
        self.declare_parameter('camera_power_line_frequency', 0)  # 0=Disabled,1=50Hz,2=60Hz
        self.declare_parameter('enable_cameras', True)  # Allow disabling cameras for debugging
        self.declare_parameter('allow_camera_failure', True)  # Continue robot operation even if cameras fail
        self.declare_parameter('arm_only_mode', False)  # Arm-only mode: disable base and cameras
        # Subscribe to /lekiwi/cmd_vel as TwistStamped (carries header.stamp so Stage 2
        # can time-sync it like the other topics). Set False to accept plain Twist
        # (e.g. teleop_twist_keyboard, or teleop_twist_joy with publish_stamped_twist=false).
        self.declare_parameter('cmd_vel_stamped', True)
        
        # Get parameters
        robot_port = self.get_parameter('robot_port').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.watchdog_timeout_ms = self.get_parameter('watchdog_timeout_ms').value
        use_degrees = self.get_parameter('use_degrees').value
        self.rotate_front_camera = self.get_parameter('rotate_front_camera').value
        front_camera_device_list = self.get_parameter('front_camera_device').value.split(',')
        wrist_camera_device_list = self.get_parameter('wrist_camera_device').value.split(',')
        camera_fourcc = self.get_parameter('camera_fourcc').value
        self.camera_manual_exposure = self.get_parameter('camera_manual_exposure').value
        self.camera_exposure_time = self.get_parameter('camera_exposure_time').value
        self.camera_gain = self.get_parameter('camera_gain').value
        self.camera_power_line_frequency = self.get_parameter('camera_power_line_frequency').value
        camera_fps = self.get_parameter('camera_fps').value
        front_camera_width = self.get_parameter('front_camera_width').value
        front_camera_height = self.get_parameter('front_camera_height').value
        wrist_camera_width = self.get_parameter('wrist_camera_width').value
        wrist_camera_height = self.get_parameter('wrist_camera_height').value
        self.enable_cameras = self.get_parameter('enable_cameras').value
        allow_camera_failure = self.get_parameter('allow_camera_failure').value
        self.arm_only_mode = self.get_parameter('arm_only_mode').value
        self.cmd_vel_stamped = self.get_parameter('cmd_vel_stamped').value
        self._robot_class = LeKiwiArmOnly if self.arm_only_mode else LeKiwi
        if self.arm_only_mode:
            self.enable_cameras = False
            self.get_logger().info('Arm-only mode enabled: base and cameras are disabled')
        
        # Configure robot
        self.get_logger().info('Configuring LeKiwi robot...')
        robot_config = LeKiwiConfig(
            port=robot_port,
            use_degrees=use_degrees,
            disable_torque_on_disconnect=True,
        )
        
        # Override camera settings
        if self.enable_cameras:
            # Use first device in the list as primary
            front_camera_device = front_camera_device_list[0].strip()
            wrist_camera_device = wrist_camera_device_list[0].strip()
            
            robot_config.cameras['front'].index_or_path = front_camera_device
            robot_config.cameras['front'].fps = camera_fps
            robot_config.cameras['front'].width = front_camera_width
            robot_config.cameras['front'].height = front_camera_height
            
            robot_config.cameras['wrist'].index_or_path = wrist_camera_device
            robot_config.cameras['wrist'].fps = camera_fps
            robot_config.cameras['wrist'].width = wrist_camera_width
            robot_config.cameras['wrist'].height = wrist_camera_height
            
            # Apply FOURCC (e.g. MJPG) to reduce USB bandwidth usage.
            # An empty string means "leave driver default".
            if camera_fourcc:
                robot_config.cameras['front'].fourcc = camera_fourcc
                robot_config.cameras['wrist'].fourcc = camera_fourcc
            
            self.get_logger().info(f'Front camera: {front_camera_device} (fallback: {front_camera_device_list[1:]})')
            self.get_logger().info(f'  Resolution: {front_camera_width}x{front_camera_height}@{camera_fps}fps')
            self.get_logger().info(f'Wrist camera: {wrist_camera_device} (fallback: {wrist_camera_device_list[1:]})')
            self.get_logger().info(f'  Resolution: {wrist_camera_width}x{wrist_camera_height}@{camera_fps}fps, rotated 90°')
            self.get_logger().info(f'  FOURCC: {camera_fourcc or "driver default"}')
        else:
            # Disable cameras by removing them from config
            robot_config.cameras = {}
            self.get_logger().info('Cameras disabled')
        
        self.robot = self._robot_class(robot_config)
        
        # Connect to robot with camera fallback handling
        self.get_logger().info('Connecting to LeKiwi robot...')
        connected = False
        last_error = None
        
        # Try initial connection
        try:
            self.robot.connect()
            self.get_logger().info('LeKiwi robot connected successfully!')
            connected = True
        except Exception as e:
            last_error = str(e)
            # If camera failure and allow_camera_failure is True, try fallback devices
            if allow_camera_failure and self.enable_cameras and 'OpenCVCamera' in str(e):
                self.get_logger().warn(f'Camera initialization failed: {e}')
                
                # Determine which camera failed and try fallbacks
                failed_camera = None
                if 'front' in str(e).lower() or front_camera_device in str(e):
                    failed_camera = 'front'
                    fallback_list = front_camera_device_list
                elif 'wrist' in str(e).lower() or wrist_camera_device in str(e):
                    failed_camera = 'wrist'
                    fallback_list = wrist_camera_device_list
                else:
                    # Try wrist fallbacks first, then front
                    failed_camera = 'wrist'
                    fallback_list = wrist_camera_device_list
                
                # Try fallback devices
                for fallback_device in fallback_list[1:]:
                    fallback_device = fallback_device.strip()
                    self.get_logger().info(f'Trying fallback {failed_camera} camera: {fallback_device}')
                    try:
                        # Recreate robot config with new device
                        if failed_camera == 'front':
                            robot_config.cameras['front'].index_or_path = fallback_device
                        else:
                            robot_config.cameras['wrist'].index_or_path = fallback_device
                        self.robot = self._robot_class(robot_config)
                        self.robot.connect()
                        self.get_logger().info(f'Connected with {failed_camera} camera: {fallback_device}')
                        connected = True
                        break
                    except Exception as fallback_error:
                        self.get_logger().warn(f'Fallback {fallback_device} failed: {fallback_error}')
                        last_error = str(fallback_error)
                
                # If all camera devices failed, try without cameras
                if not connected:
                    self.get_logger().warn('All camera devices failed. Trying without cameras...')
                    try:
                        robot_config.cameras = {}
                        self.robot = self._robot_class(robot_config)
                        self.robot.connect()
                        self.get_logger().warn('=' * 60)
                        self.get_logger().warn('WARNING: LeKiwi robot connected WITHOUT CAMERAS!')
                        self.get_logger().warn('Camera images will not be published.')
                        self.get_logger().warn('=' * 60)
                        self.enable_cameras = False  # Disable camera publishing
                        connected = True
                    except Exception as no_cam_error:
                        last_error = str(no_cam_error)
        
        if not connected:
            self.get_logger().error(f'Failed to connect to robot: {last_error}')
            raise RuntimeError(last_error)
        
        # Force manual exposure so cameras run at full frame rate (auto exposure
        # otherwise drops them to ~13.5fps in dim light).
        if self.enable_cameras and self.camera_manual_exposure:
            self._apply_manual_exposure()
        
        # Print camera status warning if cameras are disabled
        if not self.enable_cameras:
            self.get_logger().warn('Camera publishing is DISABLED - no camera topics will be published')
        
        # Initialize command tracking
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_active = False
        
        # Initialize last action state
        self.last_arm_positions = {
            "arm_shoulder_pan.pos": 0.0,
            "arm_shoulder_lift.pos": -78.0,
            "arm_elbow_flex.pos": 82.0,
            "arm_wrist_flex.pos": 62.0,
            "arm_wrist_roll.pos": 0.0,
            "arm_gripper.pos": 1.5,
        }
        self.last_base_velocities = {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }
        
        # Camera error tracking for logging throttle
        self.camera_error_counts = {
            'front': 0,
            'wrist': 0,
        }
        self.camera_last_error_time = {
            'front': None,
            'wrist': None,
        }
        self.camera_error_log_interval = 5.0  # Log camera errors at most once per 5 seconds
        
        # Reconnection tracking
        self.reconnection_attempt_count = 0
        self.max_reconnection_attempts = 3
        self.reconnection_cooldown = 5.0  # Wait 5 seconds between reconnection attempts
        self.last_reconnection_time = None
        self.consecutive_errors = 0
        self.error_threshold = 10  # Trigger reconnection after 10 consecutive errors
        
        # Store camera config for reconnection
        self.robot_config = robot_config
        self.front_camera_device_list = front_camera_device_list
        self.wrist_camera_device_list = wrist_camera_device_list
        self.allow_camera_failure = allow_camera_failure
        
        # QoS profile for real-time control
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers
        if not self.arm_only_mode:
            if self.cmd_vel_stamped:
                self.cmd_vel_sub = self.create_subscription(
                    TwistStamped,
                    '/lekiwi/cmd_vel',
                    self.cmd_vel_callback,
                    qos_profile
                )
            else:
                self.cmd_vel_sub = self.create_subscription(
                    Twist,
                    '/lekiwi/cmd_vel',
                    self.cmd_vel_callback,
                    qos_profile
                )
        
        self.arm_cmd_sub = self.create_subscription(
            JointState,
            '/lekiwi/arm_joint_commands',
            self.arm_joint_callback,
            qos_profile
        )
        
        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/lekiwi/joint_states',
            qos_profile
        )
        
        self.front_camera_pub = self.create_publisher(
            CompressedImage,
            '/lekiwi/camera/front/image_raw/compressed',
            qos_profile
        )
        
        self.wrist_camera_pub = self.create_publisher(
            CompressedImage,
            '/lekiwi/camera/wrist/image_raw/compressed',
            qos_profile
        )
        
        # Create timer for main control loop
        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'LeKiwi teleoperation node started at {self.control_frequency} Hz')
    
    def _apply_manual_exposure(self):
        """Force manual exposure on each camera via v4l2-ctl.

        These cameras default to Aperture Priority auto-exposure, which lengthens the
        exposure time in dim light and caps the effective capture rate at ~13.5fps.
        Switching to manual mode with a short exposure time restores full frame rate.
        """
        v4l2 = shutil.which('v4l2-ctl')
        if v4l2 is None:
            self.get_logger().warn(
                'v4l2-ctl not found; cannot set manual exposure. '
                'Install v4l-utils to avoid auto-exposure frame-rate drops.'
            )
            return

        cameras = getattr(self.robot, 'cameras', {}) or {}
        for cam_name, cam in cameras.items():
            device = getattr(cam, 'index_or_path', None)
            if device is None:
                continue
            device = str(device)
            controls = [
                'auto_exposure=1',  # 1 = Manual Mode
                f'exposure_time_absolute={self.camera_exposure_time}',
                f'gain={self.camera_gain}',
                f'power_line_frequency={self.camera_power_line_frequency}',
            ]
            try:
                subprocess.run(
                    [v4l2, '-d', device, '--set-ctrl', ','.join(controls)],
                    check=True, capture_output=True, text=True, timeout=5,
                )
                self.get_logger().info(
                    f"Manual exposure applied to '{cam_name}' ({device}): "
                    f"exposure_time={self.camera_exposure_time}, gain={self.camera_gain}, "
                    f"power_line_frequency={self.camera_power_line_frequency}"
                )
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as e:
                stderr = getattr(e, 'stderr', '') or ''
                self.get_logger().warn(
                    f"Failed to set manual exposure on '{cam_name}' ({device}): {e} {stderr}"
                )
    
    def cmd_vel_callback(self, msg):

        """Callback for base velocity commands (Twist or TwistStamped)."""
        twist = msg.twist if isinstance(msg, TwistStamped) else msg
        self.last_base_velocities = {
            "x.vel": twist.linear.x,
            "y.vel": twist.linear.y,
            "theta.vel": np.degrees(twist.angular.z),  # Convert rad/s to deg/s
        }
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_active = False
    
    def arm_joint_callback(self, msg: JointState):
        """Callback for arm joint position commands."""
        # Expected joint names in order:
        # arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex, 
        # arm_wrist_flex, arm_wrist_roll, arm_gripper
        if len(msg.position) >= 6:
            joint_names = [
                "arm_shoulder_pan.pos",
                "arm_shoulder_lift.pos", 
                "arm_elbow_flex.pos",
                "arm_wrist_flex.pos",
                "arm_wrist_roll.pos",
                "arm_gripper.pos",
            ]
            for i, name in enumerate(joint_names):
                if i < len(msg.position):
                    self.last_arm_positions[name] = msg.position[i]
            
            self.last_cmd_time = self.get_clock().now()
            self.watchdog_active = False
        else:
            self.get_logger().warn(f'Received joint command with only {len(msg.position)} positions, expected 6')
    
    def control_loop(self):
        """Main control loop executed at control_frequency."""
        try:
            # Check watchdog
            now = self.get_clock().now()
            time_since_last_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
            
            if time_since_last_cmd > (self.watchdog_timeout_ms / 1000.0) and not self.watchdog_active:
                self.watchdog_active = True
                if not self.arm_only_mode:
                    self.get_logger().warn(
                        f'Command not received for more than {self.watchdog_timeout_ms} ms. Stopping the base.'
                    )
                    self.robot.stop_base()
                    # Reset base velocities
                    self.last_base_velocities = {
                        "x.vel": 0.0,
                        "y.vel": 0.0,
                        "theta.vel": 0.0,
                    }
            
            # Combine arm and base commands
            if self.arm_only_mode:
                action = {**self.last_arm_positions}
            else:
                action = {**self.last_arm_positions, **self.last_base_velocities}
            
            # Send action to robot
            self.robot.send_action(action)
            
            # Get observations
            observation = self.robot.get_observation()
            
            # Reset consecutive errors on successful observation
            self.consecutive_errors = 0
            
            # Publish joint states
            self.publish_joint_states(observation)
            
            # Publish camera images
            self.publish_camera_images(observation)
            
        except Exception as e:
            self.consecutive_errors += 1
            self.get_logger().error(f'Error in control loop: {e} (consecutive errors: {self.consecutive_errors})')
            
            # Try reconnection if too many consecutive errors
            if self.consecutive_errors >= self.error_threshold:
                self.get_logger().warn(f'Reached error threshold ({self.error_threshold}). Attempting reconnection...')
                if self.attempt_reconnection():
                    self.consecutive_errors = 0
                    self.get_logger().info('Reconnection successful!')
                else:
                    self.get_logger().error('Reconnection failed. Will retry later.')
    
    def publish_joint_states(self, observation: dict):
        """Publish current joint states."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Arm joints
        msg.name = [
            'arm_shoulder_pan',
            'arm_shoulder_lift',
            'arm_elbow_flex',
            'arm_wrist_flex',
            'arm_wrist_roll',
            'arm_gripper',
        ]
        
        msg.position = [
            observation.get('arm_shoulder_pan.pos', 0.0),
            observation.get('arm_shoulder_lift.pos', 0.0),
            observation.get('arm_elbow_flex.pos', 0.0),
            observation.get('arm_wrist_flex.pos', 0.0),
            observation.get('arm_wrist_roll.pos', 0.0),
            observation.get('arm_gripper.pos', 0.0),
        ]
        
        # Base velocities (published as velocities in joint state)
        msg.velocity = [
            observation.get('x.vel', 0.0),
            observation.get('y.vel', 0.0),
            np.radians(observation.get('theta.vel', 0.0)),  # Convert deg/s to rad/s
        ]
        
        self.joint_state_pub.publish(msg)
    
    def attempt_reconnection(self) -> bool:
        """Attempt to reconnect to the robot with camera fallback."""
        now = self.get_clock().now()
        
        # Check cooldown period
        if self.last_reconnection_time is not None:
            time_since_last = (now - self.last_reconnection_time).nanoseconds / 1e9
            if time_since_last < self.reconnection_cooldown:
                self.get_logger().debug(f'Reconnection cooldown: {self.reconnection_cooldown - time_since_last:.1f}s remaining')
                return False
        
        # Check max attempts
        if self.reconnection_attempt_count >= self.max_reconnection_attempts:
            self.get_logger().warn('Max reconnection attempts reached. Resetting counter.')
            self.reconnection_attempt_count = 0
            return False
        
        self.last_reconnection_time = now
        self.reconnection_attempt_count += 1
        
        self.get_logger().info(f'Reconnection attempt {self.reconnection_attempt_count}/{self.max_reconnection_attempts}')
        
        try:
            # Disconnect current robot
            self.get_logger().info('Disconnecting current robot connection...')
            try:
                self.robot.disconnect()
            except Exception as e:
                self.get_logger().warn(f'Error during disconnect: {e}')
            
            # Try to reconnect with fallback devices
            connected = False
            
            # Try all combinations of camera devices
            for front_idx, front_device in enumerate(self.front_camera_device_list):
                if connected:
                    break
                    
                for wrist_idx, wrist_device in enumerate(self.wrist_camera_device_list):
                    front_device = front_device.strip()
                    wrist_device = wrist_device.strip()
                    
                    self.get_logger().info(f'Trying front: {front_device}, wrist: {wrist_device}')
                    
                    try:
                        # Update camera config
                        if self.enable_cameras:
                            self.robot_config.cameras['front'].index_or_path = front_device
                            self.robot_config.cameras['wrist'].index_or_path = wrist_device
                        
                        # Create new robot instance
                        self.robot = self._robot_class(self.robot_config)
                        self.robot.connect()
                        
                        self.get_logger().info(f'Reconnected with front: {front_device}, wrist: {wrist_device}')
                        connected = True
                        self.reconnection_attempt_count = 0  # Reset on success
                        break
                        
                    except Exception as e:
                        self.get_logger().debug(f'Failed with front: {front_device}, wrist: {wrist_device} - {e}')
                        continue
            
            # If all camera combinations failed, try without cameras
            if not connected and self.allow_camera_failure:
                self.get_logger().warn('All camera combinations failed. Reconnecting without cameras...')
                try:
                    self.robot_config.cameras = {}
                    self.robot = self._robot_class(self.robot_config)
                    self.robot.connect()
                    self.enable_cameras = False
                    self.get_logger().warn('Reconnected WITHOUT cameras')
                    connected = True
                    self.reconnection_attempt_count = 0
                except Exception as e:
                    self.get_logger().error(f'Failed to reconnect without cameras: {e}')
            
            return connected
            
        except Exception as e:
            self.get_logger().error(f'Reconnection failed: {e}')
            return False
    
    def _log_camera_error(self, camera_name: str, error_msg: str, log_level: str = 'warn'):
        """Log camera errors with throttling to avoid spamming logs."""
        now = self.get_clock().now()
        
        # Increment error count
        self.camera_error_counts[camera_name] += 1
        
        # Check if we should log (first error or after interval)
        should_log = False
        if self.camera_last_error_time[camera_name] is None:
            should_log = True
        else:
            time_since_last_log = (now - self.camera_last_error_time[camera_name]).nanoseconds / 1e9
            if time_since_last_log >= self.camera_error_log_interval:
                should_log = True
        
        if should_log:
            error_count = self.camera_error_counts[camera_name]
            msg = f'{camera_name.capitalize()} camera error: {error_msg}'
            if error_count > 1:
                msg += f' (occurred {error_count} times)'
            
            if log_level == 'warn':
                self.get_logger().warn(msg)
            elif log_level == 'debug':
                self.get_logger().debug(msg)
            elif log_level == 'error':
                self.get_logger().error(msg)
            
            self.camera_last_error_time[camera_name] = now
            self.camera_error_counts[camera_name] = 0  # Reset count after logging
    
    def publish_camera_images(self, observation: dict):
        """Publish camera images as compressed images."""
        if not self.enable_cameras:
            return
        
        # Front camera
        if 'front' in observation and isinstance(observation['front'], np.ndarray):
            try:
                front_image = observation['front']
                
                # Validate image before processing
                if front_image.size == 0 or front_image.shape[0] == 0 or front_image.shape[1] == 0:
                    self._log_camera_error('front', 'Invalid image dimensions')
                    return
                
                front_msg = CompressedImage()
                front_msg.header = Header()
                front_msg.header.stamp = self.get_clock().now().to_msg()
                front_msg.header.frame_id = 'front_camera'
                front_msg.format = 'jpeg'
                
                # Apply rotation if camera is mounted upside down
                if self.rotate_front_camera:
                    front_image = cv2.rotate(front_image, cv2.ROTATE_180)
                
                # Convert RGB to BGR for OpenCV encoding
                # LeRobot returns images in RGB format, but cv2.imencode expects BGR
                front_bgr = cv2.cvtColor(front_image, cv2.COLOR_RGB2BGR)
                
                # Encode image as JPEG
                ret, buffer = cv2.imencode('.jpg', front_bgr, 
                                          [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ret:
                    front_msg.data = buffer.tobytes()
                    self.front_camera_pub.publish(front_msg)
                    # Reset error count on successful publish
                    self.camera_error_counts['front'] = 0
                else:
                    self._log_camera_error('front', 'JPEG encoding failed')
            except Exception as e:
                self._log_camera_error('front', str(e))
        else:
            # Camera data not available in observation
            self._log_camera_error('front', 'Camera data not available in observation', log_level='debug')
        
        # Wrist camera
        if 'wrist' in observation and isinstance(observation['wrist'], np.ndarray):
            try:
                wrist_image = observation['wrist']
                
                # Validate image before processing
                if wrist_image.size == 0 or wrist_image.shape[0] == 0 or wrist_image.shape[1] == 0:
                    self._log_camera_error('wrist', 'Invalid image dimensions')
                    return
                
                wrist_msg = CompressedImage()
                wrist_msg.header = Header()
                wrist_msg.header.stamp = self.get_clock().now().to_msg()
                wrist_msg.header.frame_id = 'wrist_camera'
                wrist_msg.format = 'jpeg'
                
                # Convert RGB to BGR for OpenCV encoding
                # LeRobot returns images in RGB format, but cv2.imencode expects BGR
                wrist_bgr = cv2.cvtColor(wrist_image, cv2.COLOR_RGB2BGR)
                
                # Encode image as JPEG
                ret, buffer = cv2.imencode('.jpg', wrist_bgr,
                                          [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ret:
                    wrist_msg.data = buffer.tobytes()
                    self.wrist_camera_pub.publish(wrist_msg)
                    # Reset error count on successful publish
                    self.camera_error_counts['wrist'] = 0
                else:
                    self._log_camera_error('wrist', 'JPEG encoding failed')
            except Exception as e:
                self._log_camera_error('wrist', str(e))
        else:
            # Camera data not available in observation
            self._log_camera_error('wrist', 'Camera data not available in observation', log_level='debug')
    
    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down LeKiwi teleoperation node...')
        try:
            self.robot.disconnect()
            self.get_logger().info('Robot disconnected successfully')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = LeKiwiTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Keyboard interrupt received. Exiting...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
