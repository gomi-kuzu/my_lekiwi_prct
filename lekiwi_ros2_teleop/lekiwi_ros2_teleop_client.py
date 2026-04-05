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
ROS2 Teleoperation Client Node for LeKiwi Robot

This node connects to SO100 Leader Arm and Keyboard teleoperators,
and publishes their commands to ROS2 topics for controlling a LeKiwi robot
via lekiwi_teleop_node.
"""

import os
import sys
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import Header

# Import LeRobot teleoperator components
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

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig


class LeKiwiROS2TeleopClient(Node):
    """
    ROS2 Node for LeKiwi robot teleoperation via SO100 Leader Arm and Keyboard.
    
    Publishes to:
        - /lekiwi/cmd_vel (geometry_msgs/Twist): Base velocity commands
        - /lekiwi/arm_joint_commands (sensor_msgs/JointState): Arm joint position commands
    
    Optionally subscribes to:
        - /lekiwi/joint_states (sensor_msgs/JointState): Current robot state (for display)
        - /lekiwi/camera/front/image_raw/compressed (sensor_msgs/CompressedImage): Front camera
        - /lekiwi/camera/wrist/image_raw/compressed (sensor_msgs/CompressedImage): Wrist camera
    """
    
    def __init__(self):
        super().__init__('lekiwi_ros2_teleop_client')
        
        # Declare parameters
        self.declare_parameter('lekiwi_remote_ip', '')  # Will use env var if empty
        self.declare_parameter('lekiwi_id', 'my_lekiwi')
        self.declare_parameter('leader_arm_port', '/dev/tty.usbmodem585A0077581')
        self.declare_parameter('leader_arm_id', 'my_leader_arm')
        self.declare_parameter('keyboard_id', 'my_keyboard')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('use_rerun', False)
        self.declare_parameter('use_keyboard', True)  # Enable/disable keyboard teleop
        
        # Get parameters
        lekiwi_ip = self.get_parameter('lekiwi_remote_ip').value
        lekiwi_id = self.get_parameter('lekiwi_id').value
        leader_arm_port = self.get_parameter('leader_arm_port').value
        leader_arm_id = self.get_parameter('leader_arm_id').value
        keyboard_id = self.get_parameter('keyboard_id').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.use_rerun = self.get_parameter('use_rerun').value
        self.use_keyboard = self.get_parameter('use_keyboard').value
        
        # Get LeKiwi IP from environment variable if not specified in parameter
        if not lekiwi_ip:
            lekiwi_ip = os.environ.get('LEKIWI_REMOTE_IP')
            if lekiwi_ip is None:
                raise EnvironmentError(
                    "LEKIWI_REMOTE_IP environment variable is not set and no parameter provided. "
                    "Please set it to the IP address of your LeKiwi robot. "
                    "Example: export LEKIWI_REMOTE_IP='172.18.134.136'"
                )
        
        self.get_logger().info(f'Connecting to LeKiwi at {lekiwi_ip}')
        
        # Initialize LeKiwi client (for keyboard mapping helper and optionally rerun)
        self.get_logger().info('Initializing LeKiwi client...')
        robot_config = LeKiwiClientConfig(remote_ip=lekiwi_ip, id=lekiwi_id)
        self.robot = LeKiwiClient(robot_config)
        
        # Initialize teleoperators
        self.get_logger().info('Initializing SO100 Leader Arm...')
        teleop_arm_config = SO100LeaderConfig(port=leader_arm_port, id=leader_arm_id)
        self.leader_arm = SO100Leader(teleop_arm_config)
        
        if self.use_keyboard:
            self.get_logger().info('Initializing Keyboard Teleop...')
            keyboard_config = KeyboardTeleopConfig(id=keyboard_id)
            self.keyboard = KeyboardTeleop(keyboard_config)
        else:
            self.get_logger().info('Keyboard teleop disabled - use external teleop_twist_keyboard instead')
            self.keyboard = None
        
        # Connect teleoperators
        try:
            self.get_logger().info('Connecting to SO100 Leader Arm...')
            # Try to connect with retry
            max_retries = 3
            retry_delay = 1.0
            for attempt in range(max_retries):
                try:
                    self.leader_arm.connect()
                    self.get_logger().info('Leader arm connected successfully!')
                    break
                except Exception as connect_error:
                    if attempt < max_retries - 1:
                        self.get_logger().warn(
                            f'Failed to connect to leader arm (attempt {attempt + 1}/{max_retries}): {connect_error}'
                        )
                        self.get_logger().info(f'Retrying in {retry_delay} seconds...')
                        time.sleep(retry_delay)
                    else:
                        raise
        except Exception as e:
            self.get_logger().error(f'Failed to connect to leader arm after {max_retries} attempts: {e}')
            self.get_logger().error('Please check:')
            self.get_logger().error(f'  - Serial port {leader_arm_port} is correct')
            self.get_logger().error('  - Device is powered on and connected')
            self.get_logger().error('  - No other process is using the serial port')
            self.get_logger().error('  - User has permission to access the serial port')
            raise
        
        if self.use_keyboard and self.keyboard is not None:
            try:
                self.get_logger().info('Connecting to Keyboard...')
                self.keyboard.connect()
                if self.keyboard.is_connected:
                    self.get_logger().info('Keyboard connected successfully!')
                else:
                    self.get_logger().warn('Keyboard connect() called, but is_connected=False (pynput may not be available)')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to keyboard: {e}')
                raise
        
        # Initialize Rerun if requested
        if self.use_rerun:
            try:
                from lerobot.utils.visualization_utils import init_rerun
                init_rerun(session_name="lekiwi_ros2_teleop")
                self.get_logger().info('Rerun visualization initialized')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize Rerun: {e}')
                self.use_rerun = False
        
        # QoS profile for real-time control
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/lekiwi/cmd_vel',
            qos_profile
        )
        
        self.arm_cmd_pub = self.create_publisher(
            JointState,
            '/lekiwi/arm_joint_commands',
            qos_profile
        )
        
        # Create subscribers (for monitoring/visualization)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/lekiwi/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.front_camera_sub = self.create_subscription(
            CompressedImage,
            '/lekiwi/camera/front/image_raw/compressed',
            self.front_camera_callback,
            qos_profile
        )
        
        self.wrist_camera_sub = self.create_subscription(
            CompressedImage,
            '/lekiwi/camera/wrist/image_raw/compressed',
            self.wrist_camera_callback,
            qos_profile
        )
        
        # Store latest observations for rerun
        self.current_observation = {}
        
        # Create timer for main control loop
        timer_period = 1.0 / self.control_frequency
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'LeKiwi ROS2 teleoperation client started at {self.control_frequency} Hz')
        self.get_logger().info('Use keyboard to control base, leader arm to control robot arm')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state observations (for monitoring/visualization)."""
        # Store for rerun visualization
        if len(msg.position) >= 6:
            self.current_observation['arm_shoulder_pan.pos'] = msg.position[0]
            self.current_observation['arm_shoulder_lift.pos'] = msg.position[1]
            self.current_observation['arm_elbow_flex.pos'] = msg.position[2]
            self.current_observation['arm_wrist_flex.pos'] = msg.position[3]
            self.current_observation['arm_wrist_roll.pos'] = msg.position[4]
            self.current_observation['arm_gripper.pos'] = msg.position[5]
        
        if len(msg.velocity) >= 3:
            self.current_observation['x.vel'] = msg.velocity[0]
            self.current_observation['y.vel'] = msg.velocity[1]
            self.current_observation['theta.vel'] = np.degrees(msg.velocity[2])
    
    def front_camera_callback(self, msg: CompressedImage):
        """Callback for front camera observations (for monitoring/visualization)."""
        # Could decode and store for rerun if needed
        pass
    
    def wrist_camera_callback(self, msg: CompressedImage):
        """Callback for wrist camera observations (for monitoring/visualization)."""
        # Could decode and store for rerun if needed
        pass
    
    def control_loop(self):
        """Main control loop executed at control_frequency."""
        try:
            # Get teleop actions
            # Arm action from SO100 Leader
            arm_action = self.leader_arm.get_action()
            
            # Publish arm joint commands
            self._publish_arm_commands(arm_action)
            
            # Keyboard action for base (only if keyboard is enabled)
            if self.use_keyboard and self.keyboard is not None:
                keyboard_keys = self.keyboard.get_action()
                base_action = self.robot._from_keyboard_to_base_action(keyboard_keys)
                # Publish base velocity commands
                self._publish_base_commands(base_action)
            
            # Visualize with rerun if enabled
            if self.use_rerun and len(self.current_observation) > 0:
                try:
                    from lerobot.utils.visualization_utils import log_rerun_data
                    # Combine arm and base actions
                    combined_action = {f"arm_{k}": v for k, v in arm_action.items()}
                    # Only add base action if keyboard is enabled
                    if self.use_keyboard and self.keyboard is not None:
                        keyboard_keys = self.keyboard.get_action()
                        base_action = self.robot._from_keyboard_to_base_action(keyboard_keys)
                        combined_action.update(base_action)
                    log_rerun_data(observation=self.current_observation, action=combined_action)
                except Exception as e:
                    self.get_logger().warn(f'Failed to log rerun data: {e}', throttle_duration_sec=5.0)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
    
    def _publish_arm_commands(self, arm_action: dict):
        """Publish arm joint position commands."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Expected keys from SO100Leader: shoulder_pan.pos, shoulder_lift.pos, 
        # elbow_flex.pos, wrist_flex.pos, wrist_roll.pos, gripper.pos
        msg.name = [
            'arm_shoulder_pan',
            'arm_shoulder_lift',
            'arm_elbow_flex',
            'arm_wrist_flex',
            'arm_wrist_roll',
            'arm_gripper',
        ]
        
        msg.position = [
            arm_action.get('shoulder_pan.pos', 0.0),
            arm_action.get('shoulder_lift.pos', 0.0),
            arm_action.get('elbow_flex.pos', 0.0),
            arm_action.get('wrist_flex.pos', 0.0),
            arm_action.get('wrist_roll.pos', 0.0),
            arm_action.get('gripper.pos', 0.0),
        ]
        
        # Debug: Log gripper position periodically
        # current_time = self.get_clock().now()
        # if not hasattr(self, '_last_gripper_log_time'):
        #     self._last_gripper_log_time = current_time
        #     self._frame_count = 0
        # 
        # self._frame_count += 1
        # if self._frame_count % 30 == 0:  # Log every second at 30Hz
        #     self.get_logger().info(
        #         f'[ARM_CMD] Gripper pos from leader_arm: {arm_action.get("gripper.pos", "NOT_FOUND")} | '
        #         f'All positions: {msg.position}',
        #         throttle_duration_sec=1.0
        #     )
        
        self.arm_cmd_pub.publish(msg)
    
    def _publish_base_commands(self, base_action: dict):
        """Publish base velocity commands."""
        msg = Twist()
        
        # Expected keys: x.vel, y.vel, theta.vel (in degrees/s)
        msg.linear.x = base_action.get('x.vel', 0.0)
        msg.linear.y = base_action.get('y.vel', 0.0)
        msg.angular.z = np.radians(base_action.get('theta.vel', 0.0))  # Convert deg/s to rad/s
        
        self.cmd_vel_pub.publish(msg)
    
    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down LeKiwi ROS2 teleoperation client...')
        try:
            self.leader_arm.disconnect()
            self.get_logger().info('Leader arm disconnected')
        except Exception as e:
            self.get_logger().error(f'Error disconnecting leader arm: {e}')
        
        if self.use_keyboard and self.keyboard is not None:
            try:
                self.keyboard.disconnect()
                self.get_logger().info('Keyboard disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting keyboard: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = LeKiwiROS2TeleopClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Keyboard interrupt received. Exiting...')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
