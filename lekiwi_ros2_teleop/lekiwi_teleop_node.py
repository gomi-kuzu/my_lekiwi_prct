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
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
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
        
        # Get parameters
        robot_port = self.get_parameter('robot_port').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.watchdog_timeout_ms = self.get_parameter('watchdog_timeout_ms').value
        use_degrees = self.get_parameter('use_degrees').value
        self.rotate_front_camera = self.get_parameter('rotate_front_camera').value
        
        # Configure robot
        self.get_logger().info('Configuring LeKiwi robot...')
        robot_config = LeKiwiConfig(
            port=robot_port,
            use_degrees=use_degrees,
            disable_torque_on_disconnect=True,
        )
        self.robot = LeKiwi(robot_config)
        
        # Connect to robot
        self.get_logger().info('Connecting to LeKiwi robot...')
        try:
            self.robot.connect()
            self.get_logger().info('LeKiwi robot connected successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            raise
        
        # Initialize command tracking
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_active = False
        
        # Initialize last action state
        self.last_arm_positions = {
            "arm_shoulder_pan.pos": -5.0,
            "arm_shoulder_lift.pos": -78.0,
            "arm_elbow_flex.pos": 82.0,
            "arm_wrist_flex.pos": 62.0,
            "arm_wrist_roll.pos": 2.5,
            "arm_gripper.pos": 1.5,
        }
        self.last_base_velocities = {
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }
        
        # QoS profile for real-time control
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers
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
    
    def cmd_vel_callback(self, msg: Twist):
        """Callback for base velocity commands."""
        self.last_base_velocities = {
            "x.vel": msg.linear.x,
            "y.vel": msg.linear.y,
            "theta.vel": np.degrees(msg.angular.z),  # Convert rad/s to deg/s
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
                self.get_logger().warn(
                    f'Command not received for more than {self.watchdog_timeout_ms} ms. Stopping the base.'
                )
                self.watchdog_active = True
                self.robot.stop_base()
                # Reset base velocities
                self.last_base_velocities = {
                    "x.vel": 0.0,
                    "y.vel": 0.0,
                    "theta.vel": 0.0,
                }
            
            # Combine arm and base commands
            action = {**self.last_arm_positions, **self.last_base_velocities}
            
            # Send action to robot
            self.robot.send_action(action)
            
            # Get observations
            observation = self.robot.get_observation()
            
            # Publish joint states
            self.publish_joint_states(observation)
            
            # Publish camera images
            self.publish_camera_images(observation)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
    
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
    
    def publish_camera_images(self, observation: dict):
        """Publish camera images as compressed images."""
        # Front camera
        if 'front' in observation and isinstance(observation['front'], np.ndarray):
            try:
                front_msg = CompressedImage()
                front_msg.header = Header()
                front_msg.header.stamp = self.get_clock().now().to_msg()
                front_msg.header.frame_id = 'front_camera'
                front_msg.format = 'jpeg'
                
                # Apply rotation if camera is mounted upside down
                front_image = observation['front']
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
            except Exception as e:
                self.get_logger().warn(f'Failed to publish front camera image: {e}')
        
        # Wrist camera
        if 'wrist' in observation and isinstance(observation['wrist'], np.ndarray):
            try:
                wrist_msg = CompressedImage()
                wrist_msg.header = Header()
                wrist_msg.header.stamp = self.get_clock().now().to_msg()
                wrist_msg.header.frame_id = 'wrist_camera'
                wrist_msg.format = 'jpeg'
                
                # Convert RGB to BGR for OpenCV encoding
                # LeRobot returns images in RGB format, but cv2.imencode expects BGR
                wrist_bgr = cv2.cvtColor(observation['wrist'], cv2.COLOR_RGB2BGR)
                
                # Encode image as JPEG
                ret, buffer = cv2.imencode('.jpg', wrist_bgr,
                                          [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ret:
                    wrist_msg.data = buffer.tobytes()
                    self.wrist_camera_pub.publish(wrist_msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to publish wrist camera image: {e}')
    
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
