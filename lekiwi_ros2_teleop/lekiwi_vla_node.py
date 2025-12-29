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
ROS2 VLA Node for LeKiwi Robot

This node subscribes to observations from lekiwi_teleop_node,
processes them through a VLA (Vision-Language-Action) model,
and publishes the predicted actions.
"""

import os
import sys
from pathlib import Path
from typing import Optional, Dict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import Header
import torch

# Import LeRobot VLA components
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

from lerobot.common.policies.factory import make_policy
from lerobot.common.utils.utils import init_hydra_config


class LeKiwiVLANode(Node):
    """
    ROS2 Node for LeKiwi robot VLA inference.
    
    Subscribes to:
        - /lekiwi/joint_states (sensor_msgs/JointState): Current robot state
        - /lekiwi/camera/front/image_raw/compressed (sensor_msgs/CompressedImage): Front camera
        - /lekiwi/camera/wrist/image_raw/compressed (sensor_msgs/CompressedImage): Wrist camera
    
    Publishes:
        - /lekiwi/cmd_vel (geometry_msgs/Twist): Base velocity commands
        - /lekiwi/arm_joint_commands (sensor_msgs/JointState): Arm joint position commands
    """
    
    def __init__(self):
        super().__init__('lekiwi_vla_node')
        
        # Declare parameters
        self.declare_parameter('policy_path', '')
        self.declare_parameter('inference_frequency', 10.0)
        self.declare_parameter('task_instruction', 'Pick up the object')
        self.declare_parameter('use_cuda', True)
        self.declare_parameter('temporal_ensemble_coeff', 0.01)
        
        # Get parameters
        policy_path = self.get_parameter('policy_path').value
        self.inference_frequency = self.get_parameter('inference_frequency').value
        self.task_instruction = self.get_parameter('task_instruction').value
        use_cuda = self.get_parameter('use_cuda').value
        self.temporal_ensemble_coeff = self.get_parameter('temporal_ensemble_coeff').value
        
        # Set device
        self.device = torch.device("cuda" if use_cuda and torch.cuda.is_available() else "cpu")
        self.get_logger().info(f'Using device: {self.device}')
        
        # Initialize policy
        if policy_path:
            self.get_logger().info(f'Loading policy from: {policy_path}')
            try:
                self.policy = self._load_policy(policy_path)
                self.get_logger().info('Policy loaded successfully!')
            except Exception as e:
                self.get_logger().error(f'Failed to load policy: {e}')
                raise
        else:
            self.get_logger().error('policy_path parameter is required!')
            raise ValueError('policy_path parameter is required!')
        
        # Initialize observation buffers
        self.current_joint_state: Optional[JointState] = None
        self.front_image: Optional[np.ndarray] = None
        self.wrist_image: Optional[np.ndarray] = None
        
        # Previous action for temporal ensemble
        self.previous_action: Optional[Dict[str, float]] = None
        
        # QoS profile for real-time control
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers
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
        
        # Create timer for inference loop
        timer_period = 1.0 / self.inference_frequency
        self.timer = self.create_timer(timer_period, self.inference_loop)
        
        self.get_logger().info(f'LeKiwi VLA node started at {self.inference_frequency} Hz')
        self.get_logger().info(f'Task instruction: {self.task_instruction}')
    
    def _load_policy(self, policy_path: str):
        """Load the VLA policy from checkpoint."""
        policy_path = Path(policy_path)
        
        # Load configuration
        config_path = policy_path / "config.yaml"
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        
        cfg = init_hydra_config(str(config_path))
        
        # Create policy
        policy = make_policy(
            cfg=cfg.policy,
            pretrained_policy_name_or_path=str(policy_path),
            device=self.device,
        )
        
        policy.eval()
        return policy
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state observations."""
        self.current_joint_state = msg
    
    def front_camera_callback(self, msg: CompressedImage):
        """Callback for front camera observations."""
        try:
            # Decode compressed image (JPEG is in BGR format after decoding)
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert BGR to RGB for policy input
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            # Note: 180-degree rotation is already applied in lekiwi_teleop_node
            self.front_image = image_rgb
        except Exception as e:
            self.get_logger().warn(f'Failed to decode front camera image: {e}')
    
    def wrist_camera_callback(self, msg: CompressedImage):
        """Callback for wrist camera observations."""
        try:
            # Decode compressed image (JPEG is in BGR format after decoding)
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert BGR to RGB for policy input
            self.wrist_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode wrist camera image: {e}')
    
    def inference_loop(self):
        """Main inference loop executed at inference_frequency."""
        # Check if we have all observations
        if self.current_joint_state is None:
            self.get_logger().warn('Waiting for joint state observation...', throttle_duration_sec=5.0)
            return
        
        if self.front_image is None:
            self.get_logger().warn('Waiting for front camera observation...', throttle_duration_sec=5.0)
            return
        
        if self.wrist_image is None:
            self.get_logger().warn('Waiting for wrist camera observation...', throttle_duration_sec=5.0)
            return
        
        try:
            # Prepare observation for policy
            observation = self._prepare_observation()
            
            # Run inference
            with torch.no_grad():
                action = self.policy.select_action(observation)
            
            # Apply temporal ensemble if previous action exists
            if self.previous_action is not None and self.temporal_ensemble_coeff > 0:
                action = self._apply_temporal_ensemble(action, self.previous_action)
            
            # Store current action for next temporal ensemble
            self.previous_action = action.copy()
            
            # Publish action
            self._publish_action(action)
            
        except Exception as e:
            self.get_logger().error(f'Error in inference loop: {e}')
    
    def _prepare_observation(self) -> Dict:
        """Prepare observation dictionary for policy input."""
        observation = {}
        
        # Add images (already in RGB format from callbacks)
        if self.front_image is not None:
            observation['observation.images.front'] = torch.from_numpy(self.front_image).to(self.device)
        
        if self.wrist_image is not None:
            observation['observation.images.wrist'] = torch.from_numpy(self.wrist_image).to(self.device)
        
        # Add state (joint positions and base velocities)
        if self.current_joint_state is not None:
            state = []
            
            # Arm joint positions (6 joints)
            if len(self.current_joint_state.position) >= 6:
                state.extend(self.current_joint_state.position[:6])
            else:
                state.extend([0.0] * 6)
            
            # Base velocities (3 values: x, y, theta)
            if len(self.current_joint_state.velocity) >= 3:
                state.extend(self.current_joint_state.velocity[:3])
            else:
                state.extend([0.0] * 3)
            
            observation['observation.state'] = torch.tensor(state, dtype=torch.float32).to(self.device)
        
        # Add task instruction if policy supports it
        if hasattr(self.policy, 'use_language') and self.policy.use_language:
            observation['task'] = self.task_instruction
        
        return observation
    
    def _apply_temporal_ensemble(self, current_action: Dict, previous_action: Dict) -> Dict:
        """Apply temporal ensemble to smooth actions."""
        ensembled_action = {}
        for key in current_action.keys():
            if key in previous_action:
                ensembled_action[key] = (
                    self.temporal_ensemble_coeff * previous_action[key] + 
                    (1 - self.temporal_ensemble_coeff) * current_action[key]
                )
            else:
                ensembled_action[key] = current_action[key]
        return ensembled_action
    
    def _publish_action(self, action: Dict):
        """Publish action to robot control topics."""
        # Parse action dictionary
        # Expected keys: arm joint positions and base velocities
        
        # Publish arm joint commands
        arm_cmd = JointState()
        arm_cmd.header = Header()
        arm_cmd.header.stamp = self.get_clock().now().to_msg()
        arm_cmd.name = [
            'arm_shoulder_pan',
            'arm_shoulder_lift',
            'arm_elbow_flex',
            'arm_wrist_flex',
            'arm_wrist_roll',
            'arm_gripper',
        ]
        
        # Extract arm positions from action
        # Assuming action is a numpy array or dict with specific keys
        if isinstance(action, np.ndarray):
            # If action is array, first 6 elements are arm joints
            arm_cmd.position = action[:6].tolist()
        elif isinstance(action, dict):
            # Extract from dict based on expected keys
            arm_cmd.position = [
                action.get('arm_shoulder_pan.pos', 0.0),
                action.get('arm_shoulder_lift.pos', 0.0),
                action.get('arm_elbow_flex.pos', 0.0),
                action.get('arm_wrist_flex.pos', 0.0),
                action.get('arm_wrist_roll.pos', 0.0),
                action.get('arm_gripper.pos', 0.0),
            ]
        
        self.arm_cmd_pub.publish(arm_cmd)
        
        # Publish base velocity commands
        cmd_vel = Twist()
        
        if isinstance(action, np.ndarray):
            # If action is array, elements 6-8 are base velocities
            if len(action) > 6:
                cmd_vel.linear.x = float(action[6])
            if len(action) > 7:
                cmd_vel.linear.y = float(action[7])
            if len(action) > 8:
                cmd_vel.angular.z = float(np.radians(action[8]))  # Convert deg/s to rad/s
        elif isinstance(action, dict):
            cmd_vel.linear.x = action.get('x.vel', 0.0)
            cmd_vel.linear.y = action.get('y.vel', 0.0)
            # Convert deg/s to rad/s for theta velocity
            theta_vel_deg = action.get('theta.vel', 0.0)
            cmd_vel.angular.z = np.radians(theta_vel_deg)
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info('Shutting down VLA node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LeKiwiVLANode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
