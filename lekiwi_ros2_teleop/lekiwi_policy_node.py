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
ROS2 Policy Inference Node for LeKiwi Robot

This node loads a trained policy model and performs inference to control the robot.
It runs on a desktop PC with GPU and communicates with the robot via ROS2 topics.
The robot control is handled by lekiwi_teleop_node running on the Raspberry Pi.

Based on lerobot_record.py's policy inference functionality.
"""

import json
import os
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

# Import LeRobot components
lerobot_path = os.environ.get('LEROBOT_PATH')
if lerobot_path is None:
    raise EnvironmentError(
        "LEROBOT_PATH environment variable is not set. "
        "Please set it to the path of your LeRobot installation's src directory. "
        "Example: export LEROBOT_PATH='/path/to/lerobot/src'"
    )
if lerobot_path not in sys.path:
    sys.path.append(lerobot_path)

# Monkey-patch to avoid HuggingFace Hub API calls
from lerobot.datasets import utils as lerobot_utils
_original_get_safe_version = lerobot_utils.get_safe_version

def _get_safe_version_offline(repo_id: str, version: str):
    """Return the version without checking HuggingFace Hub."""
    return version if isinstance(version, str) else str(version)

lerobot_utils.get_safe_version = _get_safe_version_offline

from huggingface_hub import snapshot_download as _original_snapshot_download

def _snapshot_download_local(repo_id, *, repo_type='dataset', revision=None, local_dir=None, **kwargs):
    """Return local directory without attempting to download from Hub."""
    if local_dir:
        local_path = Path(local_dir) / repo_id
        if local_path.exists():
            return str(local_path)
        return local_dir
    return str(Path.home() / '.cache' / 'huggingface' / 'lerobot' / repo_id)

import huggingface_hub
huggingface_hub.snapshot_download = _snapshot_download_local

# Import LeRobot classes
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import lerobot.datasets.lerobot_dataset as lerobot_dataset_module
lerobot_dataset_module.get_safe_version = _get_safe_version_offline

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.processor import PolicyProcessorPipeline, RobotAction, RobotObservation
from lerobot.processor.rename_processor import rename_stats
from lerobot.datasets.utils import build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.utils import get_safe_torch_device
from lerobot.utils.control_utils import predict_action


class LeKiwiPolicyNode(Node):
    """
    ROS2 Node for LeKiwi robot policy inference.
    
    This node runs on a desktop PC with GPU and communicates with the robot
    via ROS2 topics. It subscribes to observations from lekiwi_teleop_node
    and publishes action commands.
    
    Subscribes to:
        - /lekiwi/joint_states (sensor_msgs/JointState): Current robot state
        - /lekiwi/camera/front/image_raw/compressed: Front camera images
        - /lekiwi/camera/wrist/image_raw/compressed: Wrist camera images
    
    Publishes:
        - /lekiwi/cmd_vel (geometry_msgs/Twist): Base velocity commands
        - /lekiwi/arm_joint_commands (sensor_msgs/JointState): Arm joint commands
    
    Services:
        - /lekiwi/policy/start (std_srvs/Trigger): Start policy inference
        - /lekiwi/policy/stop (std_srvs/Trigger): Stop policy inference
    """
    
    def __init__(self):
        super().__init__('lekiwi_policy_node')
        
        # Declare parameters
        self.declare_parameter('policy_path', '')
        self.declare_parameter('dataset_repo_id', '')
        self.declare_parameter('dataset_root', str(Path.home() / 'lerobot_datasets'))
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('single_task', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_amp', False)
        
        # Get parameters
        policy_path_raw = self.get_parameter('policy_path').value
        self.policy_path = str(Path(policy_path_raw).expanduser().resolve())
        self.dataset_repo_id = self.get_parameter('dataset_repo_id').value
        self.dataset_root = Path(self.get_parameter('dataset_root').value).expanduser()
        self.control_frequency = self.get_parameter('control_frequency').value
        self.single_task = self.get_parameter('single_task').value
        device = self.get_parameter('device').value
        use_amp = self.get_parameter('use_amp').value
        
        # Validate parameters
        if not self.policy_path:
            self.get_logger().error('policy_path parameter is required!')
            raise ValueError('policy_path parameter must be set')
        
        if not self.dataset_repo_id:
            self.get_logger().error('dataset_repo_id parameter is required!')
            raise ValueError('dataset_repo_id parameter must be set')
        
        # State variables
        self.is_running = False
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        self.dataset = None
        
        # Received observations from ROS topics
        self.latest_joint_state: Optional[JointState] = None
        self.latest_front_image: Optional[np.ndarray] = None
        self.latest_wrist_image: Optional[np.ndarray] = None
        self.observation_ready = False
        
        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers (commands to robot)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/lekiwi/cmd_vel', qos_profile
        )
        self.arm_joint_cmd_pub = self.create_publisher(
            JointState, '/lekiwi/arm_joint_commands', qos_profile
        )
        
        # Subscribers (observations from robot)
        self.joint_state_sub = self.create_subscription(
            JointState, '/lekiwi/joint_states', self.joint_state_callback, qos_profile
        )
        self.front_camera_sub = self.create_subscription(
            CompressedImage, '/lekiwi/camera/front/image_raw/compressed', 
            self.front_camera_callback, qos_profile
        )
        self.wrist_camera_sub = self.create_subscription(
            CompressedImage, '/lekiwi/camera/wrist/image_raw/compressed',
            self.wrist_camera_callback, qos_profile
        )
        
        # Services
        self.start_srv = self.create_service(
            Trigger, '/lekiwi/policy/start', self.start_callback
        )
        self.stop_srv = self.create_service(
            Trigger, '/lekiwi/policy/stop', self.stop_callback
        )
        
        # Initialize components
        self.get_logger().info('Initializing policy node...')
        self._load_dataset()
        self._load_policy()
        
        self.get_logger().info('LeKiwi Policy Node initialized successfully!')
        self.get_logger().info(f'Policy path: {self.policy_path}')
        self.get_logger().info(f'Dataset: {self.dataset_repo_id}')
        self.get_logger().info(f'Task: {self.single_task}')
        self.get_logger().info('Waiting for observations from /lekiwi/joint_states and camera topics...')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for receiving joint states from the robot."""
        self.latest_joint_state = msg
        self._check_observation_ready()
    
    def front_camera_callback(self, msg: CompressedImage):
        """Callback for receiving front camera images."""
        try:
            # Decode JPEG image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert BGR to RGB (LeRobot expects RGB)
            self.latest_front_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            self._check_observation_ready()
        except Exception as e:
            self.get_logger().warn(f'Failed to decode front camera image: {e}')
    
    def wrist_camera_callback(self, msg: CompressedImage):
        """Callback for receiving wrist camera images."""
        try:
            # Decode JPEG image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # Convert BGR to RGB (LeRobot expects RGB)
            self.latest_wrist_image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            self._check_observation_ready()
        except Exception as e:
            self.get_logger().warn(f'Failed to decode wrist camera image: {e}')
    
    def _check_observation_ready(self):
        """Check if all observation data is available."""
        if (self.latest_joint_state is not None and 
            self.latest_front_image is not None and 
            self.latest_wrist_image is not None):
            if not self.observation_ready:
                self.get_logger().info('All observations received. Ready for inference.')
            self.observation_ready = True
    
    def _load_dataset(self):
        """Load the dataset to get metadata and features."""
        self.get_logger().info(f'Loading dataset: {self.dataset_repo_id}')
        
        try:
            self.dataset = LeRobotDataset(
                self.dataset_repo_id,
                root=str(self.dataset_root),
            )
            self.get_logger().info(f'Dataset loaded: {len(self.dataset)} frames')
            
            # Log expected feature names for debugging
            if 'observation.state' in self.dataset.features:
                state_names = self.dataset.features['observation.state'].get('names', [])
                self.get_logger().info(f'Expected observation.state names: {state_names}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load dataset: {e}')
            raise
    
    def _load_policy(self):
        """Load the pretrained policy and processors."""
        self.get_logger().info(f'Loading policy from: {self.policy_path}')
        
        try:
            # Load config.json and remove deprecated fields
            config_path = Path(self.policy_path) / 'config.json'
            with open(config_path, 'r') as f:
                config_dict = json.load(f)
            
            # Remove deprecated fields that are not compatible with current LeRobot
            deprecated_fields = ['use_peft', 'push_to_hub', 'repo_id', 'private', 'tags', 'license']
            removed_fields = []
            for field in deprecated_fields:
                if field in config_dict:
                    config_dict.pop(field)
                    removed_fields.append(field)
            
            if removed_fields:
                self.get_logger().warn(f'Removed deprecated config fields: {removed_fields}')
            
            # Create a temporary config file without deprecated fields
            import tempfile
            with tempfile.TemporaryDirectory() as tmpdir:
                tmp_config_path = Path(tmpdir) / 'config.json'
                with open(tmp_config_path, 'w') as f:
                    json.dump(config_dict, f, indent=2)
                
                # Load config using from_pretrained (which handles type-specific configs like ACTConfig)
                policy_cfg = PreTrainedConfig.from_pretrained(tmpdir)
            
            # Set the correct pretrained_path to the original model directory
            policy_cfg.pretrained_path = self.policy_path
            
            # Override device settings
            policy_cfg.device = self.get_parameter('device').value
            policy_cfg.use_amp = self.get_parameter('use_amp').value
            
            # Create policy
            self.policy = make_policy(policy_cfg, ds_meta=self.dataset.meta)
            
            # Create preprocessor and postprocessor
            self.preprocessor, self.postprocessor = make_pre_post_processors(
                policy_cfg=policy_cfg,
                pretrained_path=self.policy_path,
                dataset_stats=self.dataset.meta.stats,
                preprocessor_overrides={
                    "device_processor": {"device": policy_cfg.device},
                },
            )
            
            self.get_logger().info('Policy loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load policy: {e}')
            raise
    
    def start_callback(self, request, response):
        """Start policy inference."""
        if self.is_running:
            response.success = False
            response.message = 'Policy inference already running'
            return response
        
        self.get_logger().info('Starting policy inference...')
        self.is_running = True
        
        # Reset policy and processors
        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()
        
        # Create control timer
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        response.success = True
        response.message = 'Policy inference started'
        return response
    
    def stop_callback(self, request, response):
        """Stop policy inference."""
        if not self.is_running:
            response.success = False
            response.message = 'Policy inference not running'
            return response
        
        self.get_logger().info('Stopping policy inference...')
        self.is_running = False
        
        # Destroy control timer
        if hasattr(self, 'control_timer'):
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)
        
        response.success = True
        response.message = 'Policy inference stopped'
        return response
    
    def control_loop(self):
        """Main control loop - get observation from ROS topics, predict action, publish commands."""
        if not self.is_running:
            return
        
        # Check if observations are available
        if not self.observation_ready:
            self.get_logger().warn('Observations not ready yet. Waiting...', throttle_duration_sec=5.0)
            return
        
        try:
            self.get_logger().debug('Control loop: building observation...', throttle_duration_sec=1.0)
            
            # Build observation from ROS topic data
            obs = self._build_observation_from_topics()
            
            self.get_logger().debug('Control loop: building observation frame...', throttle_duration_sec=1.0)
            
            # Build observation frame for policy
            observation_frame = build_dataset_frame(
                self.dataset.features, 
                obs, 
                prefix=OBS_STR
            )
            
            self.get_logger().debug('Control loop: predicting action...', throttle_duration_sec=1.0)
            
            # Predict action using policy
            action_values = predict_action(
                observation=observation_frame,
                policy=self.policy,
                device=get_safe_torch_device(self.policy.config.device),
                preprocessor=self.preprocessor,
                postprocessor=self.postprocessor,
                use_amp=self.policy.config.use_amp,
                task=self.single_task,
                robot_type='lekiwi',  # Robot type for LeKiwi
            )
            
            self.get_logger().info(f'Predicted action: {action_values}', throttle_duration_sec=1.0)
            
            # Convert to robot action
            robot_action = make_robot_action(action_values, self.dataset.features)
            
            self.get_logger().info(f'Robot action keys: {robot_action.keys() if isinstance(robot_action, dict) else type(robot_action)}', throttle_duration_sec=1.0)
            self.get_logger().info(f'Robot action: {robot_action}', throttle_duration_sec=1.0)
            
            self.get_logger().debug('Control loop: publishing commands...', throttle_duration_sec=1.0)
            
            # Publish action commands via ROS topics
            self._publish_commands(robot_action)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.is_running = False
            if hasattr(self, 'control_timer'):
                self.control_timer.cancel()
    
    def _build_observation_from_topics(self) -> RobotObservation:
        """Build observation dictionary from ROS topic data.
        
        Matches the format used in lekiwi_data_recorder.py:
        - arm_joint_0 ~ arm_joint_5 (6 joints, even though physical arm has only 5)
        - gripper_position
        - base_linear_x/y/z
        - base_angular_x/y/z
        Total: 13 dimensions
        """
        obs = {}
        
        # Extract arm joint positions (pad to 6 even if arm has only 5 joints)
        joint_positions = list(self.latest_joint_state.position) if self.latest_joint_state else []
        
        # Arm has 5 joints but dataset expects 6 - pad with last joint value or 0
        if len(joint_positions) >= 6:
            arm_positions = joint_positions[:6]
            gripper_pos = joint_positions[6] if len(joint_positions) > 6 else 0.0
        elif len(joint_positions) == 5:
            # 5-axis arm: duplicate last joint to make 6
            arm_positions = joint_positions[:5] + [joint_positions[4]]
            gripper_pos = 0.0
        else:
            # Insufficient data, pad with zeros
            arm_positions = joint_positions + [0.0] * (6 - len(joint_positions))
            gripper_pos = 0.0
        
        # Build state according to dataset feature names
        obs['arm_joint_0'] = arm_positions[0]
        obs['arm_joint_1'] = arm_positions[1]
        obs['arm_joint_2'] = arm_positions[2]
        obs['arm_joint_3'] = arm_positions[3]
        obs['arm_joint_4'] = arm_positions[4]
        obs['arm_joint_5'] = arm_positions[5]
        obs['gripper_position'] = gripper_pos
        
        # Base velocities - set to zero for observation (not using cmd_vel for observation)
        # The dataset was recorded with base velocities from cmd_vel, but during inference
        # we don't have actual base velocity sensors, so we use zero
        obs['base_linear_x'] = 0.0
        obs['base_linear_y'] = 0.0
        obs['base_linear_z'] = 0.0
        obs['base_angular_x'] = 0.0
        obs['base_angular_y'] = 0.0
        obs['base_angular_z'] = 0.0
        
        # Add camera images - use simple keys without 'observation.images.' prefix
        # build_dataset_frame() will add the prefix automatically
        if self.latest_front_image is not None:
            obs['front'] = self.latest_front_image
        
        if self.latest_wrist_image is not None:
            obs['wrist'] = self.latest_wrist_image
        
        return obs
    
    def _publish_commands(self, action: RobotAction):
        """Publish action commands to ROS topics."""
        timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'Publishing commands - action keys: {action.keys()}', throttle_duration_sec=1.0)
        
        # Extract arm joint commands from action dict
        # Action dict has keys like 'arm_joint_0_cmd', 'arm_joint_1_cmd', etc.
        arm_joint_keys = ['arm_joint_0_cmd', 'arm_joint_1_cmd', 'arm_joint_2_cmd', 
                          'arm_joint_3_cmd', 'arm_joint_4_cmd', 'arm_joint_5_cmd', 'gripper_cmd']
        
        if all(key in action for key in arm_joint_keys):
            arm_cmd = JointState()
            arm_cmd.header.stamp = timestamp
            arm_cmd.header.frame_id = 'base_link'
            arm_cmd.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            
            # Extract arm positions from action dict
            arm_cmd.position = [
                float(action['arm_joint_0_cmd']),
                float(action['arm_joint_1_cmd']),
                float(action['arm_joint_2_cmd']),
                float(action['arm_joint_3_cmd']),
                float(action['arm_joint_4_cmd']),
                float(action['arm_joint_5_cmd']),
                # float(action['gripper_cmd'])  # Skip gripper for 5-axis arm
            ]
            
            self.get_logger().info(f'Publishing arm command: {arm_cmd.position}', throttle_duration_sec=1.0)
            self.arm_joint_cmd_pub.publish(arm_cmd)
        else:
            self.get_logger().warn(f'Missing arm joint commands in action. Available keys: {list(action.keys())}')
        
        # Extract base velocity commands
        base_keys = ['base_linear_x_cmd', 'base_linear_y_cmd', 'base_angular_z_cmd']
        
        if all(key in action for key in base_keys):
            cmd_vel = Twist()
            cmd_vel.linear.x = float(action['base_linear_x_cmd'])
            cmd_vel.linear.y = float(action['base_linear_y_cmd'])
            cmd_vel.angular.z = float(action['base_angular_z_cmd'])
            
            self.get_logger().debug(f'Publishing base command: linear=({cmd_vel.linear.x}, {cmd_vel.linear.y}), angular.z={cmd_vel.angular.z}', throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            self.get_logger().debug(f'Missing base commands in action (this is normal). Available keys: {list(action.keys())}', throttle_duration_sec=5.0)
    
    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down policy node...')
        
        if self.is_running:
            self.is_running = False
            if hasattr(self, 'control_timer'):
                self.control_timer.cancel()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LeKiwiPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
