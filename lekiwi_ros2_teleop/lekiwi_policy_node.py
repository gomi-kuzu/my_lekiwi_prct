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
from lerobot.utils.feature_utils import build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.device_utils import get_safe_torch_device
from lerobot.common.control_utils import predict_action


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
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('single_task', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_amp', False)
        self.declare_parameter('rename_map', '')
        # Inference-time action chunking overrides (no retraining required).
        # n_action_steps: number of actions consumed per policy inference.
        #   0 (default) keeps the trained model's value. Smaller -> re-infer more
        #   often with fresh observations (more reactive / closer to closed-loop).
        #   Must satisfy 1 <= n_action_steps <= chunk_size.
        self.declare_parameter('n_action_steps', 0)
        # temporal_ensemble_coeff: enable ACT temporal ensembling when >= 0.
        #   Negative (default) keeps the trained model's value (usually disabled).
        #   When enabled, n_action_steps is forced to 1 (LeRobot constraint).
        self.declare_parameter('temporal_ensemble_coeff', -1.0)
        
        # Get parameters
        policy_path_raw = self.get_parameter('policy_path').value
        self.policy_path = str(Path(policy_path_raw).expanduser().resolve())
        self.dataset_repo_id = self.get_parameter('dataset_repo_id').value
        self.dataset_root = Path(self.get_parameter('dataset_root').value).expanduser()
        self.control_frequency = self.get_parameter('control_frequency').value
        self.single_task = self.get_parameter('single_task').value
        device = self.get_parameter('device').value
        use_amp = self.get_parameter('use_amp').value
        
        # Parse rename_map (JSON string -> dict)
        rename_map_str = self.get_parameter('rename_map').value
        if rename_map_str:
            self.rename_map = json.loads(rename_map_str)
            self.get_logger().info(f'Using rename_map: {self.rename_map}')
        else:
            self.rename_map = None
        
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

        # Whether the loaded policy/dataset expects camera images.
        # Auto-detected from dataset features in _load_dataset(). When the
        # dataset has no observation.images.* features (image-free policy),
        # the node runs inference without waiting for camera topics.
        self.require_front_image = True
        self.require_wrist_image = True
        # Feature set used to build observation frames. Populated in
        # _configure_image_requirements() after the policy is loaded.
        self.observation_features = None
        
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
        if self.require_front_image or self.require_wrist_image:
            self.get_logger().info(
                'Waiting for observations from /lekiwi/joint_states and camera topics...'
            )
        else:
            self.get_logger().info(
                'Waiting for observations from /lekiwi/joint_states (image-free, no cameras required)...'
            )
    
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
        """Check if all observation data is available.

        Camera images are only required when the loaded policy/dataset
        actually expects them (auto-detected in _load_dataset()). This allows
        image-free policies to run using only joint states.
        """
        if self.latest_joint_state is None:
            return
        if self.require_front_image and self.latest_front_image is None:
            return
        if self.require_wrist_image and self.latest_wrist_image is None:
            return
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

            # Log image features present in the dataset (informational only).
            # Whether the node actually waits for these cameras is decided by
            # the policy config in _configure_image_requirements().
            image_features = {
                k for k in self.dataset.features if k.startswith('observation.images.')
            }
            if image_features:
                self.get_logger().info(
                    f'Dataset image features: {sorted(image_features)}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Failed to load dataset: {e}')
            raise
    
    def _apply_chunking_overrides(self, policy_cfg):
        """Override action chunking parameters at inference time.

        Both ``n_action_steps`` and ``temporal_ensemble_coeff`` are inference-time
        settings for ACT-style policies, so changing them does NOT require
        retraining. They must be applied before ``make_policy`` builds the model.

        Constraints enforced by LeRobot's ACTConfig:
            - 1 <= n_action_steps <= chunk_size
            - n_action_steps must be 1 when temporal ensembling is enabled
        """
        chunk_size = getattr(policy_cfg, 'chunk_size', None)

        # Temporal ensembling: enabled when coeff >= 0.
        te_coeff = self.get_parameter('temporal_ensemble_coeff').value
        if te_coeff is not None and te_coeff >= 0.0:
            if not hasattr(policy_cfg, 'temporal_ensemble_coeff'):
                self.get_logger().warn(
                    'Policy config has no temporal_ensemble_coeff; ignoring override.'
                )
            else:
                policy_cfg.temporal_ensemble_coeff = float(te_coeff)
                # LeRobot requires n_action_steps == 1 with temporal ensembling.
                policy_cfg.n_action_steps = 1
                self.get_logger().info(
                    f'Temporal ensembling enabled (coeff={te_coeff}); '
                    'n_action_steps forced to 1.'
                )
                return

        # n_action_steps override: 0 keeps the trained model's value.
        n_action_steps = self.get_parameter('n_action_steps').value
        if n_action_steps and n_action_steps > 0:
            if not hasattr(policy_cfg, 'n_action_steps'):
                self.get_logger().warn(
                    'Policy config has no n_action_steps; ignoring override.'
                )
            elif chunk_size is not None and n_action_steps > chunk_size:
                self.get_logger().warn(
                    f'n_action_steps={n_action_steps} exceeds chunk_size={chunk_size}; '
                    f'clamping to {chunk_size}.'
                )
                policy_cfg.n_action_steps = chunk_size
            else:
                policy_cfg.n_action_steps = int(n_action_steps)
                self.get_logger().info(
                    f'n_action_steps overridden to {policy_cfg.n_action_steps} '
                    f'(chunk_size={chunk_size}).'
                )
        else:
            self.get_logger().info(
                f'Using trained n_action_steps='
                f'{getattr(policy_cfg, "n_action_steps", "?")} '
                f'(chunk_size={chunk_size}).'
            )

    def _load_policy(self):
        """Load the pretrained policy and processors."""
        self.get_logger().info(f'Loading policy from: {self.policy_path}')
        
        try:
            # Load config.json and remove deprecated fields
            config_path = Path(self.policy_path) / 'config.json'
            with open(config_path, 'r') as f:
                config_dict = json.load(f)
            
            # Remove deprecated fields that are not compatible with current LeRobot
            deprecated_fields = ['use_peft', 'push_to_hub', 'repo_id', 'private', 'tags', 'license',
                                 'compile_model', 'compile_mode']
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
            
            # Override action chunking behavior at inference time (no retraining).
            self._apply_chunking_overrides(policy_cfg)
            
            # Apply rename_map to dataset metadata if provided
            if self.rename_map:
                original_features = self.dataset.meta.info["features"]
                renamed_features = {}
                for k, v in original_features.items():
                    new_key = self.rename_map.get(k, k)
                    renamed_features[new_key] = v
                self.dataset.meta.info["features"] = renamed_features
                self.get_logger().info(f'Renamed dataset features: {list(renamed_features.keys())}')
            
            # Create policy
            self.policy = make_policy(policy_cfg, ds_meta=self.dataset.meta, rename_map=self.rename_map)

            # Determine which camera images the POLICY actually consumes.
            # The dataset may contain image features even when the trained
            # policy ignores them (image-free policy). Use the policy config's
            # image_features so we only wait for cameras the model needs.
            self._configure_image_requirements(policy_cfg)
            
            # Build preprocessor overrides
            preprocessor_overrides = {
                "device_processor": {"device": policy_cfg.device},
            }
            if self.rename_map:
                preprocessor_overrides["rename_observations_processor"] = {
                    "rename_map": self.rename_map
                }
            
            # Create preprocessor and postprocessor
            dataset_stats = self.dataset.meta.stats
            if self.rename_map:
                dataset_stats = rename_stats(dataset_stats, self.rename_map)
            self.preprocessor, self.postprocessor = make_pre_post_processors(
                policy_cfg=policy_cfg,
                pretrained_path=self.policy_path,
                dataset_stats=dataset_stats,
                preprocessor_overrides=preprocessor_overrides,
            )
            
            self.get_logger().info('Policy loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load policy: {e}')
            raise

    def _configure_image_requirements(self, policy_cfg):
        """Decide which camera images are required based on the policy config.

        A policy trained without images (image-free) has no visual entries in
        its ``image_features``/``input_features``, so the node should not wait
        for camera topics even if the dataset still contains image features.
        """
        # policy_cfg.image_features is a list of input feature keys with
        # visual type (e.g. 'observation.images.front'). Fall back to scanning
        # input_features if the property is unavailable.
        image_feature_keys = getattr(policy_cfg, 'image_features', None)
        if image_feature_keys is None:
            input_features = getattr(policy_cfg, 'input_features', {}) or {}
            image_feature_keys = [
                k for k in input_features if 'image' in k.lower()
            ]
        image_feature_keys = set(image_feature_keys)

        # Resolve rename_map so short keys (e.g. camera1) are matched.
        front_feature = 'observation.images.front'
        wrist_feature = 'observation.images.wrist'
        if self.rename_map:
            front_feature = self.rename_map.get(front_feature, front_feature)
            wrist_feature = self.rename_map.get(wrist_feature, wrist_feature)

        self.require_front_image = front_feature in image_feature_keys
        self.require_wrist_image = wrist_feature in image_feature_keys

        # Build the feature set passed to build_dataset_frame(). Dataset may
        # contain image features the policy does not use; drop those so the
        # frame builder does not demand missing image observations.
        self.observation_features = {
            k: v for k, v in self.dataset.features.items()
            if not (k.startswith('observation.images.') and k not in image_feature_keys)
        }

        if not image_feature_keys:
            self.get_logger().info(
                'Policy uses no camera images; running image-free inference '
                '(camera topics not required).'
            )
        else:
            self.get_logger().info(
                f'Policy requires image features: {sorted(image_feature_keys)}'
            )

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
            
            # Build observation frame for policy. Use observation_features
            # (image features the policy ignores are excluded) so image-free
            # policies don't require camera observations.
            observation_frame = build_dataset_frame(
                self.observation_features, 
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
        
        Matches the format used in lekiwi_data_recorder.py (9 dimensions):
        - arm_shoulder_pan.pos ~ arm_wrist_roll.pos (5 joints)
        - arm_gripper.pos (1 gripper)
        - x.vel, y.vel, theta.vel (3 base velocities)
        Total: 9 dimensions
        """
        obs = {}
        
        # Extract arm joint positions (5 joints + 1 gripper)
        joint_positions = list(self.latest_joint_state.position) if self.latest_joint_state else []
        
        if len(joint_positions) >= 6:
            arm_positions = joint_positions[:6]  # 5 arm + gripper
        elif len(joint_positions) == 5:
            arm_positions = joint_positions[:5] + [0.0]  # Add gripper as 0
        else:
            arm_positions = joint_positions + [0.0] * (6 - len(joint_positions))
        
        # Build state according to dataset feature names (match recorder)
        obs['arm_shoulder_pan.pos'] = arm_positions[0]
        obs['arm_shoulder_lift.pos'] = arm_positions[1]
        obs['arm_elbow_flex.pos'] = arm_positions[2]
        obs['arm_wrist_flex.pos'] = arm_positions[3]
        obs['arm_wrist_roll.pos'] = arm_positions[4]
        obs['arm_gripper.pos'] = arm_positions[5]
        
        # Base velocities from joint_state.velocity (実測値)
        # joint_state.velocity contains [x.vel, y.vel, theta.vel] from teleop_node
        if self.latest_joint_state is not None and len(self.latest_joint_state.velocity) >= 3:
            obs['x.vel'] = self.latest_joint_state.velocity[0]
            obs['y.vel'] = self.latest_joint_state.velocity[1]
            obs['theta.vel'] = self.latest_joint_state.velocity[2]  # Already in rad/s from teleop
        else:
            obs['x.vel'] = 0.0
            obs['y.vel'] = 0.0
            obs['theta.vel'] = 0.0
        
        # Add camera images - use simple keys without 'observation.images.' prefix
        # build_dataset_frame() will add the prefix automatically
        # If rename_map is set, use renamed short keys (e.g., 'camera1' instead of 'front')
        front_key = 'front'
        wrist_key = 'wrist'
        if self.rename_map:
            front_key = self.rename_map.get('observation.images.front', 'observation.images.front').removeprefix('observation.images.')
            wrist_key = self.rename_map.get('observation.images.wrist', 'observation.images.wrist').removeprefix('observation.images.')
        
        if self.latest_front_image is not None:
            obs[front_key] = self.latest_front_image
        
        if self.latest_wrist_image is not None:
            obs[wrist_key] = self.latest_wrist_image
        
        return obs
    
    def _publish_commands(self, action: RobotAction):
        """Publish action commands to ROS topics."""
        timestamp = self.get_clock().now().to_msg()
        
        self.get_logger().debug(f'Publishing commands - action keys: {action.keys()}', throttle_duration_sec=1.0)
        
        # Extract arm joint commands from action dict (9次元: 5軸+グリッパー+基部3次元)
        # Action dict has keys matching recorder: 'arm_shoulder_pan.pos', etc.
        arm_joint_keys = ['arm_shoulder_pan.pos', 'arm_shoulder_lift.pos', 'arm_elbow_flex.pos',
                          'arm_wrist_flex.pos', 'arm_wrist_roll.pos', 'arm_gripper.pos']
        
        if all(key in action for key in arm_joint_keys):
            arm_cmd = JointState()
            arm_cmd.header.stamp = timestamp
            arm_cmd.header.frame_id = 'base_link'
            arm_cmd.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']
            
            # Extract arm positions from action dict
            arm_cmd.position = [
                float(action['arm_shoulder_pan.pos']),
                float(action['arm_shoulder_lift.pos']),
                float(action['arm_elbow_flex.pos']),
                float(action['arm_wrist_flex.pos']),
                float(action['arm_wrist_roll.pos']),
                float(action['arm_gripper.pos']),
            ]
            
            self.get_logger().info(f'Publishing arm command: {arm_cmd.position}', throttle_duration_sec=1.0)
            self.arm_joint_cmd_pub.publish(arm_cmd)
        else:
            self.get_logger().warn(f'Missing arm joint commands in action. Available keys: {list(action.keys())}')
        
        # Extract base velocity commands (x.vel, y.vel, theta.vel)
        base_keys = ['x.vel', 'y.vel', 'theta.vel']
        
        if all(key in action for key in base_keys):
            cmd_vel = Twist()
            cmd_vel.linear.x = float(action['x.vel'])
            cmd_vel.linear.y = float(action['y.vel'])
            cmd_vel.angular.z = float(action['theta.vel'])
            
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
