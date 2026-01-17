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
ROS2 Data Recorder Node for LeKiwi Robot

This node subscribes to robot observation and action topics,
and records the data in LeRobot Dataset v3 format for later training.
"""

import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, CompressedImage
from std_srvs.srv import Trigger

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

# Monkey-patch get_safe_version to avoid HuggingFace Hub API calls for local datasets
# This must be done BEFORE importing LeRobotDataset
from lerobot.datasets import utils as lerobot_utils
_original_get_safe_version = lerobot_utils.get_safe_version

def _get_safe_version_offline(repo_id: str, version: str):
    """Return the version without checking HuggingFace Hub.
    
    This allows loading local datasets without network access.
    """
    return version if isinstance(version, str) else str(version)

# Apply the patch to utils module
lerobot_utils.get_safe_version = _get_safe_version_offline

# Also patch snapshot_download to avoid HuggingFace Hub access
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

# Now import LeRobotDataset (after patching)
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Also patch the lerobot_dataset module's global namespace
import lerobot.datasets.lerobot_dataset as lerobot_dataset_module
lerobot_dataset_module.get_safe_version = _get_safe_version_offline

# Import other LeRobot components
from lerobot.datasets.pipeline_features import create_initial_features
from lerobot.datasets.utils import build_dataset_frame
from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.processor import RobotAction, RobotObservation


class LeKiwiDataRecorder(Node):
    """
    ROS2 Node for recording LeKiwi robot data in LeRobot Dataset v3 format.
    
    Subscribes to:
        - /lekiwi/joint_states (sensor_msgs/JointState): Current robot state
        - /lekiwi/cmd_vel (geometry_msgs/Twist): Base velocity commands (actions)
        - /lekiwi/arm_joint_commands (sensor_msgs/JointState): Arm joint commands (actions)
        - /lekiwi/camera/front/image_raw/compressed (sensor_msgs/CompressedImage): Front camera
        - /lekiwi/camera/wrist/image_raw/compressed (sensor_msgs/CompressedImage): Wrist camera
    
    Services:
        - ~/start_episode (std_srvs/Trigger): Start recording a new episode
        - ~/stop_episode (std_srvs/Trigger): Stop and save current episode
        - ~/save_dataset (std_srvs/Trigger): Finalize dataset
    """
    
    def __init__(self):
        super().__init__('lekiwi_data_recorder')
        
        # Declare parameters
        self.declare_parameter('dataset_repo_id', 'your_username/your_dataset_name')
        self.declare_parameter('dataset_root', str(Path.home() / 'lerobot_datasets'))
        self.declare_parameter('single_task', 'Pick and place task')
        self.declare_parameter('fps', 30)
        self.declare_parameter('robot_type', 'lekiwi_client')
        self.declare_parameter('resume', False)  # Resume recording on existing dataset
        self.declare_parameter('num_image_writer_processes', 0)
        self.declare_parameter('num_image_writer_threads', 4)
        self.declare_parameter('use_videos', True)
        self.declare_parameter('video_encoding_batch_size', 1)
        
        # Get parameters
        self.dataset_repo_id = self.get_parameter('dataset_repo_id').value
        
        # Validate dataset_repo_id format (must be in 'username/dataset_name' format)
        if '/' not in self.dataset_repo_id:
            raise ValueError(
                f"Invalid dataset_repo_id: '{self.dataset_repo_id}'. "
                f"Must be in format 'username/dataset_name' (e.g., 'john/pick_place_data'). "
                f"This format is required for proper dataset organization and compatibility with LeRobot tools."
            )
        
        username, dataset_name = self.dataset_repo_id.split('/', 1)
        if not username or not dataset_name:
            raise ValueError(
                f"Invalid dataset_repo_id: '{self.dataset_repo_id}'. "
                f"Both username and dataset_name must be non-empty."
            )
        
        if username in ['your_username', 'username'] or dataset_name in ['your_dataset_name', 'dataset_name']:
            self.get_logger().warn(
                f"Using placeholder dataset_repo_id: '{self.dataset_repo_id}'. "
                f"Please update to a meaningful name (e.g., 'john/lekiwi_pick_place')."
            )
        
        dataset_root_param = self.get_parameter('dataset_root').value
        # Expand ~ and convert to absolute path
        self.dataset_root = str(Path(dataset_root_param).expanduser().resolve())
        self.single_task = self.get_parameter('single_task').value
        self.fps = self.get_parameter('fps').value
        self.robot_type = self.get_parameter('robot_type').value
        self.resume = self.get_parameter('resume').value
        self.num_image_writer_processes = self.get_parameter('num_image_writer_processes').value
        self.num_image_writer_threads = self.get_parameter('num_image_writer_threads').value
        self.use_videos = self.get_parameter('use_videos').value
        self.video_encoding_batch_size = self.get_parameter('video_encoding_batch_size').value
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/lekiwi/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/lekiwi/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.arm_cmd_sub = self.create_subscription(
            JointState,
            '/lekiwi/arm_joint_commands',
            self.arm_cmd_callback,
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
        
        # Create services
        self.start_episode_srv = self.create_service(
            Trigger,
            '~/start_episode',
            self.start_episode_callback
        )
        
        self.stop_episode_srv = self.create_service(
            Trigger,
            '~/stop_episode',
            self.stop_episode_callback
        )
        
        self.save_dataset_srv = self.create_service(
            Trigger,
            '~/save_dataset',
            self.save_dataset_callback
        )
        
        # Internal state
        self.latest_joint_state: Optional[JointState] = None
        self.latest_cmd_vel: Optional[Twist] = None
        self.latest_arm_cmd: Optional[JointState] = None
        self.latest_front_image: Optional[np.ndarray] = None
        self.latest_wrist_image: Optional[np.ndarray] = None
        
        self.recording = False
        self.dataset: Optional[LeRobotDataset] = None
        self.episode_start_time: Optional[float] = None
        self.frame_count = 0
        
        # Initialize dataset
        self._initialize_dataset()
        
        # Create timer for periodic data recording
        self.record_timer = self.create_timer(1.0 / self.fps, self.record_frame_callback)
        
        # Display initialization summary
        dataset_path = Path(self.dataset_root) / self.dataset_repo_id
        mode = "RESUME" if self.resume else "NEW"
        self.get_logger().info(f'LeKiwi Data Recorder initialized [{mode} mode]')
        self.get_logger().info(f'Dataset repo_id: {self.dataset_repo_id}')
        self.get_logger().info(f'Dataset location: {dataset_path}')
        self.get_logger().info(f'Task: {self.single_task}')
        self.get_logger().info(f'FPS: {self.fps}')
        self.get_logger().info(f'Total episodes: {self.dataset.num_episodes}')
        self.get_logger().info(f'Use videos: {self.use_videos}')
        self.get_logger().info('Services:')
        self.get_logger().info('  - ~/start_episode: Start recording new episode')
        self.get_logger().info('  - ~/stop_episode: Stop and save current episode')
        self.get_logger().info('  - ~/save_dataset: Display dataset info')
        self.get_logger().info('Ready to record. Call ~/start_episode to begin.')
    
    def _initialize_dataset(self):
        """Initialize LeRobot dataset with appropriate features."""
        # Define features for Dataset v3 format
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (9,),  # 5 arm joints + 1 gripper + 2 base linear + 1 base angular
                "names": [
                    "arm_shoulder_pan.pos",
                    "arm_shoulder_lift.pos",
                    "arm_elbow_flex.pos",
                    "arm_wrist_flex.pos",
                    "arm_wrist_roll.pos",
                    "arm_gripper.pos",
                    "x.vel",
                    "y.vel",
                    "theta.vel",
                ],
            },
            "observation.images.front": {
                "dtype": "video" if self.use_videos else "image",
                "shape": (480, 640, 3),  # height, width, channels (RGB)
                "names": ["height", "width", "channels"],
            },
            "observation.images.wrist": {
                "dtype": "video" if self.use_videos else "image",
                "shape": (640, 480, 3),  # height, width, channels (RGB) 
                "names": ["height", "width", "channels"],
            },
            "action": {
                "dtype": "float32",
                "shape": (9,),  # Same as state
                "names": [
                    "arm_shoulder_pan.pos",
                    "arm_shoulder_lift.pos",
                    "arm_elbow_flex.pos",
                    "arm_wrist_flex.pos",
                    "arm_wrist_roll.pos",
                    "arm_gripper.pos",
                    "x.vel",
                    "y.vel",
                    "theta.vel",
                ],
            },
        }
        
        # Create or load dataset
        dataset_path = Path(self.dataset_root) / self.dataset_repo_id
        dataset_exists = dataset_path.exists() and (dataset_path / 'meta' / 'info.json').exists()
        
        if self.resume:
            # Resume mode: append to existing dataset
            if not dataset_exists:
                raise ValueError(
                    f'Resume mode enabled but dataset not found at {dataset_path}. '
                    f'To create a new dataset, set resume:=false'
                )
            
            self.get_logger().info(f'[RESUME MODE] Appending to existing dataset: {self.dataset_repo_id}')
            self.get_logger().info(f'Dataset path: {dataset_path}')
            
            try:
                # Load existing dataset from local files (with patched get_safe_version)
                # Note: LeRobotDataset expects root to be the dataset directory itself, not the parent
                dataset_full_path = Path(self.dataset_root) / self.dataset_repo_id
                self.dataset = LeRobotDataset(
                    repo_id=self.dataset_repo_id,
                    root=str(dataset_full_path),  # Full path to dataset directory
                    revision='v3.0',  # Use fixed version
                    batch_encoding_size=self.video_encoding_batch_size,
                )
                
                # Start image writer for recording
                self.dataset.start_image_writer(
                    num_processes=self.num_image_writer_processes,
                    num_threads=self.num_image_writer_threads,
                )
                
                self.get_logger().info(f'✓ Loaded existing dataset with {self.dataset.num_episodes} episodes')
                self.get_logger().info(f'  Total frames: {self.dataset.num_frames}')
                self.get_logger().info(f'  FPS: {self.dataset.fps}')
                self.get_logger().info(f'  Robot type: {self.dataset.meta.robot_type}')
                
                # Verify compatibility
                if self.dataset.fps != self.fps:
                    self.get_logger().warn(
                        f'Dataset FPS ({self.dataset.fps}) differs from requested FPS ({self.fps}). '
                        f'Using dataset FPS to maintain consistency.'
                    )
                    self.fps = self.dataset.fps
                    
            except Exception as load_error:
                self.get_logger().error(f'Failed to load existing dataset: {load_error}')
                import traceback
                self.get_logger().error(f'\nTraceback:\n{traceback.format_exc()}')
                raise
        else:
            # New dataset mode: create new (error if already exists)
            if dataset_exists:
                raise ValueError(
                    f'Dataset already exists at {dataset_path}. '
                    f'To add episodes to existing dataset, use resume:=true. '
                    f'To create a new dataset, delete the existing one first: rm -rf {dataset_path}'
                )
            # Create new dataset
            self.get_logger().info(f'Creating new dataset: {self.dataset_repo_id}')
            self.get_logger().info(f'Dataset will be created at: {dataset_path}')
            self.get_logger().info(f'Robot type: {self.robot_type}')
            self.get_logger().info(f'FPS: {self.fps}')
            self.get_logger().info(f'Use videos: {self.use_videos}')
            
            try:
                self.dataset = LeRobotDataset.create(
                    repo_id=self.dataset_repo_id,
                    fps=self.fps,
                    root=self.dataset_root,
                    robot_type=self.robot_type,
                    features=features,
                    use_videos=self.use_videos,
                    image_writer_processes=self.num_image_writer_processes,
                    image_writer_threads=self.num_image_writer_threads,
                    batch_encoding_size=self.video_encoding_batch_size,
                )
                self.get_logger().info(f'✓ Successfully created new dataset')
                self.get_logger().info(f'  Location: {dataset_path}')
                self.get_logger().info(f'  Repo ID: {self.dataset_repo_id}')
            except Exception as e:
                self.get_logger().error(f'Failed to create dataset: {e}')
                self.get_logger().error(f'Dataset root: {self.dataset_root}')
                self.get_logger().error(f'Dataset repo_id: {self.dataset_repo_id}')
                self.get_logger().error(f'Dataset path: {dataset_path}')
                self.get_logger().error(f'\nCommon issues:')
                self.get_logger().error(f'  1. Ensure dataset_repo_id is in format "username/dataset_name"')
                self.get_logger().error(f'  2. Check that image resolution matches your camera feeds')
                self.get_logger().error(f'  3. Verify LEROBOT_PATH environment variable is set correctly')
                self.get_logger().error(f'  4. If updating features, delete existing dataset directory first')
                import traceback
                self.get_logger().error(f'\nFull traceback:\n{traceback.format_exc()}')
                raise
    
    def joint_state_callback(self, msg: JointState):
        """Callback for robot joint state."""
        self.latest_joint_state = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """Callback for base velocity commands."""
        self.latest_cmd_vel = msg
    
    def arm_cmd_callback(self, msg: JointState):
        """Callback for arm joint commands."""
        self.latest_arm_cmd = msg
    
    def front_camera_callback(self, msg: CompressedImage):
        """Callback for front camera images."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                # Convert BGR to RGB
                self.latest_front_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode front camera image: {e}')
    
    def wrist_camera_callback(self, msg: CompressedImage):
        """Callback for wrist camera images."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                # Convert BGR to RGB
                self.latest_wrist_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().warn(f'Failed to decode wrist camera image: {e}')
    
    def start_episode_callback(self, request, response):
        """Service callback to start recording a new episode."""
        if self.recording:
            response.success = False
            response.message = 'Already recording an episode'
            return response
        
        self.recording = True
        self.episode_start_time = time.time()
        self.frame_count = 0
        
        response.success = True
        response.message = f'Started recording episode {self.dataset.num_episodes}'
        self.get_logger().info(response.message)
        
        return response
    
    def stop_episode_callback(self, request, response):
        """Service callback to stop and save current episode."""
        if not self.recording:
            response.success = False
            response.message = 'Not currently recording'
            return response
        
        self.recording = False
        
        try:
            self.dataset.save_episode()
            episode_duration = time.time() - self.episode_start_time
            response.success = True
            response.message = (
                f'Saved episode {self.dataset.num_episodes - 1} '
                f'({self.frame_count} frames, {episode_duration:.2f}s)'
            )
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save episode: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def save_dataset_callback(self, request, response):
        """Service callback to finalize and save dataset."""
        if self.recording:
            response.success = False
            response.message = 'Cannot save dataset while recording. Stop episode first.'
            return response
        
        try:
            # Dataset is already saved per episode, just log info
            response.success = True
            response.message = (
                f'Dataset saved with {self.dataset.num_episodes} episodes. '
                f'Use upload script to push to Hugging Face Hub if desired.'
            )
            self.get_logger().info(response.message)
            self.get_logger().info(f'Dataset location: {self.dataset.root}')
        except Exception as e:
            response.success = False
            response.message = f'Error finalizing dataset: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def record_frame_callback(self):
        """Timer callback to record a frame of data."""
        if not self.recording:
            return
        
        # Check if we have all required data
        if (self.latest_joint_state is None or 
            self.latest_arm_cmd is None or
            self.latest_front_image is None or
            self.latest_wrist_image is None):
            self.get_logger().warn(
                'Missing data for frame recording '
                f'(joint_state: {self.latest_joint_state is not None}, '
                f'arm_cmd: {self.latest_arm_cmd is not None}, '
                f'front_img: {self.latest_front_image is not None}, '
                f'wrist_img: {self.latest_wrist_image is not None})',
                throttle_duration_sec=1.0
            )
            return
        
        try:
            # Build observation and action
            observation = self._build_observation()
            action = self._build_action()
            
            # Create frame directly without build_dataset_frame
            frame = {
                "observation.state": observation["state"],
                "observation.images.front": observation["images"]["front"],
                "observation.images.wrist": observation["images"]["wrist"],
                "action": action,
                "task": self.single_task
            }
            
            # Add frame to dataset
            self.dataset.add_frame(frame)
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:  # Log every second
                self.get_logger().info(
                    f'Recording episode {self.dataset.num_episodes}: '
                    f'{self.frame_count} frames'
                )
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'Failed to record frame: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}', throttle_duration_sec=5.0)
    
    def _build_observation(self) -> Dict[str, Any]:
        """Build observation dict from latest sensor data."""
        # Extract arm joint positions (5 joints + 1 gripper)
        # If gripper data is not available, pad with 0
        joint_positions = list(self.latest_joint_state.position)
        if len(joint_positions) >= 6:
            arm_positions = joint_positions[:6]
        elif len(joint_positions) == 5:
            arm_positions = joint_positions[:5] + [0.0]  # Add gripper position as 0
        else:
            # Pad to 6 elements if less than 5
            arm_positions = joint_positions + [0.0] * (6 - len(joint_positions))
        
        # Extract base velocities (x, y, theta)
        # Use joint_state.velocity which contains [x.vel, y.vel, theta.vel] from teleop node
        if self.latest_joint_state is not None and len(self.latest_joint_state.velocity) >= 3:
            base_velocities = [
                self.latest_joint_state.velocity[0],  # x.vel
                self.latest_joint_state.velocity[1],  # y.vel
                self.latest_joint_state.velocity[2],  # theta.vel
            ]
        else:
            base_velocities = [0.0, 0.0, 0.0]
        
        # Combine into state vector
        state = np.array(arm_positions + base_velocities, dtype=np.float32)
        
        observation = {
            "state": state,
            "images": {
                "front": self.latest_front_image,
                "wrist": self.latest_wrist_image,
            }
        }
        
        return observation
    
    def _build_action(self) -> Dict[str, Any]:
        """Build action dict from latest command data."""
        # Extract arm joint commands (5 joints + 1 gripper)
        # If gripper data is not available, pad with 0
        joint_commands = list(self.latest_arm_cmd.position)
        if len(joint_commands) >= 6:
            arm_commands = joint_commands[:6]
        elif len(joint_commands) == 5:
            arm_commands = joint_commands[:5] + [0.0]  # Add gripper command as 0
        else:
            # Pad to 6 elements if less than 5
            arm_commands = joint_commands + [0.0] * (6 - len(joint_commands))
        
        # Extract base velocity commands (x, y, theta)
        if self.latest_cmd_vel is not None:
            base_vel_commands = [
                self.latest_cmd_vel.linear.x,   # x.vel_cmd
                self.latest_cmd_vel.linear.y,   # y.vel_cmd
                self.latest_cmd_vel.angular.z,  # theta.vel_cmd
            ]
        else:
            base_vel_commands = [0.0, 0.0, 0.0]
        
        # Combine into action vector
        action_array = np.array(arm_commands + base_vel_commands, dtype=np.float32)
        
        action = action_array
        
        return action
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        if self.recording:
            self.get_logger().warn('Node destroyed while recording. Saving episode...')
            try:
                self.dataset.save_episode()
            except Exception as e:
                self.get_logger().error(f'Failed to save episode on shutdown: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LeKiwiDataRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in data recorder node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
