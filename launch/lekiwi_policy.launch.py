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
Launch file for LeKiwi robot policy inference.

This launch file starts the policy inference node for autonomous robot control.
The node runs on a desktop PC with GPU and communicates with the robot via ROS2 topics.
The robot control is handled by lekiwi_teleop_node running on the Raspberry Pi.

Example usage:
    # On Raspberry Pi (robot):
    ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py

    # On Desktop PC (inference):
    ros2 launch lekiwi_ros2_teleop lekiwi_policy.launch.py \
        policy_path:=/path/to/trained/model \
        dataset_repo_id:=username/my_dataset \
        single_task:="Pick and place the cube"
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    
    # Policy arguments
    policy_path_arg = DeclareLaunchArgument(
        'policy_path',
        default_value='',
        description='Path to the trained policy model directory (required)'
    )
    
    dataset_repo_id_arg = DeclareLaunchArgument(
        'dataset_repo_id',
        default_value='',
        description='Dataset repository ID used for training (e.g., username/dataset_name) (required)'
    )
    
    dataset_root_arg = DeclareLaunchArgument(
        'dataset_root',
        default_value=str(Path.home() / 'lerobot_datasets'),
        description='Root directory where dataset is stored'
    )
    
    single_task_arg = DeclareLaunchArgument(
        'single_task',
        default_value='',
        description='Task description for the policy (optional, depends on policy type)'
    )
    
    # Control arguments
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='30.0',
        description='Control loop frequency in Hz'
    )
    
    # Device arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device for policy inference (cuda, cpu, or mps)'
    )
    
    use_amp_arg = DeclareLaunchArgument(
        'use_amp',
        default_value='false',
        description='Use automatic mixed precision for inference'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='',
        description='Robot port (not used in distributed mode, kept for compatibility)'
    )
    
    use_degrees_arg = DeclareLaunchArgument(
        'use_degrees',
        default_value='false',
        description='Use degrees instead of radians for joint angles'
    )
    
    rotate_front_camera_arg = DeclareLaunchArgument(
        'rotate_front_camera',
        default_value='true',
        description='Rotate front camera image by 180 degrees'
    )
    
    # Create policy node
    policy_node = Node(
        package='lekiwi_ros2_teleop',
        executable='lekiwi_policy_node',
        name='lekiwi_policy_node',
        output='screen',
        parameters=[{
            'robot_port': LaunchConfiguration('robot_port'),
            'policy_path': LaunchConfiguration('policy_path'),
            'dataset_repo_id': LaunchConfiguration('dataset_repo_id'),
            'dataset_root': LaunchConfiguration('dataset_root'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'single_task': LaunchConfiguration('single_task'),
            'use_degrees': LaunchConfiguration('use_degrees'),
            'rotate_front_camera': LaunchConfiguration('rotate_front_camera'),
            'device': LaunchConfiguration('device'),
            'use_amp': LaunchConfiguration('use_amp'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        policy_path_arg,
        dataset_repo_id_arg,
        dataset_root_arg,
        single_task_arg,
        control_frequency_arg,
        device_arg,
        use_amp_arg,
        robot_port_arg,
        use_degrees_arg,
        rotate_front_camera_arg,
        # Nodes
        policy_node,
    ])
