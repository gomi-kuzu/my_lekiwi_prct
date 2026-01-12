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
Launch file for LeKiwi robot data recording.

This launch file starts both the teleoperation client and data recorder nodes.

Example usage:
    ros2 launch lekiwi_ros2_teleop lekiwi_record.launch.py \
        dataset_repo_id:=username/my_dataset \
        single_task:="Pick and place the cube" \
        leader_arm_port:=/dev/ttyACM0
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    
    # Control arguments
    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='false',
        description='Launch teleoperation node (set to false if teleop node is already running)'
    )
    
    # Teleoperation arguments
    lekiwi_remote_ip_arg = DeclareLaunchArgument(
        'lekiwi_remote_ip',
        default_value='',
        description='IP address of LeKiwi robot (or set LEKIWI_REMOTE_IP env var)'
    )
    
    leader_arm_port_arg = DeclareLaunchArgument(
        'leader_arm_port',
        default_value='/dev/ttyACM0',
        description='Serial port for SO101 Leader Arm'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='Control loop frequency in Hz'
    )
    
    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard',
        default_value='true',
        description='Enable keyboard teleoperation for base control'
    )
    
    # Dataset recording arguments
    dataset_repo_id_arg = DeclareLaunchArgument(
        'dataset_repo_id',
        default_value='username/dataset_name',
        description='Dataset repository ID (e.g., username/dataset_name)'
    )
    
    dataset_root_arg = DeclareLaunchArgument(
        'dataset_root',
        default_value=str(Path.home() / 'lerobot_datasets'),
        description='Root directory for dataset storage'
    )
    
    single_task_arg = DeclareLaunchArgument(
        'single_task',
        default_value='Pick and place task',
        description='Description of the task being recorded'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Recording frames per second'
    )
    
    use_videos_arg = DeclareLaunchArgument(
        'use_videos',
        default_value='true',
        description='Encode images as videos in dataset'
    )
    
    num_image_writer_processes_arg = DeclareLaunchArgument(
        'num_image_writer_processes',
        default_value='0',
        description='Number of image writer processes (0 for threads only)'
    )
    
    num_image_writer_threads_arg = DeclareLaunchArgument(
        'num_image_writer_threads',
        default_value='4',
        description='Number of image writer threads per camera'
    )
    
    video_encoding_batch_size_arg = DeclareLaunchArgument(
        'video_encoding_batch_size',
        default_value='1',
        description='Number of episodes before batch encoding videos'
    )
    
    # Teleoperation client node (conditional)
    teleop_node = Node(
        package='lekiwi_ros2_teleop',
        executable='lekiwi_ros2_teleop_client',
        name='lekiwi_teleop_client',
        output='screen',
        parameters=[{
            'lekiwi_remote_ip': LaunchConfiguration('lekiwi_remote_ip'),
            'leader_arm_port': LaunchConfiguration('leader_arm_port'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'use_keyboard': LaunchConfiguration('use_keyboard'),
        }],
        condition=IfCondition(LaunchConfiguration('launch_teleop'))
    )
    
    # Data recorder node
    recorder_node = Node(
        package='lekiwi_ros2_teleop',
        executable='lekiwi_data_recorder',
        name='lekiwi_data_recorder',
        output='screen',
        parameters=[{
            'dataset_repo_id': LaunchConfiguration('dataset_repo_id'),
            'dataset_root': LaunchConfiguration('dataset_root'),
            'single_task': LaunchConfiguration('single_task'),
            'fps': LaunchConfiguration('fps'),
            'use_videos': LaunchConfiguration('use_videos'),
            'num_image_writer_processes': LaunchConfiguration('num_image_writer_processes'),
            'num_image_writer_threads': LaunchConfiguration('num_image_writer_threads'),
            'video_encoding_batch_size': LaunchConfiguration('video_encoding_batch_size'),
        }],
    )
    
    return LaunchDescription([
        # Launch arguments
        launch_teleop_arg,
        lekiwi_remote_ip_arg,
        leader_arm_port_arg,
        control_frequency_arg,
        use_keyboard_arg,
        dataset_repo_id_arg,
        dataset_root_arg,
        single_task_arg,
        fps_arg,
        use_videos_arg,
        num_image_writer_processes_arg,
        num_image_writer_threads_arg,
        video_encoding_batch_size_arg,
        
        # Nodes
        teleop_node,
        recorder_node,
    ])
