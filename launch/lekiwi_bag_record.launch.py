#!/usr/bin/env python3
# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""Launch file for LeKiwi MCAP-based data recording."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop', default_value='false',
        description='Launch teleoperation client alongside the bag recorder.'
    )

    lekiwi_remote_ip_arg = DeclareLaunchArgument(
        'lekiwi_remote_ip', default_value='',
        description='IP of the LeKiwi robot (or set LEKIWI_REMOTE_IP env).')

    leader_arm_port_arg = DeclareLaunchArgument(
        'leader_arm_port', default_value='/dev/ttyACM0',
        description='Serial port for SO101 Leader Arm.')

    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency', default_value='30.0',
        description='Control loop frequency in Hz.')

    use_keyboard_arg = DeclareLaunchArgument(
        'use_keyboard', default_value='true',
        description='Enable keyboard teleop for base control.')

    session_name_arg = DeclareLaunchArgument(
        'session_name', default_value='',
        description='Session directory name (default: session_YYYYmmdd_HHMMSS).')

    output_root_arg = DeclareLaunchArgument(
        'output_root', default_value=str(Path.home() / 'lekiwi_bags'),
        description='Root directory for recorded MCAP bags.')

    single_task_arg = DeclareLaunchArgument(
        'single_task', default_value='Pick and place task',
        description='Task description (stored in session/episode meta).')

    target_fps_arg = DeclareLaunchArgument(
        'target_fps', default_value='30',
        description='Target FPS for downstream dataset (stored in meta).')

    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='lekiwi_client',
        description='Robot type string stored in meta.')

    storage_id_arg = DeclareLaunchArgument(
        'storage_id', default_value='mcap',
        description='rosbag2 storage plugin id.')

    operator_arg = DeclareLaunchArgument(
        'operator', default_value='',
        description='Operator name (recorded in session.yaml).')
    location_arg = DeclareLaunchArgument(
        'location', default_value='',
        description='Recording location (recorded in session.yaml).')
    note_arg = DeclareLaunchArgument(
        'note', default_value='',
        description='Freeform note (recorded in session.yaml).')
    arm_calibration_file_arg = DeclareLaunchArgument(
        'arm_calibration_file', default_value='',
        description='Path to arm calibration file (stored as provenance).')
    front_camera_id_arg = DeclareLaunchArgument(
        'front_camera_id', default_value='',
        description='Front camera identifier/serial (provenance).')
    wrist_camera_id_arg = DeclareLaunchArgument(
        'wrist_camera_id', default_value='',
        description='Wrist camera identifier/serial (provenance).')

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
        condition=IfCondition(LaunchConfiguration('launch_teleop')),
    )

    bag_recorder_node = Node(
        package='lekiwi_ros2_teleop',
        executable='lekiwi_bag_recorder',
        name='lekiwi_bag_recorder',
        output='screen',
        parameters=[{
            'session_name': LaunchConfiguration('session_name'),
            'output_root': LaunchConfiguration('output_root'),
            'single_task': LaunchConfiguration('single_task'),
            'robot_type': LaunchConfiguration('robot_type'),
            'target_fps': LaunchConfiguration('target_fps'),
            'storage_id': LaunchConfiguration('storage_id'),
            'operator': LaunchConfiguration('operator'),
            'location': LaunchConfiguration('location'),
            'note': LaunchConfiguration('note'),
            'lekiwi_remote_ip': LaunchConfiguration('lekiwi_remote_ip'),
            'leader_arm_port': LaunchConfiguration('leader_arm_port'),
            'arm_calibration_file':
                LaunchConfiguration('arm_calibration_file'),
            'front_camera_id': LaunchConfiguration('front_camera_id'),
            'wrist_camera_id': LaunchConfiguration('wrist_camera_id'),
        }],
    )

    return LaunchDescription([
        launch_teleop_arg,
        lekiwi_remote_ip_arg,
        leader_arm_port_arg,
        control_frequency_arg,
        use_keyboard_arg,
        session_name_arg,
        output_root_arg,
        single_task_arg,
        target_fps_arg,
        robot_type_arg,
        storage_id_arg,
        operator_arg,
        location_arg,
        note_arg,
        arm_calibration_file_arg,
        front_camera_id_arg,
        wrist_camera_id_arg,
        teleop_node,
        bag_recorder_node,
    ])
