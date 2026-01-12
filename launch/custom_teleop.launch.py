#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    """
    カスタムJoy設定でteleop_twist_joyを起動するlaunchファイル
    
    使用方法:
        ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py
        
    または、カスタム設定ファイルを指定:
        ros2 launch lekiwi_ros2_teleop custom_teleop.launch.py \\
            config_filepath:=/path/to/your/config.yaml
    """
    
    # パッケージのディレクトリを取得
    pkg_dir = get_package_share_directory('lekiwi_ros2_teleop')
    
    # デフォルトの設定ファイルパス
    default_config = os.path.join(pkg_dir, 'config', 'custom_joy.config.yaml')
    
    # Launch引数の定義
    joy_vel = launch.substitutions.LaunchConfiguration('joy_vel')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        # cmd_velトピック名
        launch.actions.DeclareLaunchArgument(
            'joy_vel', 
            default_value='/lekiwi/cmd_vel',
            description='Velocity command topic name'
        ),
        
        # ジョイスティックデバイスID
        launch.actions.DeclareLaunchArgument(
            'joy_dev', 
            default_value='0',
            description='Joystick device ID'
        ),
        
        # スタンプ付きTwistメッセージを発行するか
        launch.actions.DeclareLaunchArgument(
            'publish_stamped_twist', 
            default_value='false',
            description='Publish TwistStamped instead of Twist'
        ),
        
        # 設定ファイルパス
        launch.actions.DeclareLaunchArgument(
            'config_filepath', 
            default_value=default_config,
            description='Path to teleop config file'
        ),

        # Joyノード起動
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),
        
        # Teleop Twist Joyノード起動
        launch_ros.actions.Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                config_filepath, 
                {'publish_stamped_twist': publish_stamped_twist}
            ],
            remappings=[('/cmd_vel', joy_vel)],
            output='screen'
        ),
    ])
