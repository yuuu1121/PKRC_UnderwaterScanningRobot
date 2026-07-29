#!/usr/bin/env python3
"""PKRC 웹 GUI 서버 실행.

    ros2 launch pkrc_control gui.launch.py
    ros2 launch pkrc_control gui.launch.py port:=9000
    ros2 launch pkrc_control gui.launch.py host:=127.0.0.1   # 로컬만 허용
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='바인드 주소. 0.0.0.0 은 같은 망에서 접속 허용, '
                        '127.0.0.1 은 로컬(SSH 포트포워딩)만 허용',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP 포트',
        ),
        Node(
            package='pkrc_control',
            executable='gui_server',
            name='gui_server',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
