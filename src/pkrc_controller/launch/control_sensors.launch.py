#!/usr/bin/env python3
"""
PKRC Control Sensors Launch File
제어에 필요한 최소 센서 3종만 실행 (DVL / 압력 / IMU)

full_system.launch.py 와 달리 카메라·소나·ArUco 는 띄우지 않는다.

Usage:
    ros2 launch pkrc_controller control_sensors.launch.py
    ros2 launch pkrc_controller control_sensors.launch.py dvl_address:=192.168.1.99
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, launch_arguments=None):
    """패키지 share/launch/<file> 을 포함한다."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ),
        launch_arguments=launch_arguments,
    )


def generate_launch_description():
    dvl_address = LaunchConfiguration('dvl_address')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dvl_address',
            default_value='192.168.0.220',
            description='DVL-A50 IP address',
        ),

        # 압력 센서 (Bar10XT) — 수심
        _include('bar10xt_ros2', 'bar10xt.launch.py'),

        # DVL-A50 — 속도 / 고도
        _include('dvl_a50', 'dvl_a50.launch.py',
                 launch_arguments=[('ip_address', dvl_address)]),

        # Microstrain IMU — 자세
        _include('microstrain_inertial_driver', 'microstrain_launch.py'),
    ])
