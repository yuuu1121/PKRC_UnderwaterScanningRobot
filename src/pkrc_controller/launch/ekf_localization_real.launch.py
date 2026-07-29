#!/usr/bin/env python3
"""
EKF Localization Launch File for Real PKRC Robot

Launches:
- EKF Localization (with IMU & DVL coordinate transforms)
- RViz2 Visualization (optional)

Usage:
  ros2 launch pkrc_controller ekf_localization_real.launch.py
  ros2 launch pkrc_controller ekf_localization_real.launch.py rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pkrc_controller')

    # Config file path (if exists)
    # config_file = os.path.join(pkg_dir, 'config', 'ekf_real.yaml')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    frequency = LaunchConfiguration('frequency')

    # Coordinate transform settings
    imu_inverted = LaunchConfiguration('imu_inverted')
    imu_rotation_axis = LaunchConfiguration('imu_rotation_axis')
    dvl_transform_enabled = LaunchConfiguration('dvl_transform_enabled')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='50.0',
            description='EKF update frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'imu_inverted',
            default_value='true',
            description='IMU mounted upside-down (180° rotation)'
        ),
        DeclareLaunchArgument(
            'imu_rotation_axis',
            default_value='x',
            description='IMU rotation axis: x, y, or z'
        ),
        DeclareLaunchArgument(
            'dvl_transform_enabled',
            default_value='false',
            description='Apply DVL coordinate transform (false if already transformed in DVL node)'
        ),

        # EKF Localization Node
        Node(
            package='pkrc_controller',
            executable='ekf_localization_real.py',
            name='ekf_localization_real',
            output='screen',
            parameters=[{
                'frequency': frequency,
                'imu_inverted': imu_inverted,
                'imu_rotation_axis': imu_rotation_axis,
                'dvl_transform_enabled': dvl_transform_enabled,

                # Sensor parameters
                'water_density': 1025.0,
                'gravity': 9.81,

                # Process noise
                'process_noise_pos': 0.01,
                'process_noise_orient': 0.001,
                'process_noise_vel': 0.01,

                # Measurement noise
                'measurement_noise_aruco_pos': 0.1,
                'measurement_noise_aruco_orient': 0.05,
                'measurement_noise_imu_orient': 0.01,
                'measurement_noise_dvl_vel': 0.05,
                'measurement_noise_depth': 0.01,
            }],
        ),

        # RViz2 Visualization (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ekf_real.rviz')]
                if os.path.exists(os.path.join(pkg_dir, 'rviz', 'ekf_real.rviz'))
                else []
        ),
    ])
