#!/usr/bin/env python3
"""
EKF vs UKFM Comparison Launch File

Launches both localization algorithms and comparison logger:
- EKF Localization
- UKFM Localization
- Comparison Logger

Usage:
  # Real robot (no ground truth)
  ros2 launch pkrc_controller ekf_ukfm_comparison.launch.py

  # Simulation (with ground truth)
  ros2 launch pkrc_controller ekf_ukfm_comparison.launch.py use_ground_truth:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_ground_truth = LaunchConfiguration('use_ground_truth')
    log_rate = LaunchConfiguration('log_rate')

    # Coordinate transform settings
    imu_inverted = LaunchConfiguration('imu_inverted')
    imu_rotation_axis = LaunchConfiguration('imu_rotation_axis')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_ground_truth',
            default_value='false',
            description='Use ground truth for comparison (simulation only)'
        ),
        DeclareLaunchArgument(
            'log_rate',
            default_value='10.0',
            description='Logging rate (Hz)'
        ),
        DeclareLaunchArgument(
            'imu_inverted',
            default_value='true',
            description='IMU mounted upside-down'
        ),
        DeclareLaunchArgument(
            'imu_rotation_axis',
            default_value='x',
            description='IMU rotation axis'
        ),

        # EKF Localization
        Node(
            package='pkrc_controller',
            executable='ekf_localization_real.py',
            name='ekf_localization_real',
            output='screen',
            parameters=[{
                'frequency': 50.0,
                'imu_inverted': imu_inverted,
                'imu_rotation_axis': imu_rotation_axis,
                'dvl_transform_enabled': False,  # Already transformed in DVL node
                'water_density': 1025.0,
                'gravity': 9.81,
                'process_noise_pos': 0.01,
                'process_noise_orient': 0.001,
                'process_noise_vel': 0.01,
                'measurement_noise_aruco_pos': 0.1,
                'measurement_noise_aruco_orient': 0.05,
                'measurement_noise_imu_orient': 0.01,
                'measurement_noise_dvl_vel': 0.05,
                'measurement_noise_depth': 0.01,
            }],
        ),

        # UKFM Localization (already running, but listed for reference)
        # Note: You should launch UKFM separately or include it here
        # Node(
        #     package='pkrc_controller',
        #     executable='ukfm_localization.py',
        #     name='ukfm_localization',
        #     output='screen',
        #     ...
        # ),

        # EKF vs UKFM Comparison Logger
        Node(
            package='pkrc_controller',
            executable='ekf_ukfm_comparison_logger.py',
            name='ekf_ukfm_comparison_logger',
            output='screen',
            parameters=[{
                'output_dir': os.path.expanduser('~/ukfm_logs'),
                'log_rate': log_rate,
                'use_ground_truth': use_ground_truth,
            }],
        ),
    ])
