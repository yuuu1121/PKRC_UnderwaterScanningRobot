#!/usr/bin/env python3
"""
Localization Launch File
모든 센서를 한번에 실행

Usage:
    ros2 launch pkrc_controller localization.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    return LaunchDescription([
        # # Microstrain IMU (GV7-INS)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('microstrain_inertial_driver'),
        #         '/launch/microstrain_launch.py'
        #     ]),
        # ),

        # # DVL-A50
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         FindPackageShare('dvl_a50'),
        #         '/launch/dvl_a50.launch.py'
        #     ]),
        # ),

        # ArUco Detector 6DOF (2Hz blinking marker optimized)
        Node(
            package='active_marker',
            executable='aruco_detector_6dof',
            name='aruco_detector_6dof',
            output='screen',
            parameters=[{
                # by-path: /dev/videoN 번호는 USB 열거 순서에 밀린다.
                # 노드 기본값과 같지만 배선 변경 시 눈에 띄도록 명시한다.
                'camera_device': ('/dev/v4l/by-path/platform-3610000.usb'
                                  '-usb-0:2.1.3:1.0-video-index0'),
                'marker_ids': [0, 1, 2, 3],
                'marker_map_ids': [0, 1, 2, 3],
                # === 2Hz blink tolerance settings ===
                # 2Hz = 500ms period (250ms ON, 250ms OFF)
                # At 30fps, OFF period = ~8 frames
                # Hold for 15 frames (~0.5s) to bridge OFF periods
                'detection_hold_frames': 15,
                # Camera settings for bright LED marker (reduce overexposure)
                'exposure_time': 1,      # Minimum exposure to prevent LED bloom
                'brightness': -64,       # Minimum brightness
                'contrast': 64,          # Maximum contrast for edge detection
                'gain': 0,               # No gain amplification
            }],
        ),

        # # Pressure Sensor (MS5837)
        # Node(
        #     package='pressure_sensor',
        #     executable='pressure_sensor_node',
        #     name='pressure_sensor',
        #     output='screen',
        # ),

        # UKF-M Localization
        Node(
            package='pkrc_controller',
            executable='ukfm_localization',
            name='ukfm_localization',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'pressure_topic': '/bar10xt/pressure',
                'dvl_topic': '/dvl/data',
                'aruco_topic': '/aruco/pose_array',
                'use_dvl': True,
                'dvl_mount_yaw': 90.0,
            }],
        ),
    ])
