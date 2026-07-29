#!/usr/bin/env python3
"""
PKRC Full System Launch File
모든 센서 + 카메라 2대를 한번에 실행

Usage:
    ros2 launch pkrc_controller full_system.launch.py
    ros2 launch pkrc_controller full_system.launch.py dvl_address:=192.168.1.99
    ros2 launch pkrc_controller full_system.launch.py cameras:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, condition=None, launch_arguments=None):
    """패키지 share/launch/<file> 을 포함한다."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
        ),
        condition=condition,
        launch_arguments=launch_arguments,
    )


def generate_launch_description():
    dvl_address = LaunchConfiguration('dvl_address')
    cameras = LaunchConfiguration('cameras')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dvl_address',
            default_value='192.168.0.220',
            description='DVL-A50 IP address',
        ),
        DeclareLaunchArgument(
            'cameras',
            default_value='true',
            description='stellarHD / exploreHD 카메라 2대 실행 여부',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Ping1D RViz 시각화 실행 여부',
        ),

        # 압력 센서 (Bar10XT)
        _include('bar10xt_ros2', 'bar10xt.launch.py'),

        # DVL-A50
        _include('dvl_a50', 'dvl_a50.launch.py',
                 launch_arguments=[('ip_address', dvl_address)]),

        # Microstrain IMU
        _include('microstrain_inertial_driver', 'microstrain_launch.py'),

        # Lumen LED
        _include('lumen_led', 'lumen_led.launch.py'),

        # Ping1D 소나 (전체 실행 시 RViz 는 기본 off — use_rviz:=true 로 켤 수 있음)
        _include('ping1d_sonar', 'ping_sonar.launch.py',
                 launch_arguments=[('use_rviz', LaunchConfiguration('use_rviz'))]),

        # ArUco 6DOF 마커 검출 (active_marker 는 launch 파일이 없어 노드로 직접 실행)
        Node(
            package='active_marker',
            executable='aruco_detector_6dof',
            name='aruco_detector_6dof',
            output='screen',
        ),

        # Camera 1: stellarHD - /dev/video4 (1600x1200 @ 60fps) - usb_cam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='stellarHD',
            namespace='stellarHD',
            output='screen',
            condition=IfCondition(cameras),
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 1600,
                'image_height': 1200,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 60.0,
                'camera_name': 'stellarHD',
                'camera_info_url': 'file:///home/hero/.ros/camera_info/default_cam.yaml',
            }],
        ),

        # Camera 2: exploreHD - /dev/video0 (1920x1080 @ 30fps) - gscam
        # image_encoding=jpeg → JPEG 디코딩 없이 CompressedImage로 직접 퍼블리시
        Node(
            package='gscam',
            executable='gscam_node',
            name='exploreHD',
            namespace='exploreHD',
            output='screen',
            condition=IfCondition(cameras),
            parameters=[{
                'camera_info_url': 'file:///home/hero/.ros/camera_info/explorehd_usb_camera:_explorehd.yaml',
                'gscam_config': 'v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1',
                'image_encoding': 'jpeg',
                'sync_sink': True,
                'frame_id': 'exploreHD_frame',
            }],
        ),
    ])
