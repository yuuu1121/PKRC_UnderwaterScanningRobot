"""하방(downward) exploreHD 카메라 퍼블리셔.

레이저 카메라와 같은 노드(camera_node.py)를 device/topic 만 바꿔 띄운다.
장치는 by-path 로 지정 — 허브 포트 2.1.1 (2026-09-28 배선: 카메라 3대가
전부 포트 2.1 허브 밑, 2.1.1=하방 / 2.1.2=레이저 / 2.1.3=stellarHD).
(레이저 exploreHD 와 시리얼이 같아 by-id 로는 구분 불가)

    ros2 launch laser_camera_publisher downward_camera.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_camera_publisher',
            executable='laser_camera_publisher',
            name='downward_camera_publisher',
            output='screen',
            parameters=[{
                'device': ('/dev/v4l/by-path/'
                           'platform-3610000.usb-usb-0:2.1.1:1.0-video-index0'),
                'topic': '/downward/image_raw/compressed',
            }],
        ),
    ])
