from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_camera_publisher',
            executable='laser_camera_publisher',
            name='laser_camera_publisher',
            output='screen',
            parameters=[{
                'exposure': 10,
            }],
        ),
    ])
