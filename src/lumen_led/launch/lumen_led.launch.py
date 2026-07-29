from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lumen_led',
            executable='lumen_node',
            name='lumen_led',
            output='screen',
            parameters=[{
                'initial_brightness': 0.5,
            }],
        ),
    ])
