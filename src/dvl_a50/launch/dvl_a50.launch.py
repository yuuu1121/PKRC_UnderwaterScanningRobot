import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ip_address_arg = launch.actions.DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.220'
    )

    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_sensor',
        parameters=[{'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')}],
        output='screen')

    return launch.LaunchDescription([
        ip_address_arg,
        dvl_a50,
    ])


       
