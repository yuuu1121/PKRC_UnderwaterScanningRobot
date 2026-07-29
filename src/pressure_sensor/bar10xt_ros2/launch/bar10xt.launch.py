from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('bus', default_value='1'),
        DeclareLaunchArgument('rate', default_value='5.0'),
        DeclareLaunchArgument('frame_id', default_value='pressure_sensor'),
        DeclareLaunchArgument('water_density', default_value='997.0'),
        DeclareLaunchArgument('gravity', default_value='9.80665'),
        DeclareLaunchArgument('atmospheric_pressure', default_value='101325.0'),
        DeclareLaunchArgument('auto_zero', default_value='false'),
        DeclareLaunchArgument('pressure_offset', default_value='0.0'),
        DeclareLaunchArgument('temperature_offset', default_value='0.0'),
    ]

    def p(name, t):
        return ParameterValue(LaunchConfiguration(name), value_type=t)

    node = Node(
        package='bar10xt_ros2',
        executable='bar10xt_node',
        name='bar10xt',
        output='screen',
        parameters=[{
            'bus': p('bus', int),
            'rate': p('rate', float),
            'frame_id': p('frame_id', str),
            'water_density': p('water_density', float),
            'gravity': p('gravity', float),
            'atmospheric_pressure': p('atmospheric_pressure', float),
            'auto_zero': p('auto_zero', bool),
            'pressure_offset': p('pressure_offset', float),
            'temperature_offset': p('temperature_offset', float),
        }],
    )

    return LaunchDescription(args + [node])
