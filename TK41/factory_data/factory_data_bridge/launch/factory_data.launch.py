from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://127.0.0.1:8000/data'),
        DeclareLaunchArgument('timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('max_response_bytes', default_value='52428800'),
        Node(package='factory_data_bridge', executable='factory_data_service', output='screen',
             parameters=[{
                 'server_url': ParameterValue(LaunchConfiguration('server_url'), value_type=str),
                 'timeout_sec': ParameterValue(LaunchConfiguration('timeout_sec'), value_type=float),
                 'max_response_bytes': ParameterValue(LaunchConfiguration('max_response_bytes'), value_type=int),
             }]),
    ])
