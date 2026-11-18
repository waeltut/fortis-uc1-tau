from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_mir_description', default_value='false',
            description='Enable only when no other robot_state_publisher owns the MiR TF tree'),
        Node(
            package='mir_driver',
            namespace='mir_driver',
            executable='mir_bridge',
            name='mir_bridge',
            output='screen',
        ),
        Node(
            package='mir_driver',
            namespace='mir_driver',
            executable='scanners_merger',
            name='scanners_merger',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mir_description'), 'launch', 'description.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('publish_mir_description')),
        ),
    ])
