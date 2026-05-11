from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        )
    ])