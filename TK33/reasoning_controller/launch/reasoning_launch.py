from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='motivation_work',
            name='motivation_work',
            output='screen',
        ),
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='motivation_replan',
            name='motivation_replan',
            output='screen',
        ),
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='motivation_inform',
            name='motivation_inform',
            output='screen',
        ),
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='motivation_capability',
            name='motivation_capability',
            output='screen',
        ),
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='motivation_respond',
            name='motivation_respond',
            output='screen',
        ),
        Node(
            package='reasoning_controller',
            namespace='reasoning',
            executable='aggregate_motivation',
            name='aggregate_motivation',
            output='screen',
        ),
    ])