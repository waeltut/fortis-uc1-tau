import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mur_reachability'), 'config', 'reachability.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('start_builder', default_value='true'),
        Node(package='mur_reachability', executable='cache_server', name='reachability_cache',
             parameters=[LaunchConfiguration('config')], output='screen',
             condition=IfCondition(LaunchConfiguration('start_builder'))),
        Node(package='mur_reachability', executable='base_candidates', name='base_candidates',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
