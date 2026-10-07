import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mur_base_selector'), 'config', 'base_selector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('selector_config', default_value=config),
        DeclareLaunchArgument('arm_mode', default_value='either'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='mur_base_selector', executable='base_selector', name='mur_base_selector',
             output='screen', parameters=[LaunchConfiguration('selector_config'),
                 {'arm_mode': LaunchConfiguration('arm_mode'), 'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
