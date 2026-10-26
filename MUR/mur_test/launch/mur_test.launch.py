from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mur_test'), 'config', 'mur_test.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='mur_test', executable='mur_test', name='mur_test',
             output='screen', emulate_tty=True, parameters=[LaunchConfiguration('config')]),
    ])
