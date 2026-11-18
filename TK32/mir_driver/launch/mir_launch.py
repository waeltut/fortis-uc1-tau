from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Nodes / launch-files
    bridge_node = Node(
        package='mir_driver',
        namespace='mir_driver',
        executable='mir_bridge',
        name='mir_bridge',
        output='screen',
    )
    scanner_node = Node(
        package='mir_driver',
        namespace='mir_driver',
        executable='scanners_merger',
        name='scanners_merger',
        output='screen',
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mir_description'), 'launch', 'description.launch.py')),
    )

    # What to launch
    ld.add_action(bridge_node)
    ld.add_action(scanner_node)
    ld.add_action(description_launch)

    return ld