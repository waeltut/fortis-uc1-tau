"""Fake hardware and MoveIt using the same configuration as real bringup."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("duo_ur"), "launch", "duo_ur_real.launch.py"])),
        launch_arguments={"use_fake_hardware": "true", "launch_rviz": "true"}.items(),
    )])
