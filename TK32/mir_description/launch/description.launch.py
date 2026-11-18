from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    pkg_share = FindPackageShare("mir_description").find("mir_description")

    xacro_file = os.path.join(pkg_share, "urdf", "mir.urdf.xacro")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro", " ", xacro_file]),
                    value_type=str
                )
            }],
            output="screen"
        ),

        #Node(
        #    package="joint_state_publisher",
        #    executable="joint_state_publisher"
        #)
    ])