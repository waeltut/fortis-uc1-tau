from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('volume_file',default_value='~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz'),
        DeclareLaunchArgument('minimum_coverage',default_value='0.9'),
        DeclareLaunchArgument('profile_angle_deg',default_value='-1.0'),
        DeclareLaunchArgument('arm',default_value='both'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='mur_base_selector',executable='volume_visualizer',name='mur_volume_visualizer',output='screen',parameters=[{
            'volume_file':LaunchConfiguration('volume_file'),
            'minimum_coverage':ParameterValue(LaunchConfiguration('minimum_coverage'),value_type=float),
            'profile_angle_deg':ParameterValue(LaunchConfiguration('profile_angle_deg'),value_type=float),
            'arm':LaunchConfiguration('arm'),
            'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
        }]),
    ])
