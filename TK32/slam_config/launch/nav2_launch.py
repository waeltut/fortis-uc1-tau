import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------
    bringup_dir = get_package_share_directory('slam_config')

    default_map = os.path.join(
        bringup_dir, 'maps', 'map_6_1.yaml')

    default_params = os.path.join(
        bringup_dir, 'config', 'nav2_params.yaml')

    # --------------------------------------------------
    # Launch configurations
    # --------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # --------------------------------------------------
    # Rewritten params
    # --------------------------------------------------
    configured_params = params_file

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to nav2 parameters file'),

        # ==================================================
        # MAP SERVER
        # ==================================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params,
                        {'yaml_filename': map_yaml}],
        ),

        # ==================================================
        # AMCL
        # ==================================================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
        ),

        # ==================================================
        # LIFECYCLE MANAGER – LOCALIZATION
        # ==================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl'
                ]
            }],
        ),

        # ==================================================
        # NAV2 SERVERS
        # ==================================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
        ),

        # ==================================================
        # LIFECYCLE MANAGER – NAVIGATION
        # ==================================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server',
                    'waypoint_follower',
                    'velocity_smoother'
                ]
            }],
        ),
    ])