from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.event_handlers import OnProcessStart, OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)

from launch_param_builder import load_yaml
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    joint_controllers_file = os.path.join(
        get_package_share_directory('duo_ur'), 'config', 'duo_ur5e_controllers.yaml'
    )

    robot_description_content = Command(
        ['xacro ', os.path.join(
            get_package_share_directory("duo_ur"), "urdf", "duo_ur_onehand.urdf.xacro"
        )]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller manager — no MoveIt config, just the URDF + controller YAML
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, joint_controllers_file],
        output='screen',
        remappings=[
            ("~/robot_description", "/robot_description"),
            ('left_cartesian_motion_controller/target_frame', 'left_target_frame'),
            ('right_cartesian_motion_controller/target_frame', 'right_target_frame'),
            # keep only the remappings you actually need
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    left_cartesian_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_cartesian_motion_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    right_cartesian_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_cartesian_motion_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn controllers only after controller_manager is up
    delay_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(period=2.0, actions=[
                    joint_state_broadcaster_spawner,
                    left_cartesian_spawner,
                    right_cartesian_spawner,
                ])
            ]
        )
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("duo_ur"), "rviz", "cartesian_control_duo_ur5e.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Use a simple rviz config, no MoveIt plugin needed
        parameters=[robot_description],
        arguments=["-d", rviz_config_file],
    )

    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(delay_controllers)
    ld.add_action(rviz_node)

    return ld
