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
    # gazebo_launch_file = os.path.join(
    #     get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    # )h

    robot_description_file = os.path.join(get_package_share_directory("duo_ur"), "urdf", "duo_ur_onehand.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="duo_ur5e_torso_moveit_config")
        .robot_description(robot_description_file)
        .robot_description_semantic(file_path="config/duo_ur_onehand.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )


    # x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    # y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    # z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')

    # Include Gazebo launch file
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gazebo_launch_file),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'debug': 'false',
    #         'gui': 'true',
    #         'paused': 'true',
    #         #'world' : world_file
    #     }.items()
    # )

    # 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("duo_ur"), "rviz", "adv_control_duo_ur5e.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # {"use_sim_time": True}
        ],
    )

    # # spawn the robot
    # spawn_the_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'ur',
    #         '-topic', 'robot_description',
    #         '-x', LaunchConfiguration('x'),
    #         '-y', LaunchConfiguration('y'),
    #         '-z', LaunchConfiguration('z')
    #     ],
    #     output='screen',
    # )

    # controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, joint_controllers_file],
        output='screen',
        remappings=[
            ("~/robot_description", "/robot_description"),
            
            ('left_motion_control_handle/target_frame', 'left_target_frame'),
            ('left_cartesian_motion_controller/target_frame', 'left_target_frame'),
            ('left_cartesian_compliance_controller/target_frame', 'left_target_frame'),
            ('left_cartesian_force_controller/target_wrench', 'left_target_wrench'),
            ('left_cartesian_compliance_controller/target_wrench', 'left_target_wrench'),
            ('left_cartesian_force_controller/ft_sensor_wrench', 'left_ft_sensor_wrench'),
            ('left_cartesian_compliance_controller/ft_sensor_wrench', 'left_ft_sensor_wrench'),
            ('left_force_torque_sensor_broadcaster/wrench', 'left_ft_sensor_wrench'),
            ('right_motion_control_handle/target_frame', 'right_target_frame'),
            ('right_cartesian_motion_controller/target_frame', 'right_target_frame'),
            ('right_cartesian_compliance_controller/target_frame', 'right_target_frame'),
            ('right_cartesian_force_controller/target_wrench', 'right_target_wrench'),
            ('right_cartesian_compliance_controller/target_wrench', 'right_target_wrench'),
            ('right_cartesian_force_controller/ft_sensor_wrench', 'right_ft_sensor_wrench'),
            ('right_cartesian_compliance_controller/ft_sensor_wrench', 'right_ft_sensor_wrench'),
            ('right_force_torque_sensor_broadcaster/wrench', 'right_ft_sensor_wrench'),

        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            moveit_config.robot_description,
            # {"use_sim_time": True}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", 
            "/controller_manager"],
        output="screen",
    )

    # left_joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "left_joint_state_broadcaster", 
    #         "--controller-manager", 
    #         "/controller_manager"],
    #     output="screen",
    # )

    # left_arm_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "left_joint_trajectory_controller",
    #         "--inactive",  
    #         "--controller-manager", 
    #         "/controller_manager"],
    #     output="screen",
    # )

    # left_cartesian_motion_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "left_cartesian_motion_controller", 
    #         # "--inactive", 
    #         "--controller-manager", 
    #         "/controller_manager"
    #     ],
    #     output="screen",
    # )

    # right_joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "left_joint_state_broadcaster", 
    #         "--controller-manager", 
    #         "/controller_manager"],
    #     output="screen",
    # )

    # right_arm_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "right_joint_trajectory_controller", 
    #         "--inactive", 
    #         "--controller-manager", 
    #         "/controller_manager"],
    #     output="screen",
    # )

    # right_cartesian_motion_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "right_cartesian_motion_controller", 
    #         # "--inactive", 
    #         "--controller-manager", 
    #         "/controller_manager"
    #     ],
    #     output="screen",
    # )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10",
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        #### "joint_state_broadcaster",
        #"left_io_and_status_controller",
        #"left_speed_scaling_state_broadcaster",
        #"left_force_torque_sensor_broadcaster",
        "left_tcp_pose_broadcaster",
        #"left_ur_configuration_controller",
        # "left_cartesian_motion_controller",
        "left_joint_trajectory_controller",
        #"right_io_and_status_controller",
        #"right_speed_scaling_state_broadcaster",
        #"right_force_torque_sensor_broadcaster",
        "right_tcp_pose_broadcaster",
        #"right_ur_configuration_controller",
        # "right_cartesian_motion_controller",
        "right_joint_trajectory_controller",
    ]
    controllers_inactive = [
        "left_scaled_joint_trajectory_controller",
        #"left_joint_trajectory_controller",
        "left_forward_velocity_controller",
        "left_forward_position_controller",
        #"left_force_mode_controller",
        #"left_passthrough_trajectory_controller",
        #"left_freedrive_mode_controller",
        #"left_tool_contact_controller",
        #"left_cartesian_compliance_controller",
        #"left_cartesian_force_controller",
        "left_cartesian_motion_controller",
        "left_motion_control_handle",
        "right_scaled_joint_trajectory_controller",
        #"right_joint_trajectory_controller",
        "right_forward_velocity_controller",
        "right_forward_position_controller",
        #"right_force_mode_controller",
        #"right_passthrough_trajectory_controller",
        #"right_freedrive_mode_controller",
        #"right_tool_contact_controller",
        #"right_cartesian_compliance_controller",
        #"right_cartesian_force_controller",
        "right_cartesian_motion_controller",
        "right_motion_control_handle",
    ]

    # initial_joint_controllers = PathJoinSubstitution(
    #     [FindPackageShare(runtime_config_package), "config", controllers_file]
    # )

    # if activate_joint_controller.perform(context) == "true":
    # controllers_active.append(initial_joint_controller.perform(context))
    # controllers_inactive.remove(initial_joint_controller.perform(context))


    # if use_fake_hardware.perform(context) == "true":
    if True:
        controllers_active.remove("left_tcp_pose_broadcaster")
        controllers_active.remove("right_tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    # gripper_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    use_sim_time={"use_sim_time": False}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    
    #######################
    # octomap_config = {
    #     'octomap_frame': 'world',
    #     'octomap_resolution': 0.01,
    #     'max_range': 0.1
    # }
    # octomap_updater_config_path = os.path.join(
    #     get_package_share_directory('duo_ur5e_torso_moveit_config'), 
    #     'config', 
    #     'sensors_3d.yaml'
    # )
    # try:
    #     with open(octomap_updater_config_path, 'r') as f:
    #         octomap_updater_config = yaml.safe_load(f)
    # except FileNotFoundError:
    #     print(f"sensors_3d.yaml not found at {octomap_updater_config_path}")
    #     octomap_updater_config = {}
    #######################

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    table_node = Node(
        package="moveit_utils_pkg",
        executable="table_node",
        output="screen",
    )
    delay_table_node = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[TimerAction(period=2.0, actions=[table_node])]
        )
    )


    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[
                joint_state_broadcaster_spawner,
                # left_joint_state_broadcaster_spawner,
                # right_joint_state_broadcaster_spawner,
            ],
        )
    )

    # delay_arm_controller = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=move_group_node,
    #         on_start=[
    #             left_arm_trajectory_controller_spawner,
    #             right_arm_trajectory_controller_spawner,
    #         ],
    #     )
    # )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=controller_spawners,
        )
    )

    #controller_spawners

    # delay_cartesian_controller = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=move_group_node,
    #         on_start=[
    #             left_cartesian_motion_controller_spawner,
    #             right_cartesian_motion_controller_spawner,
    #         ],
    #     )
    # )
    

    # delay_gripper_controller = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_start=[gripper_position_controller_spawner],
    #     )
    # )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[rviz_node],
        )
    )

    moveit_interface_node = Node(
        package="moveit_utils_pkg",
        executable="moveit_interface_node",
        output="screen",
    )

    # Launch Description
    # ld.add_action(x_arg)
    # ld.add_action(y_arg)
    # ld.add_action(z_arg)
    # ld.add_action(gazebo)
    ld.add_action(controller_manager_node)  # has to be loaded first
    # ld.add_action(spawn_the_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)
    # delay of the controllers
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    # ld.add_action(delay_gripper_controller)
    # ld.add_action(delay_cartesian_controller)
    ld.add_action(delay_rviz_node)
    ld.add_action(delay_table_node)

    ld.add_action(moveit_interface_node)
    
    
    #ld.add_action(humanoid_hand_node)

    return ld
