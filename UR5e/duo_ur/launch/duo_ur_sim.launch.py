import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()

    joint_controllers_file = os.path.join(
        get_package_share_directory("duo_ur"), "config", "duo_ur5e_controllers.yaml"
    )

    robot_description_file = os.path.join(
        get_package_share_directory("duo_ur"), "urdf", "duo_ur_onehand.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "custom_robot", package_name="duo_ur5e_torso_moveit_config"
        )
        .robot_description(robot_description_file)
        .robot_description_semantic(file_path="config/duo_ur_onehand.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

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
        ],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, joint_controllers_file],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("left_motion_control_handle/target_frame", "left_target_frame"),
            ("left_cartesian_motion_controller/target_frame", "left_target_frame"),
            ("left_cartesian_compliance_controller/target_frame", "left_target_frame"),
            ("left_cartesian_force_controller/target_wrench", "left_target_wrench"),
            (
                "left_cartesian_compliance_controller/target_wrench",
                "left_target_wrench",
            ),
            (
                "left_cartesian_force_controller/ft_sensor_wrench",
                "left_ft_sensor_wrench",
            ),
            (
                "left_cartesian_compliance_controller/ft_sensor_wrench",
                "left_ft_sensor_wrench",
            ),
            ("left_force_torque_sensor_broadcaster/wrench", "left_ft_sensor_wrench"),
            ("right_motion_control_handle/target_frame", "right_target_frame"),
            ("right_cartesian_motion_controller/target_frame", "right_target_frame"),
            (
                "right_cartesian_compliance_controller/target_frame",
                "right_target_frame",
            ),
            ("right_cartesian_force_controller/target_wrench", "right_target_wrench"),
            (
                "right_cartesian_compliance_controller/target_wrench",
                "right_target_wrench",
            ),
            (
                "right_cartesian_force_controller/ft_sensor_wrench",
                "right_ft_sensor_wrench",
            ),
            (
                "right_cartesian_compliance_controller/ft_sensor_wrench",
                "right_ft_sensor_wrench",
            ),
            ("right_force_torque_sensor_broadcaster/wrench", "right_ft_sensor_wrench"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

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
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
    ]
    controllers_inactive = [
        "left_scaled_joint_trajectory_controller",
        "left_forward_velocity_controller",
        "left_forward_position_controller",
        "left_cartesian_motion_controller",
        "left_motion_control_handle",
        "right_scaled_joint_trajectory_controller",
        "right_forward_velocity_controller",
        "right_forward_position_controller",
        "right_cartesian_motion_controller",
        "right_motion_control_handle",
    ]

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    use_sim_time = {"use_sim_time": False}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=controller_spawners,
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[rviz_node],
        )
    )

    ld.add_action(controller_manager_node)  # has to be loaded first
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_rviz_node)

    return ld
