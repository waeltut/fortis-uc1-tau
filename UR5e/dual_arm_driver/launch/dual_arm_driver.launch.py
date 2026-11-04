from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    start_hands = LaunchConfiguration("start_hands")
    initialize_payload = LaunchConfiguration("initialize_payload")
    reference_frame = LaunchConfiguration("reference_frame")
    headless_mode = LaunchConfiguration("headless_mode")

    left_group = LaunchConfiguration("left_group")
    right_group = LaunchConfiguration("right_group")
    dual_group = LaunchConfiguration("dual_group")
    commander_config = LaunchConfiguration("commander_config")
    presets_file = LaunchConfiguration("presets_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("start_hands", default_value="true"),
            DeclareLaunchArgument("initialize_payload", default_value="true"),
            DeclareLaunchArgument(
                "reference_frame",
                default_value="mur",
                description="Reference frame for published TCP poses; goals carry their own frame",
            ),
            DeclareLaunchArgument(
                "headless_mode",
                default_value="false",
                description="Run UR External Control directly in Remote Control mode without pressing pendant Play",
            ),
            DeclareLaunchArgument(
                "left_group",
                default_value="left_ur_manipulator",
                description="MoveIt planning group for the left arm",
            ),
            DeclareLaunchArgument(
                "right_group",
                default_value="right_ur_manipulator",
                description="MoveIt planning group for the right arm",
            ),
            DeclareLaunchArgument(
                "dual_group",
                default_value="both_ur_manipulator",
                description="Existing combined MoveIt planning group for both arms",
            ),
            DeclareLaunchArgument("commander_config", default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_commander"), "config", "commander.yaml"])),
            DeclareLaunchArgument("presets_file", default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_commander"), "config", "presets.yaml"])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("duo_ur"),
                            "launch",
                            "duo_ur_real.launch.py",
                        ]
                    )
                ),
                launch_arguments={"headless_mode": headless_mode,
                                  "use_fake_hardware": use_fake_hardware}.items(),
            ),

            Node(
                package="dual_arm_driver",
                executable="tcp_pose_publisher",
                name="dual_arm_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "reference_frame": reference_frame,
                        "left_reference_frame": "",
                        "right_reference_frame": "",
                        "left_tcp_frame": "left_tcp",
                        "right_tcp_frame": "right_tcp",
                        "publish_rate": 10.0,
                    }
                ],
            ),
            Node(
                package="dual_arm_commander",
                executable="arm_commander",
                name="dual_arm_commander",
                output="screen",
                parameters=[
                    commander_config,
                    {"robot_description": ParameterValue(
                        LaunchConfiguration("dual_arm_robot_description"), value_type=str),
                     "presets_file": presets_file,
                     "left_group": left_group,
                     "right_group": right_group,
                     "dual_group": dual_group},
                ],
            ),
            # Apply configured mass, CoG and optional inertia once each UR program runs.
            # Requires both /<side>_io_and_status_controller/set_payload services.
            Node(
                package="dual_arm_driver",
                executable="payload_initializer",
                condition=IfCondition(AndSubstitution(initialize_payload, NotSubstitution(use_fake_hardware))),
                name="payload_initializer",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("dual_arm_driver"), "config", "payload.yaml"]
                    )
                ],
            ),
            Node(
                package="hand_control",
                executable="hand_control",
                condition=IfCondition(AndSubstitution(start_hands, NotSubstitution(use_fake_hardware))),
                name="left_hand_control",
                output="screen",
                parameters=[{"hand": "L"}],
            ),

            Node(
                package="hand_control",
                executable="hand_control",
                condition=IfCondition(AndSubstitution(start_hands, NotSubstitution(use_fake_hardware))),
                name="right_hand_control",
                output="screen",
                parameters=[{"hand": "R"}],
            ),
        ]
    )
