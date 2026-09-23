# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
#
# Joint-velocity variant of duo_ur_real.launch.py.
# Mirrors the sim -> sim_joint reduction applied in duo_ur_sim_joint.launch.py:
#   - No MoveIt (no move_group, no MoveItConfigsBuilder)
#   - No RViz MoveIt plugin config (plain robot_description only)
#   - No table_node / moveit_interface_node
#   - Only joint_state_broadcaster + left/right forward_velocity_controller spawned
#   - Adds cartesian_to_joint_velocity node from dual_arm_app
# All real-robot infrastructure (ur_ros2_control_node, dashboard_client,
# urscript_interface, controller_stopper, xacro args, etc.) is preserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip_1 = LaunchConfiguration("robot_ip_1")
    robot_ip_2 = LaunchConfiguration("robot_ip_2")

    script_command_port_1 = LaunchConfiguration("script_command_port_1")
    reverse_port_1 = LaunchConfiguration("reverse_port_1")
    script_sender_port_1 = LaunchConfiguration("script_sender_port_1")
    trajectory_port_1 = LaunchConfiguration("trajectory_port_1")

    script_command_port_2 = LaunchConfiguration("script_command_port_2")
    reverse_port_2 = LaunchConfiguration("reverse_port_2")
    script_sender_port_2 = LaunchConfiguration("script_sender_port_2")
    trajectory_port_2 = LaunchConfiguration("trajectory_port_2")

    tf_prefix_2 = LaunchConfiguration("tf_prefix_1")
    tf_prefix_1 = LaunchConfiguration("tf_prefix_2")

    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("duo_ur"), "urdf", "duo_ur_onehand.urdf.xacro"]),
            " ",
            "robot_ip_1:=", robot_ip_1, " ",
            "robot_ip_2:=", robot_ip_2, " ",
            "joint_limit_params:=", joint_limit_params, " ",
            "kinematics_params:=", kinematics_params_file, " ",
            "physical_params:=", physical_params, " ",
            "visual_params:=", visual_params, " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=", "ur", " ",
            "script_filename:=", script_filename, " ",
            "input_recipe_filename:=", input_recipe_filename, " ",
            "output_recipe_filename:=", output_recipe_filename, " ",
            "tf_prefix_1:=", tf_prefix_1, " ",
            "tf_prefix_2:=", tf_prefix_2, " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "fake_sensor_commands:=", fake_sensor_commands, " ",
            "headless_mode:=", headless_mode, " ",
            "use_tool_communication:=", use_tool_communication, " ",
            "tool_parity:=", tool_parity, " ",
            "tool_baud_rate:=", tool_baud_rate, " ",
            "tool_stop_bits:=", tool_stop_bits, " ",
            "tool_rx_idle_chars:=", tool_rx_idle_chars, " ",
            "tool_tx_idle_chars:=", tool_tx_idle_chars, " ",
            "tool_device_name:=", tool_device_name, " ",
            "tool_tcp_port:=", tool_tcp_port, " ",
            "tool_voltage:=", tool_voltage, " ",
            "reverse_ip:=", reverse_ip, " ",
            "script_command_port_1:=", script_command_port_1, " ",
            "reverse_port_1:=", reverse_port_1, " ",
            "script_sender_port_1:=", script_sender_port_1, " ",
            "trajectory_port_1:=", trajectory_port_1, " ",
            "script_command_port_2:=", script_command_port_2, " ",
            "reverse_port_2:=", reverse_port_2, " ",
            "script_sender_port_2:=", script_sender_port_2, " ",
            "trajectory_port_2:=", trajectory_port_2, " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # Simple RViz config (no MoveIt plugin)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("duo_ur"), "rviz", "cartesian_control_duo_ur5e.rviz"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )

    # Fake-hardware control node (used when use_fake_hardware:=true)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # Real UR control node
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node_1 = Node(
        package="ur_robot_driver",
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        executable="dashboard_client",
        name="left_dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip_1}],
    )

    dashboard_client_node_2 = Node(
        package="ur_robot_driver",
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        executable="dashboard_client",
        name="right_dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip_2}],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip_1,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )

    urscript_interface_1 = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip_1}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    urscript_interface_2 = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip_2}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # Controller stoppers — keep the forward_velocity_controller consistent so
    # it isn't stopped on protective-stop / e-stop recoveries.
    controller_stopper_node_1 = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="left_controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": True},
            {
                "consistent_controllers": [
                    "left_io_and_status_controller",
                    "left_force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "left_speed_scaling_state_broadcaster",
                    "left_tcp_pose_broadcaster",
                    "left_ur_configuration_controller",
                    "left_forward_velocity_controller",
                ]
            },
        ],
    )

    controller_stopper_node_2 = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="right_controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": True},
            {
                "consistent_controllers": [
                    "right_io_and_status_controller",
                    "right_force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "right_speed_scaling_state_broadcaster",
                    "right_tcp_pose_broadcaster",
                    "right_ur_configuration_controller",
                    "right_forward_velocity_controller",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --- Spawners: only what the joint-velocity pipeline needs ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        output="screen",
    )

    left_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_forward_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        output="screen",
    )

    right_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_forward_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_spawner_timeout,
        ],
        output="screen",
    )

    left_hand_node = Node(
    package="hand_control",
    executable="hand_control",
    name="left_hand_control",
    output="screen",
    parameters=[{"hand": "L"}],
    )

    right_hand_node = Node(
        package="hand_control",
        executable="hand_control",
        name="right_hand_control",
        output="screen",
        parameters=[{"hand": "R"}],
    )

    # On the real robot either control_node (fake) or ur_control_node (real)
    # starts the controller_manager. Delay spawners until one of them is up.
    delay_controllers_fake = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(period=2.0, actions=[
                    joint_state_broadcaster_spawner,
                    left_velocity_spawner,
                    right_velocity_spawner,
                ])
            ],
        )
    )

    delay_controllers_real = RegisterEventHandler(
        OnProcessStart(
            target_action=ur_control_node,
            on_start=[
                TimerAction(period=10.0, actions=[
                    joint_state_broadcaster_spawner,
                    left_velocity_spawner,
                    right_velocity_spawner,
                ])
            ],
        )
    )

    # Cartesian -> joint velocity bridge (same node as sim_joint)
    cartesian_to_joint_velocity_node = Node(
        package="dual_arm_app",
        executable="cartesian_to_joint_velocity",
        parameters=[robot_description],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    nodes_to_start = [
        control_node,
        ur_control_node,
        dashboard_client_node_1,
        dashboard_client_node_2,
        tool_communication_node,
        urscript_interface_1,
        urscript_interface_2,
        controller_stopper_node_1,
        controller_stopper_node_2,
        robot_state_publisher_node,
        delay_controllers_fake,
        delay_controllers_real,
        cartesian_to_joint_velocity_node,
        rviz_node,
        left_hand_node,
        right_hand_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e",
                "ur12e", "ur16e", "ur15", "ur20", "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_1", description="IP address by which the robot can be reached.",
            default_value="192.168.1.102",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_2", description="IP address by which the robot can be reached.",
            default_value="192.168.1.103",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="duo_ur_adv_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("ur_type"),
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_1",
            default_value="left_",
            description="tf_prefix of the joint names for arm 1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_2",
            default_value="right_",
            description="tf_prefix of the joint names for arm 2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="false", description="Launch Dashboard Client?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_parity", default_value="0")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_baud_rate", default_value="115200")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_stop_bits", default_value="1")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_tcp_port", default_value="54321")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tool_voltage", default_value="0")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="192.168.1.75",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("script_command_port_1", default_value="50004")
    )
    declared_arguments.append(
        DeclareLaunchArgument("reverse_port_1", default_value="50001")
    )
    declared_arguments.append(
        DeclareLaunchArgument("script_sender_port_1", default_value="50002")
    )
    declared_arguments.append(
        DeclareLaunchArgument("trajectory_port_1", default_value="50003")
    )
    declared_arguments.append(
        DeclareLaunchArgument("script_command_port_2", default_value="50014")
    )
    declared_arguments.append(
        DeclareLaunchArgument("reverse_port_2", default_value="50011")
    )
    declared_arguments.append(
        DeclareLaunchArgument("script_sender_port_2", default_value="50012")
    )
    declared_arguments.append(
        DeclareLaunchArgument("trajectory_port_2", default_value="50013")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])