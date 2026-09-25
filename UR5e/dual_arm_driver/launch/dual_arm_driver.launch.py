from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    reference_frame = LaunchConfiguration('reference_frame')
    execute = LaunchConfiguration('execute')

    left_group = LaunchConfiguration('left_group')
    right_group = LaunchConfiguration('right_group')

    moveit_config = (
        MoveItConfigsBuilder(
            "custom_robot",
            package_name="duo_ur5e_torso_moveit_config"
        )
        .robot_description_semantic(
            file_path="config/duo_ur_onehand.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .to_moveit_configs()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'reference_frame',
            default_value='chest',
            description='Default reference frame for TCP poses and goals'
        ),

        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='If true, MoveIt executes planned trajectories'
        ),

        DeclareLaunchArgument(
            'left_group',
            default_value='left_ur_manipulator',
            description='MoveIt planning group for the left arm'
        ),

        DeclareLaunchArgument(
            'right_group',
            default_value='right_ur_manipulator',
            description='MoveIt planning group for the right arm'
        ),

        Node(
            package='dual_arm_driver',
            executable='tcp_pose_publisher',
            name='dual_arm_pose_publisher',
            output='screen',
            parameters=[{
                'reference_frame': reference_frame,

                'left_reference_frame': '',
                'right_reference_frame': '',

                'left_tcp_frame': 'left_tcp',
                'right_tcp_frame': 'right_tcp',

                'publish_rate': 10.0,
            }]
        ),

        Node(
            package='moveit_utils_pkg',
            executable='dual_arm_pose_commander',
            name='dual_arm_moveit_commander',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    "left_group": "left_ur_manipulator",
                    "right_group": "right_ur_manipulator",

                    "left_end_effector_link": "left_tcp",
                    "right_end_effector_link": "right_tcp",

                    "execute": execute,

                    "velocity_scaling": 0.10,
                    "acceleration_scaling": 0.10,

                    "planning_time": 5.0,
                    "planning_attempts": 5,
                },
            ],
        ),
    ])