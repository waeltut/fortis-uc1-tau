from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    execute = LaunchConfiguration("execute")
    reference_frame = LaunchConfiguration("reference_frame")

    # ---------------------------------------------------------
    # MiR driver
    # ---------------------------------------------------------

    mir_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mir_driver"),
                    "launch",
                    "mir_launch.py",
                ]
            )
        )
    )

    # ---------------------------------------------------------
    # Dual UR5e driver
    # ---------------------------------------------------------

    dual_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dual_arm_driver"),
                    "launch",
                    "dual_arm_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "execute": execute,
            "reference_frame": reference_frame,
        }.items(),
    )

    # ---------------------------------------------------------
    # MiR -> dual-arm mounting transform
    #
    # MiR TF:
    #   base_odomprint -> base_link
    #
    # Combined:
    #   base_odomprint -> base_link -> mur -> ...
    # ---------------------------------------------------------

    mir_to_mur_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mir_to_mur_tf",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.3125",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "mur",
        ],
    )

    mur_footprint = Node(
        package='mur_driver',
        executable='mur_footprint',
        name='mur_footprint',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'robot_description_topic': '/robot_description',
            'publish_rate': 10.0,
            'padding': 0.03,
        }],
    )

        # Saved map + localisation
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": os.path.expanduser(
                "~/ros2_ws/maps/leonardos.yaml"
            ),
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "global_frame_id": "map",
            "odom_frame_id": "odom",
            "base_frame_id": "base_link",
            "scan_topic": "/scan",
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "tf_broadcast": True,
            "transform_tolerance": 1.0,
            "min_particles": 500,
            "max_particles": 2000,
            "update_min_d": 0.1,
            "update_min_a": 0.1,

            # Initially wait for a pose estimate on /initialpose.
            "set_initial_pose": True,

            # Replace these after matching home to the map.
            "initial_pose.x": -0.007,
            "initial_pose.y": -0.032,
            "initial_pose.z": 0.0,
            "initial_pose.yaw": -0.10127,
        }],
    )

    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_config"),
                "launch",
                "nav2_launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "false",
            "map": os.path.expanduser(
                "~/ros2_ws/maps/leonardos.yaml"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description="Execute MoveIt trajectories",
            ),

            DeclareLaunchArgument(
                "reference_frame",
                default_value="mur",
                description="Reference frame used by the dual-arm control nodes",
            ),

            mir_driver,
            dual_arm_driver,
            mir_to_mur_tf,
            mur_footprint,
            #nav2,

            #map_server,
            #amcl,
            #localization_manager,
        ]
    )