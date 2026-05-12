from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    
    # TK21 Nodes
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'pointcloud.enable': False,
            'enable_imu': False
        }]
    )
    mic_publisher_node = Node(
        package='mic_driver',
        executable='mic_publisher',
    )

    # TK22 Nodes
    hearing_node = Node(
        package='sensing_controller',
        executable='hearing',
    )
    vision_node = Node(
        package='sensing_controller',
        executable='vision',
    )
    audio_analyser_node = Node(
        package='environment_perception',
        executable='audio_analyser',
    )
    quality_assessment_node = Node(
        package='environment_perception',
        executable='video_quality_assessment',
    )

    # TK23 Nodes
    # TODO: Add TK23 nodes here

    # TK24 Nodes
    # TODO: Add TK24 nodes here

    # TK32 Nodes    
    mir_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mir_driver'), 'launch', 'mir_launch.py')),
    )
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        #default_value=os.path.join(get_package_share_directory('slam_config'), 'config', 'mapper_params_localization.yaml'),    # <-- Use this for localization
        default_value=os.path.join(get_package_share_directory('slam_config'), 'config', 'mapper_params_online_async.yaml'),    # <-- Use this for mapping
        description='Full path to SLAM params YAML')
    slam_params_file = LaunchConfiguration('slam_params_file') 
    slam_node = Node(
        package='slam_toolbox',
        #executable='localization_slam_toolbox_node',    # <-- Use this for localization
        executable='async_slam_toolbox_node',    # <-- Use this for mapping
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
        ]
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_config'), 'launch', 'nav2_launch.py')),
    )

    # TK33 Nodes
    basic_move_node = Node(
        package='move_controller',
        executable='basic_move',
    )
    coordinator_node = Node(
        package='move_controller',
        executable='move_coordinator',
    )
    aggregate_node = Node(
        package='reasoning_controller',
        executable='aggregate_motivation',
    )
    capability_node = Node(
        package='reasoning_controller',
        executable='motivation_capability',
    )
    inform_node = Node(
        package='reasoning_controller',
        executable='motivation_inform',
    )
    replan_node = Node(
        package='reasoning_controller',
        executable='motivation_replan',
    )
    respond_node = Node(
        package='reasoning_controller',
        executable='motivation_respond',
    )
    work_node = Node(
        package='reasoning_controller',
        executable='motivation_work',
    )

    # SA41 Nodes
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge'
        )

    ### Starting Sequence ###


    ### Launching nodes in sequence ###
    # TODO: add the launch correct order 

    ### Launching nodes in parallel (uncomment if you want to launch all nodes at once) ###
    # ld.add_action(foxglove_bridge_node)
    # ld.add_action(realsense_node)
    # ld.add_action(mic_publisher_node)
    # ld.add_action(hearing_node)
    # ld.add_action(vision_node)
    # ld.add_action(audio_analyser_node)
    # ld.add_action(quality_assessment_node)
    # ld.add_action(mir_launch)
    # ld.add_action(slam_node)
    #ld.add_action(nav2_launch)
    # ld.add_action(basic_move_node)
    # ld.add_action(coordinator_node)
    # ld.add_action(aggregate_node)
    # ld.add_action(capability_node)
    # ld.add_action(inform_node)
    # ld.add_action(replan_node)
    # ld.add_action(respond_node)
    # ld.add_action(work_node)


    ### Launching nodes for Mapping
    # ld.add_action(declare_slam_params_file)
    # ld.add_action(mic_publisher_node)
    #ld.add_action(realsense_node)
    # ld.add_action(slam_node)


    ### Launching nodes for Recording Ros bags
    # ld.add_action(realsense_node)
    # ld.add_action(mic_publisher_node)
    # ld.add_action(mir_bridge_node)


    ### Launching nodes for rosbag playing
    # ld.add_action(declare_slam_params_file)
    # ld.add_action(foxglove_bridge_node)
    # ld.add_action(hearing_node)
    # ld.add_action(vision_node)
    # ld.add_action(proximity_node)
    # ld.add_action(audio_analyser_node)
    # ld.add_action(quality_assessment_node)
    # ld.add_action(slam_node)
    # ld.add_action(nav2_launch)
    # ld.add_action(basic_move_node)
    # ld.add_action(coordinator_node)
    # ld.add_action(aggregate_node)
    # ld.add_action(capability_node)
    # ld.add_action(inform_node)
    # ld.add_action(replan_node)
    # ld.add_action(respond_node)
    # ld.add_action(work_node)

    return ld
