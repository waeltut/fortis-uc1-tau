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
            'enable_sync': True,
            'enable_rgbd':True,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'align_depth.enable': True,
            'pointcloud__neon_.enable': False,
            'enable_imu': False
        }]
    )
    mic_publisher_node = Node(
        package='mic_driver',
        executable='mic_publisher',
    )

    # Transform so that the realsense-camera is a part of the tf-tree
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform',
        arguments=[
            '0.0', '0.0', '0.235',
            '0.0', '-0.13237', '0.0',    # For some reason, the MiR-camera-frame is rotated (7.58 degrees around y) :))))))
            'camera_floor_link',
            'camera_link'
        ]
    )

    # TK22 Nodes
    hearing_node = Node(
        package='human_perception',
        executable='hearing',
    )
    vision_node = Node(
        package='human_perception',
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
    pkg_dir = get_package_share_directory('human_robot_interaction')
    config_path = os.path.join(pkg_dir, 'param_config', 'param_sets.yaml')
    
    mm_audio_descriptor_node = Node(
        package='human_robot_interaction',
        executable='audio_descriptor_processing_node',
        name='audio_descriptor_processing_node',
        parameters=[config_path]
    )
    
    mm_audio_transcription_node = Node(
        package='human_robot_interaction',
        executable='audio_transcription_node',
        name='audio_transcription_node',
        parameters=[config_path]
    )
    mm_visual_classification_node = Node(
        package='human_robot_interaction',
        executable='visual_cue_classification_node',
        name='visual_cue_classification_node',
        parameters=[config_path]
    )
    mm_visual_descriptor_node = Node(
        package='human_robot_interaction',
        executable='visual_descriptor_processing_node',
        name='visual_descriptor_processing_node',
        parameters=[config_path]
    )
    mm_correlator_node = Node(
        package='human_robot_interaction',
        executable='correlator_node',
        name='correlator_node',
        parameters=[config_path]
    )
    mm_fusion_node = Node(
        package='human_robot_interaction',
        executable='fusion_node',
        name='fusion_node',
        parameters=[config_path]
    )

    mm_plotting_node = Node(
        package='human_robot_interaction',
        executable='plotting_node',
        name='plotting_node',
        parameters=[config_path]
    )

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
    motivation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('reasoning_controller'), 'launch', 'reasoning_launch.py')),
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
    ld.add_action(realsense_node)
    ld.add_action(mic_publisher_node)
    ld.add_action(static_camera_tf)
    ld.add_action(hearing_node)
    ld.add_action(vision_node)
    # ld.add_action(quality_assessment_node)
    # ld.add_action(audio_analyser_node)
    # ld.add_action(mir_launch)
    # #ld.add_action(slam_node)
    ld.add_action(nav2_launch)
    # ld.add_action(basic_move_node)
    # ld.add_action(coordinator_node)
    # ld.add_action(aggregate_node)
    # ld.add_action(capability_node)
    # ld.add_action(inform_node)
    # ld.add_action(replan_node)
    # ld.add_action(respond_node)
    # ld.add_action(work_node)
    # ld.add_action(mm_audio_descriptor_node)
    # ld.add_action(mm_audio_transcription_node)
    # ld.add_action(mm_visual_classification_node)
    # ld.add_action(mm_visual_descriptor_node)
    # ld.add_action(mm_correlator_node)
    # ld.add_action(mm_fusion_node)
    # ld.add_action(mm_plotting_node)
    # ld.add_action(motivation_launch)
    

    ### Launching nodes for Mapping
    #ld.add_action(declare_slam_params_file)
    #ld.add_action(slam_node)


    ### Launching nodes for Recording Ros bags
    # ld.add_action(realsense_node)
    # ld.add_action(mic_publisher_node)
    # ld.add_action(mir_bridge_node)


    ### Launching nodes for rosbag playing
    #ld.add_action(declare_slam_params_file)
    # ld.add_action(foxglove_bridge_node)
    # ld.add_action(hearing_node)
    # ld.add_action(vision_node)
    # ld.add_action(proximity_node)
    # ld.add_action(audio_analyser_node)
    # ld.add_action(quality_assessment_node)
    #ld.add_action(slam_node)
    #ld.add_action(nav2_launch)
    # ld.add_action(basic_move_node)
    # ld.add_action(coordinator_node)
    # ld.add_action(aggregate_node)
    # ld.add_action(capability_node)
    # ld.add_action(inform_node)
    # ld.add_action(replan_node)
    # ld.add_action(respond_node)
    # ld.add_action(work_node)

    return ld
