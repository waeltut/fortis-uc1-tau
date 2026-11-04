"""Opt-in diagnostic process; never included by base_selector.launch.py."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    path = Path(LaunchConfiguration('selector_config').perform(context)).expanduser()
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict): raise ValueError('Invalid selector parameter YAML')
    parameters = dict(data.get('/**', {}).get('ros__parameters', {}))
    selected = data.get('mur_base_selector', data.get('/mur_base_selector', {}))
    parameters.update(selected.get('ros__parameters', {}))
    if not parameters: raise ValueError('selector_config needs mur_base_selector or /** ros__parameters')
    # Load the selector's actual tuning under the diagnostic node's own name.
    return [Node(package='mur_base_selector', executable='score_visualizer',
        name='mur_score_visualizer', output='screen', parameters=[parameters,
            LaunchConfiguration('diagnostic_config'),
            {'diagnostic.mode': LaunchConfiguration('mode'),
             'arm_mode': LaunchConfiguration('arm_mode'),
             'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])]


def generate_launch_description():
    config = Path(get_package_share_directory('mur_base_selector'))/'config'
    return LaunchDescription([
        DeclareLaunchArgument('selector_config', default_value=str(config/'volume_selector.yaml')),
        DeclareLaunchArgument('diagnostic_config', default_value=str(config/'score_visualizer.yaml')),
        DeclareLaunchArgument('mode', default_value='fast', choices=['fast', 'detailed']),
        DeclareLaunchArgument('arm_mode', default_value='either'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start),
    ])
