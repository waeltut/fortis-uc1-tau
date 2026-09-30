"""Launch upstream rosbridge and rosapi with toolkit defaults."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_bridge(context):
    config_path = LaunchConfiguration('config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        settings = yaml.safe_load(stream)
    defaults = dict(address='0.0.0.0', port=9091, topics_glob='',
                    services_glob='', params_glob='')
    if not isinstance(settings, dict) or set(settings) - set(defaults):
        raise ValueError('Bridge config must be a mapping with only: ' + ', '.join(defaults))
    defaults.update(settings)
    for key in ('address', 'port'):
        override = LaunchConfiguration(key).perform(context)
        if override:
            defaults[key] = override
    raw_port = defaults['port']
    if isinstance(raw_port, bool) or not str(raw_port).isdigit():
        raise ValueError('port must be an integer from 1 to 65535')
    port = int(raw_port)
    if not 1 <= port <= 65535:
        raise ValueError('port must be an integer from 1 to 65535')
    defaults['port'] = str(port)
    for key in ('address', 'topics_glob', 'services_glob', 'params_glob'):
        if not isinstance(defaults[key], str):
            raise ValueError(key + ' must be a string')
    upstream = Path(get_package_share_directory('rosbridge_server')) / 'launch' / 'rosbridge_websocket_launch.xml'
    return [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(upstream)),
        launch_arguments=defaults.items(),
    )]


def generate_launch_description():
    config = str(Path(get_package_share_directory('toolkit_rosbridge')) / 'config' / 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('address', default_value='', description='Override YAML listen address'),
        DeclareLaunchArgument('port', default_value='', description='Override YAML WebSocket port'),
        DeclareLaunchArgument('demo', default_value='false', description='Start isolated test topics/service'),
        OpaqueFunction(function=launch_bridge),
        Node(package='toolkit_rosbridge', executable='demo_node', output='screen',
             condition=IfCondition(LaunchConfiguration('demo'))),
    ])
