"""Deployment entrypoint for the TK2.4 TTS node.

All runtime configuration (language, output, topics, ...) comes from
``TK24_TTS_*`` environment variables, which the node inherits.
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch_ros.actions import Node


def _on_node_exit(event: ProcessExited, context: LaunchContext) -> list[Shutdown]:
    # `ros2 launch` exits 0 when a process dies; an exception in an event handler
    # makes it exit 1, so a failed node also fails the container.
    if event.returncode != 0:
        raise RuntimeError(f"tts_node exited with code {event.returncode}")
    return [Shutdown(reason="tts_node exited")]


def generate_launch_description() -> LaunchDescription:
    node = Node(
        package="fortis_tk24_tts",
        executable="tts_node",
        name="tts_node",
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription(
        [
            node,
            RegisterEventHandler(
                OnProcessExit(target_action=node, on_exit=_on_node_exit)
            ),
        ]
    )
