"""Relay 2D Lichtblick PoseStamped clicks to a configurable constant Z height.

The input's frame, timestamp, XY position and full orientation are preserved.
This executable is independent of mur_test's motion/test nodes.
"""

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


def lifted_pose(msg: PoseStamped, height_m: float) -> PoseStamped:
    """Return an independent PoseStamped with its position.z replaced."""
    result = deepcopy(msg)
    result.pose.position.z = height_m
    return result


class PoseHeightRelay(Node):
    def __init__(self) -> None:
        super().__init__('pose_height_relay')
        self.declare_parameter('input_topic', '/goal_pose_2d')
        self.declare_parameter('output_topic', '/base_selection/target')
        self.declare_parameter('height_m', 1.3)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._height_m = float(self.get_parameter('height_m').value)

        if not math.isfinite(self._height_m):
            raise ValueError('height_m must be a finite number')
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must differ to prevent a relay loop')

        self._publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self._subscription = self.create_subscription(
            PoseStamped, input_topic, self._on_pose, 10
        )
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic}; target z={self._height_m:.3f} m'
        )

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name in ('input_topic', 'output_topic'):
                return SetParametersResult(
                    successful=False,
                    reason='Topic parameters are fixed after startup. Restart the node to change them.',
                )
            if parameter.name == 'height_m':
                value = parameter.value
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    return SetParametersResult(
                        successful=False, reason='height_m must be a finite number'
                    )
        for parameter in parameters:
            if parameter.name == 'height_m':
                self._height_m = float(parameter.value)
                self.get_logger().info(f'New target height: {self._height_m:.3f} m')
        return SetParametersResult(successful=True)

    def _on_pose(self, msg: PoseStamped) -> None:
        result = lifted_pose(msg, self._height_m)
        self._publisher.publish(result)
        self.get_logger().info(
            f'Published target in {result.header.frame_id or "<empty frame>"}: '
            f'x={result.pose.position.x:.3f}, '
            f'y={result.pose.position.y:.3f}, '
            f'z={result.pose.position.z:.3f} m'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PoseHeightRelay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
