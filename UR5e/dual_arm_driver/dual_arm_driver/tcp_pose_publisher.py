#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException


class TcpPosePublisher(Node):

    def __init__(self):
        super().__init__('dual_arm_pose_publisher')

        # Common reference frame.
        self.declare_parameter('reference_frame', 'chest')

        # Individual overrides.
        # Leave empty to use reference_frame.
        self.declare_parameter('left_reference_frame', '')
        self.declare_parameter('right_reference_frame', '')

        self.declare_parameter('left_tcp_frame', 'left_tcp')
        self.declare_parameter('right_tcp_frame', 'right_tcp')

        self.declare_parameter('publish_rate', 10.0)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_pub = self.create_publisher(
            PoseStamped,
            '/left_arm/pose',
            10
        )

        self.right_pub = self.create_publisher(
            PoseStamped,
            '/right_arm/pose',
            10
        )

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.last_warning = {
            'left': 0.0,
            'right': 0.0,
        }

        self.get_logger().info(
            'Dual-arm TCP pose publisher started'
        )

    def get_reference_frame(self, side):
        override = self.get_parameter(
            f'{side}_reference_frame'
        ).value

        if override:
            return override

        return self.get_parameter('reference_frame').value

    def timer_callback(self):
        self.publish_tcp_pose('left', self.left_pub)
        self.publish_tcp_pose('right', self.right_pub)

    def publish_tcp_pose(self, side, publisher):
        reference_frame = self.get_reference_frame(side)

        tcp_frame = self.get_parameter(
            f'{side}_tcp_frame'
        ).value

        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame,
                tcp_frame,
                Time()
            )

        except TransformException as ex:
            # Avoid filling the terminal at 10 Hz if TF is unavailable.
            now = time.monotonic()

            if now - self.last_warning[side] > 2.0:
                self.get_logger().warning(
                    f'Cannot transform {reference_frame} -> '
                    f'{tcp_frame}: {ex}'
                )
                self.last_warning[side] = now

            return

        msg = PoseStamped()

        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = reference_frame

        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z

        msg.pose.orientation = transform.transform.rotation

        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TcpPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()