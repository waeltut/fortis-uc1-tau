#!/usr/bin/env python3
"""Isolated test endpoints; never commands robot hardware."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BridgeDemo(Node):
    def __init__(self):
        super().__init__('toolkit_bridge_demo')
        self.counter = 0
        self.echo = self.create_publisher(String, '/bridge_demo/echo', 10)
        self.status = self.create_publisher(String, '/bridge_demo/status', 10)
        self.input = self.create_subscription(String, '/bridge_demo/input', self.echo.publish, 10)
        self.service = self.create_service(Trigger, '/bridge_demo/ping', self.ping)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info('Demo ready: input -> echo, status (1 Hz), ping service')

    def tick(self):
        self.counter += 1
        self.status.publish(String(data=f'toolkit bridge demo: {self.counter}'))

    def ping(self, request, response):
        response.success = True
        response.message = 'pong'
        return response


def main():
    rclpy.init()
    node = BridgeDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
