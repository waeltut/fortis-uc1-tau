#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclCovarianceMonitor(Node):

    def __init__(self):
        super().__init__('amcl_covariance_monitor')

        self.declare_parameter('window_duration', 30.0)

        self.window_duration = self.get_parameter(
            'window_duration'
        ).value

        self.samples = []

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.timer = self.create_timer(
            self.window_duration,
            self.report_statistics
        )

        self.get_logger().info(
            f'Computing AMCL covariance statistics every '
            f'{self.window_duration} seconds'
        )

    def amcl_callback(self, msg):

        cov = msg.pose.covariance

        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]

        self.samples.append({
            'var_x': var_x,
            'var_y': var_y,
            'var_yaw': var_yaw
        })

    def report_statistics(self):

        if len(self.samples) == 0:
            self.get_logger().warn(
                'No AMCL samples received yet.'
            )
            return

        avg_var_x = sum(s['var_x'] for s in self.samples) / len(self.samples)
        avg_var_y = sum(s['var_y'] for s in self.samples) / len(self.samples)
        avg_var_yaw = sum(s['var_yaw'] for s in self.samples) / len(self.samples)

        avg_position_variance = (avg_var_x + avg_var_y) / 2.0
        position_trace = avg_var_x + avg_var_y

        self.get_logger().info(
            '\n'
            f'Samples: {len(self.samples)}\n'
            f'Avg X Variance   : {avg_var_x:.6f}\n'
            f'Avg Y Variance   : {avg_var_y:.6f}\n'
            f'Avg Yaw Variance : {avg_var_yaw:.6f}\n'
            f'Avg Pos Variance : {avg_position_variance:.6f}\n'
            f'Position Trace   : {position_trace:.6f}'
        )

        #self.samples.clear()


def main(args=None):
    rclpy.init(args=args)

    node = AmclCovarianceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()