import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import math
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import numpy as np


class ScannersMerger(Node):
    def __init__(self):
        super().__init__('scanners_merger')

        self.get_logger().info('Scanners merger node has started ...!!')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scanners_tf = {}

        # Topics
        self.create_subscription(LaserScan, "/f_scan", self.lidar_callback, 1)
        self.create_subscription(LaserScan, "/b_scan", self.lidar_callback, 1)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 1)

        # Timers
        self.init_tf_timer = self.create_timer(0.5, self.init_tf)
        self.publish_timer = self.create_timer(0.08, self.scan_publisher)

        # Scan parameters
        self.angle_increment = 0.00872664712369442
        self.scan_angles = np.arange(0.0, 2.0 * np.pi, self.angle_increment)
        self.scan_ranges = [float('inf')] * len(self.scan_angles)
        self.latest_scans = {
            "front_laser_link": None,
            "back_laser_link": None,
}


    # ---------------- TF ----------------

    def init_tf(self):
        """Acquire and cache static TFs once"""
        frames = ["front_laser_link", "back_laser_link"]

        for frame in frames:
            if frame in self.scanners_tf:
                continue

            if not self.tf_buffer.can_transform(
                "base_footprint",
                frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            ):
                self.get_logger().warn(f"Waiting for TF: base_footprint <- {frame}")
                return

            t = self.tf_buffer.lookup_transform(
                "base_footprint",
                frame,
                rclpy.time.Time()
            )
            self.scanners_tf[frame] = t
            self.get_logger().info(f"TF acquired for {frame}")

        # Stop this timer once all TFs are ready
        if len(self.scanners_tf) == 2:
            self.init_tf_timer.cancel()
            self.get_logger().info("All scanner TFs initialized")

    # ---------------- Callbacks ----------------

    
    def lidar_callback(self, msg: LaserScan):
        frame = msg.header.frame_id

        if frame not in self.latest_scans:
            return

        self.latest_scans[frame] = msg


   

    def scan_publisher(self):
        if len(self.scanners_tf) < 2:
            return

        # Reset output scan
        self.scan_ranges = [float('inf')] * len(self.scan_ranges)

        for frame, scan in self.latest_scans.items():
            if scan is None:
                continue

            tf = self.scanners_tf[frame]

            # Extract transform
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation

            # Yaw only (2D lidar)
            yaw = math.atan2(
                2.0 * (q.w * q.z),
                1.0 - 2.0 * (q.z * q.z)
            )

            angle = scan.angle_min

            for r in scan.ranges:
                if (
                    not math.isfinite(r)
                    or r < scan.range_min
                    or r > scan.range_max
                ):
                    angle += scan.angle_increment
                    continue

                # 1) Polar → Cartesian (scanner frame)
                x_s = r * math.cos(angle)
                y_s = r * math.sin(angle)

                # 2) Rotate + translate → base_footprint
                x_b = math.cos(yaw) * x_s - math.sin(yaw) * y_s + tx
                y_b = math.sin(yaw) * x_s + math.cos(yaw) * y_s + ty

                # 3) Cartesian → Polar (base frame)
                r_b = math.hypot(x_b, y_b)
                a_b = math.atan2(y_b, x_b)

                if a_b < 0.0:
                    a_b += 2.0 * math.pi

                idx = int(a_b / self.angle_increment)

                if 0 <= idx < len(self.scan_ranges):
                    self.scan_ranges[idx] = min(self.scan_ranges[idx], r_b)

                angle += scan.angle_increment

        # Publish merged scan
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = self.angle_increment

        msg.range_min = 0.05
        msg.range_max = 29.0

        msg.ranges = self.scan_ranges
        msg.intensities = []

        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / len(self.scan_ranges)

        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScannersMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutdown called")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()