import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import numpy as np
import cv2
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

class CostmapToPNG(Node):
    
    def __init__(self):
        super().__init__('deb_node')
        self.create_subscription(OccupancyGrid,'/clicked_arrow',self.cb, 10)
        
    def cb(self, msg):
        self.get_logger().info('Cost Map has arrived ...')
        self.get_logger().info(msg)      


def main(args=None):
    rclpy.init(args=args)
    node = CostmapToPNG()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutdown called")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()