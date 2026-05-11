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
        self.create_subscription(OccupancyGrid,'global_costmap/costmap',self.cb, 10)
        
    def cb(self, msg):
        self.get_logger().info('Cost Map has arrived ...')
       
        w= msg.info.width
        h = msg.info.height

        data = np.array(msg.data, dtype=np.uint8)
        data = data.reshape((h, w))

        img = np.zeros((h, w), dtype=np.uint8)
        img=data

        # img[data == 0] = 255          # free
        # img[(data > 0) & (data < 253)] = 200
        # img[data == 253] = 50         # inscribed
        # img[data == 254] = 0          # lethal
        # img[data == 255] = 128        # unknown

        # Flip Y for correct map orientation
        img = np.flipud(img)

        cv2.imwrite('global_costmap_1.png', img)
        self.get_logger().info('Saved global_costmap.png')


        values, counts = np.unique(img, return_counts=True)
        self.get_logger().info(str(dict(zip(values, counts))))


        rclpy.shutdown()

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