import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2



class Display(Node):
    def __init__(self):
        # Initialization
        super().__init__('map_display')

        # Ros
        self.get_logger().info("Node started. Listening to maps...")
        self.mic_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 1)


    def map_callback(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Convert to image: unknown(-1)=128 gray, free(0)=255 white, occupied(100)=0 black
        img = np.zeros_like(data, dtype=np.uint8)
        img[data == -1] = 128
        img[data == 0] = 255
        img[data == 100] = 0

        # Show the image
        #self.get_logger().info("Got map")
        cv2.imshow("Occupancy Grid", img)
        cv2.waitKey(1)  # needed to refresh OpenCV window


def main(args=None):
    rclpy.init(args=args)
    display = Display()
    try:
        rclpy.spin(display)
    except KeyboardInterrupt:
        display.get_logger().warn("Shutdown called")
    except Exception as e:
        display.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()