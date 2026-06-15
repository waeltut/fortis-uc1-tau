import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from fortis_interfaces.msg import Capability

class CameraHealth(Node):

    def __init__(self):
        super().__init__('camera_health')

        self.last_frame_time = time.monotonic()
        self.frame_timeout = 1.0   # seconds (adjust to FPS)
        self.subscription = self.create_subscription(Image,'/image_raw',self.image_cb,10)
        self.timer = self.create_timer(5.0, self.health_check)

        self.capability_pub = self.create_publisher(Capability, 'capability/vision', 10)
        self.capability_informed = False

    def image_cb(self, msg):
        # Minimal sanity check
        
        if msg.height > 0 and msg.width > 0:
            self.last_frame_time = time.monotonic()

    def health_check(self):
        now = time.monotonic()

        if now - self.last_frame_time > self.frame_timeout:
            msg = Capability()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.capability_name = "Vision"
            msg.capability_description = "Camera is not streaming video signal"
            msg.capability_devices = ["Camera"]
            msg.capability_health = "NOT_STREAMING"
            self.capability_pub.publish(msg)
            self.capability_informed = False

        elif not self.capability_informed:
            msg = Capability()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.capability_name = "Vision"
            msg.capability_description = "Camera is streaming video signal"
            msg.capability_devices = ["Camera"]
            msg.capability_health = "STREAMING"
            self.capability_pub.publish(msg)
            self.capability_informed = True

def main(args=None):
    rclpy.init(args=args)
    node = CameraHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
