
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class PiperTTSNode(Node):
    def __init__(self):
        super().__init__('piper_tts_node')

        # Parameters
        self.model = os.getenv("PIPER_MODEL", "/opt/piper/models/en_US-amy-medium.onnx")
        self.device = "plughw:2,0"

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/speaker_driver/speak',
            self.callback,
            10
        )

        self.get_logger().info("Speaking node is ready!!")

    def callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Speaking: {text}")

        cmd = [
            "bash", "-c",
            f'echo "{text}" | piper --model {self.model} --output-raw | '
            f'aplay -r 22050 -f S16_LE -c 1 -D {self.device}'
        ]

        subprocess.run(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PiperTTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
