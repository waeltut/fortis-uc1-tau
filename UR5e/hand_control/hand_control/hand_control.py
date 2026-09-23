import asyncio
import threading
import rclpy
from rclpy.node import Node

from hand_control.revo1_utils import open_modbus_revo1, libstark

LEFT_PREFIX = "hand_L"
RIGHT_PREFIX = "hand_R"
SUFFIX = "finger_joint_angles"

HAND_PORTS = {
    "L": "/dev/hand_L",
    "R": "/dev/hand_R",
}

from interfaces_pkg.msg import FingerJointAngles

GLOVE_JOINT_MAX = 180


class RevoHandNode(Node):

    def __init__(self):
        super().__init__('revo_hand_node')

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

        self.declare_parameter("hand", "R")
        hand = self.get_parameter("hand").value

        if hand not in HAND_PORTS:
            raise ValueError(f"Invalid hand '{hand}', expected one of {list(HAND_PORTS)}")

        self.port = HAND_PORTS[hand]
        prefix = LEFT_PREFIX if hand == "L" else RIGHT_PREFIX
        topic = f"/{prefix}/{SUFFIX}"

        self.get_logger().info(f"Hand '{hand}' on port {self.port}")
        self.get_logger().info(f"Listening to {topic}")

        self.subscription = self.create_subscription(
            FingerJointAngles,
            topic,
            self.joint_callback,
            10
        )

        self.client = None
        self.slave_id = None

        future = asyncio.run_coroutine_threadsafe(self.init_device(), self._loop)
        future.result()

    async def init_device(self):
        self.client, self.slave_id = await open_modbus_revo1(port_name=self.port)
        device_info = await self.client.get_device_info(self.slave_id)
        self.get_logger().info(f"Connected: {device_info.description}")


    def joint_callback(self, msg):
        if self.client is None:
            self.get_logger().warn("Client is None!")
            return

        glove_dict = {
            field.lstrip('_'): getattr(msg, field)
            for field in msg.__slots__
            if field != "timestamp"
        }

        q = process_glove_data(glove_dict)

        async def send_all():
            try:
                finger_ids = [
                    libstark.FingerId.Thumb,
                    libstark.FingerId.ThumbAux,
                    libstark.FingerId.Index,
                    libstark.FingerId.Middle,
                    libstark.FingerId.Ring,
                    libstark.FingerId.Pinky,
                ]
                for finger_id, position in zip(finger_ids, q):
                    await self.client.set_finger_position(self.slave_id, finger_id, position)
            except Exception as e:
                self.get_logger().error(f"send_all error: {type(e).__name__}: {e}")

        future = asyncio.run_coroutine_threadsafe(send_all(), self._loop)
        try:
            future.result(timeout=2.0)
        except Exception as e:
            self.get_logger().error(f"Future error: {type(e).__name__}: {e}")

    def destroy_node(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join()
        super().destroy_node()


def process_glove_data(glove_data):
    finger_joints = [
        # Thumb: cmc_stretch + mcp_stretch + ip_stretch
        ((55-glove_data.get("thumb_cmc_stretch", 0)) + glove_data.get("thumb_mcp_stretch", 0) + glove_data.get("thumb_ip_stretch", 0))*1.5, 
        # ThumbAux
        glove_data.get("thumb_cmc_spread",0) * 3.5,
        # Index
        glove_data.get("index_mcp_stretch", 0) + glove_data.get("index_pip_stretch", 0) + glove_data.get("index_dip_stretch", 0) -40,
        # Middle
        glove_data.get("middle_mcp_stretch", 0) + glove_data.get("middle_pip_stretch", 0) + glove_data.get("middle_dip_stretch", 0) -40,
        # Ring
        glove_data.get("ring_mcp_stretch", 0) + glove_data.get("ring_pip_stretch", 0) + glove_data.get("ring_dip_stretch", 0) -40,
        # Pinky
        glove_data.get("pinky_mcp_stretch", 0) + glove_data.get("pinky_pip_stretch", 0) + glove_data.get("pinky_dip_stretch", 0) -40,
    ]

    q = [
        int(max(0, min(1000, round(val * 1000.0 / GLOVE_JOINT_MAX))))
        for val in finger_joints
    ]

    return q


# Usage:
# ros2 run hand_control hand_control
# ros2 run hand_control hand_control --ros-args -p hand:=L

def main(args=None):
    rclpy.init(args=args)
    node = RevoHandNode()
    rclpy.spin(node)
    libstark.modbus_close(node.client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()