import asyncio
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from hand_control.revo1_utils import open_modbus_revo1, libstark


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

HAND_PORTS = {
    "L": "/dev/hand_L",
    "R": "/dev/hand_R",
}

FINGER_IDS = [
    libstark.FingerId.Thumb,
    libstark.FingerId.ThumbAux,
    libstark.FingerId.Index,
    libstark.FingerId.Middle,
    libstark.FingerId.Ring,
    libstark.FingerId.Pinky,
]

FINGER_NAMES = [
    "thumb",
    "thumb_aux",
    "index",
    "middle",
    "ring",
    "pinky",
]

REVO_MIN_POSITION = 0
REVO_MAX_POSITION = 1000


class RevoHandNode(Node):

    def __init__(self):
        super().__init__("revo_hand_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        self.declare_parameter("hand", "R")
        hand = self.get_parameter("hand").value.upper()

        if hand not in HAND_PORTS:
            raise ValueError(
                f"Invalid hand '{hand}'. Expected one of: {list(HAND_PORTS)}"
            )

        self.hand = hand
        self.port = HAND_PORTS[hand]

        # Example:
        # /hand_R/finger_positions
        # /hand_L/finger_positions
        self.topic = f"/hand_{hand}/finger_positions"

        # ------------------------------------------------------------------
        # Asyncio thread for Modbus communication
        # ------------------------------------------------------------------

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
        )
        self._loop_thread.start()

        self.client = None
        self.slave_id = None

        # ------------------------------------------------------------------
        # Connect to hand
        # ------------------------------------------------------------------

        future = asyncio.run_coroutine_threadsafe(
            self._init_device(),
            self._loop,
        )

        try:
            future.result(timeout=10.0)
        except Exception:
            self._shutdown_async_loop()
            raise

        # ------------------------------------------------------------------
        # ROS subscription
        # ------------------------------------------------------------------

        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.topic,
            self.joint_callback,
            10,
        )

        self.get_logger().info(
            f"Revo1 hand '{self.hand}' connected on {self.port}"
        )
        self.get_logger().info(
            f"Listening for normalized finger positions on {self.topic}"
        )
        self.get_logger().info(
            f"Finger order: {FINGER_NAMES}"
        )

    # ----------------------------------------------------------------------
    # Async setup
    # ----------------------------------------------------------------------

    def _run_async_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _init_device(self):
        self.client, self.slave_id = await open_modbus_revo1(
            port_name=self.port
        )

        device_info = await self.client.get_device_info(self.slave_id)

        self.get_logger().info(
            f"Connected to device: {device_info.description}"
        )

    # ----------------------------------------------------------------------
    # ROS callback
    # ----------------------------------------------------------------------

    def joint_callback(self, msg: Float32MultiArray):
        if self.client is None:
            self.get_logger().warn("Hand is not connected")
            return

        if len(msg.data) != len(FINGER_IDS):
            self.get_logger().warn(
                f"Expected {len(FINGER_IDS)} values, "
                f"received {len(msg.data)}"
            )
            return

        if not all(math.isfinite(value) for value in msg.data):
            self.get_logger().warn(
                "Received NaN or infinite finger position"
            )
            return

        positions = [
            self._normalized_to_revo(value)
            for value in msg.data
        ]

        asyncio.run_coroutine_threadsafe(
            self._send_positions(positions),
            self._loop,
        )

    # ----------------------------------------------------------------------
    # Position conversion
    # ----------------------------------------------------------------------

    @staticmethod
    def _normalized_to_revo(value: float) -> int:
        """
        Convert normalized finger position [0.0, 1.0]
        to Revo1 position [0, 1000].

        Values outside the normalized range are clamped.
        """

        value = max(0.0, min(1.0, value))

        return round(
            REVO_MIN_POSITION
            + value * (REVO_MAX_POSITION - REVO_MIN_POSITION)
        )

    # ----------------------------------------------------------------------
    # Hand communication
    # ----------------------------------------------------------------------

    async def _send_positions(self, positions):
        try:
            for finger_id, position in zip(FINGER_IDS, positions):
                await self.client.set_finger_position(
                    self.slave_id,
                    finger_id,
                    position,
                )

        except Exception as exc:
            self.get_logger().error(
                f"Failed to send finger positions: "
                f"{type(exc).__name__}: {exc}"
            )

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------

    def _shutdown_async_loop(self):
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        if self._loop_thread.is_alive():
            self._loop_thread.join()

    def destroy_node(self):
        if self.client is not None:
            try:
                libstark.modbus_close(self.client)
            except Exception as exc:
                self.get_logger().warn(
                    f"Error while closing Modbus connection: {exc}"
                )

        self._shutdown_async_loop()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RevoHandNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()