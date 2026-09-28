import asyncio
import math
import os
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from hand_control.revo1_utils import open_modbus_revo1, libstark


HAND_PORTS = {"L": "/dev/hand_L", "R": "/dev/hand_R"}
FINGER_IDS = [
    libstark.FingerId.Thumb,
    libstark.FingerId.ThumbAux,
    libstark.FingerId.Index,
    libstark.FingerId.Middle,
    libstark.FingerId.Ring,
    libstark.FingerId.Pinky,
]
FINGER_NAMES = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]
REVO_MIN_POSITION = 0
REVO_MAX_POSITION = 1000

RECONNECT_INTERVAL = 1.0
IO_TIMEOUT = 3.0
CONNECT_TIMEOUT = 10.0


class RevoHandNode(Node):
    def __init__(self):
        super().__init__("revo_hand_node")
        self.declare_parameter("hand", "R")
        self.hand = self.get_parameter("hand").value.upper()
        if self.hand not in HAND_PORTS:
            raise ValueError(f"Invalid hand '{self.hand}'. Expected L or R")

        self.port = HAND_PORTS[self.hand]
        self.topic = f"/hand_{self.hand}/finger_positions"
        self.client = None
        self.slave_id = None

        self._lock = threading.Lock()
        self._connected = False
        self._pending_positions = None
        self._waiting_logged = False
        self._stop = threading.Event()

        self.subscription = self.create_subscription(
            Float32MultiArray, self.topic, self.joint_callback, 1
        )
        self.get_logger().info(f"Listening on {self.topic}; order: {FINGER_NAMES}")

        # This single worker owns ALL Modbus calls, including open and close.
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

    def joint_callback(self, msg):
        # Do not retain commands received while disconnected.
        with self._lock:
            if not self._connected or self._stop.is_set():
                return
            if len(msg.data) != len(FINGER_IDS):
                self.get_logger().warn(
                    f"Expected {len(FINGER_IDS)} values, received {len(msg.data)}"
                )
                return
            if not all(math.isfinite(value) for value in msg.data):
                self.get_logger().warn("Received NaN or infinite finger position")
                return
            # Keep only the newest command; never build an unbounded queue.
            self._pending_positions = [
                self._normalized_to_revo(value) for value in msg.data
            ]

    @staticmethod
    def _normalized_to_revo(value):
        value = max(0.0, min(1.0, value))
        return round(
            REVO_MIN_POSITION + value * (REVO_MAX_POSITION - REVO_MIN_POSITION)
        )

    def _set_waiting(self, reason):
        with self._lock:
            self._connected = False
            self._pending_positions = None
        if not self._waiting_logged:
            self.get_logger().warn(
                f"hand_{self.hand} is not connected ({self.port}): {reason}. "
                "Waiting for connection..."
            )
            self._waiting_logged = True

    def _close_device(self):
        client = self.client
        self.client = None
        self.slave_id = None
        if client is not None:
            try:
                libstark.modbus_close(client)
            except Exception as exc:
                # Avoid repeated warnings during reconnect attempts.
                self.get_logger().debug(f"Error closing Modbus: {exc}")

    async def _pause(self, seconds):
        loop = asyncio.get_running_loop()
        deadline = loop.time() + seconds
        while not self._stop.is_set():
            remaining = deadline - loop.time()
            if remaining <= 0:
                return
            await asyncio.sleep(min(0.05, remaining))

    async def _connect(self):
        self.client, self.slave_id = await asyncio.wait_for(
            open_modbus_revo1(port_name=self.port), timeout=CONNECT_TIMEOUT
        )
        if self.client is None or self.slave_id is None:
            raise ConnectionError("No Modbus device returned")
        info = await self._check_device()
        if self._stop.is_set():
            return
        with self._lock:
            self._pending_positions = None
            self._connected = True
        self._waiting_logged = False
        self.get_logger().info(
            f"hand_{self.hand} connected on {self.port}: {info.description}"
        )

    async def _check_device(self):
        info = await asyncio.wait_for(
            self.client.get_device_info(self.slave_id), timeout=IO_TIMEOUT
        )
        if info is None or not hasattr(info, "description"):
            raise ConnectionError("No valid response from hand")
        return info

    async def _serve(self):
        while not self._stop.is_set():
            # exists() also returns False for a broken /dev/hand_* symlink.
            if not os.path.exists(self.port):
                raise ConnectionError("Device path disappeared")

            with self._lock:
                positions = self._pending_positions
                self._pending_positions = None
            if positions is not None:
                for finger_id, position in zip(FINGER_IDS, positions):
                    if self._stop.is_set():
                        return
                    await asyncio.wait_for(
                        self.client.set_finger_position(
                            self.slave_id, finger_id, position
                        ),
                        timeout=IO_TIMEOUT,
                    )
            await self._pause(0.01)

    async def _connection_worker(self):
        try:
            while not self._stop.is_set():
                if not os.path.exists(self.port):
                    self._set_waiting("Device path does not exist")
                    await self._pause(RECONNECT_INTERVAL)
                    continue
                try:
                    await self._connect()
                    await self._serve()
                except Exception as exc:
                    self._set_waiting(f"{type(exc).__name__}: {exc}")
                finally:
                    with self._lock:
                        self._connected = False
                        self._pending_positions = None
                    self._close_device()
                await self._pause(RECONNECT_INTERVAL)
        finally:
            self._close_device()

    def _run_worker(self):
        asyncio.run(self._connection_worker())

    def destroy_node(self):
        self._stop.set()
        with self._lock:
            self._connected = False
            self._pending_positions = None
        # Let the owner thread finish its current call and close the device.
        self._worker.join()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RevoHandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
