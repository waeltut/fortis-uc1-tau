"""Apply a configured payload to each UR arm after External Control starts.

ROS 2 Humble compatible. Never sends trajectories or restarts robot programs.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from ur_msgs.srv import SetPayload


class PayloadInitializer(Node):
    def __init__(self):
        super().__init__('payload_initializer')
        self.arms = ('left', 'right')
        self.transition_time = float(self.declare_parameter('transition_time', 0.0).value)
        self.startup_delay = float(self.declare_parameter('startup_delay', 1.0).value)
        self.retry_interval = float(self.declare_parameter('retry_interval', 5.0).value)

        if self.transition_time < 0.0 or self.startup_delay < 0.0 or self.retry_interval <= 0.0:
            raise ValueError('transition_time and startup_delay must be >= 0; retry_interval > 0')

        self.specs = {}
        self.running = {arm: False for arm in self.arms}
        self.started_at = {arm: None for arm in self.arms}
        self.applied = {arm: False for arm in self.arms}
        self.futures = {arm: None for arm in self.arms}
        self.last_attempt = {arm: -1e20 for arm in self.arms}
        self.payload_clients = {}
        self.payload_subscriptions = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        for arm in self.arms:
            enabled = bool(self.declare_parameter(f'{arm}.enabled', True).value)
            mass = float(self.declare_parameter(f'{arm}.mass', 1.0).value)
            cog = list(self.declare_parameter(f'{arm}.cog', [0.0, 0.0, 0.0]).value)
            inertia = list(self.declare_parameter(f'{arm}.inertia', [0.0] * 6).value)

            if mass <= 0 or not math.isfinite(mass):
                raise ValueError(f'{arm}.mass must be a finite positive number')
            if len(cog) != 3 or len(inertia) != 6 or not all(
                math.isfinite(float(v)) for v in cog + inertia
            ):
                raise ValueError(f'{arm} needs a finite 3-element cog and 6-element inertia')

            self.specs[arm] = (enabled, mass, cog, inertia)
            self.payload_clients[arm] = self.create_client(
                SetPayload, f'/{arm}_io_and_status_controller/set_payload'
            )
            self.payload_subscriptions.append(self.create_subscription(
                Bool,
                f'/{arm}_io_and_status_controller/robot_program_running',
                lambda msg, side=arm: self._on_program(side, msg),
                qos,
            ))
            self.get_logger().info(
                f'{arm}: enabled={enabled}, mass={mass:.3f} kg, cog={cog} m; '
                'waiting for External Control'
            )

        self.payload_timer = self.create_timer(0.5, self._tick)

    def _on_program(self, arm, msg):
        new_state = bool(msg.data)
        if new_state == self.running[arm]:
            return
        self.running[arm] = new_state
        if new_state:
            self.started_at[arm] = time.monotonic()
            self.applied[arm] = False
            self.get_logger().info(f'{arm}: External Control running; scheduling payload')
        else:
            self.started_at[arm] = None
            self.applied[arm] = False
            self.get_logger().warn(f'{arm}: External Control stopped; payload will be reapplied on restart')

    def _tick(self):
        now = time.monotonic()
        for arm in self.arms:
            enabled, mass, cog, inertia = self.specs[arm]
            if not enabled or not self.running[arm] or self.applied[arm]:
                continue
            if self.futures[arm] is not None:
                continue
            if now - self.started_at[arm] < self.startup_delay:
                continue
            if now - self.last_attempt[arm] < self.retry_interval:
                continue
            self.last_attempt[arm] = now
            if not self.payload_clients[arm].service_is_ready():
                self.get_logger().warn(f'{arm}: set_payload service not ready; retrying')
                continue

            request = SetPayload.Request()
            request.mass = mass
            request.center_of_gravity.x = float(cog[0])
            request.center_of_gravity.y = float(cog[1])
            request.center_of_gravity.z = float(cog[2])
            for attr, value in zip(('ixx','iyy','izz','ixy','ixz','iyz'), inertia):
                setattr(request, attr, float(value))
            seconds = int(self.transition_time)
            nanoseconds = int(round((self.transition_time - seconds) * 1_000_000_000))
            if nanoseconds == 1_000_000_000:
                seconds, nanoseconds = seconds + 1, 0
            request.transition_time.sec = seconds
            request.transition_time.nanosec = nanoseconds

            self.get_logger().info(f'{arm}: applying {mass:.3f} kg payload')
            future = self.payload_clients[arm].call_async(request)
            self.futures[arm] = future
            future.add_done_callback(lambda fut, side=arm: self._on_result(side, fut))

    def _on_result(self, arm, future):
        self.futures[arm] = None
        try:
            response = future.result()
            ok = response is not None and bool(response.success)
        except Exception as exc:
            self.get_logger().error(f'{arm}: payload service failed: {exc}')
            return
        if ok and self.running[arm]:
            self.applied[arm] = True
            self.get_logger().info(f'{arm}: payload set successfully')
        elif not ok:
            self.get_logger().error(f'{arm}: payload rejected; retrying while program is running')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadInitializer()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
