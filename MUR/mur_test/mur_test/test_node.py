"""MUR test: Nav2 -> actual base pose -> selected-arm MoveIt commander topic.

MoveIt planning and execution belong to dual_arm_pose_commander, not this node.
The selected arm is taken ONLY from mur_base_selector's matched JSON result.
No IK, collision-matrix manipulation, or MoveGroup action is performed here.
"""
import copy
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class MurTest(Node):
    def __init__(self):
        super().__init__('mur_test')
        self.declare_parameter('target_topic', '/base_selection/target')
        self.declare_parameter('selected_pose_topic', '/base_selection/selected_pose')
        self.declare_parameter('selection_diagnostics_topic', '/base_selection/candidate_diagnostics')
        self.declare_parameter('selection_result_topic', '/base_selection/result')
        self.declare_parameter('navigation_action', '/navigate_to_pose')
        self.declare_parameter('base_goal_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planning_frame', 'mur')
        self.declare_parameter('left_goal_topic', '/left_arm/goal_pose')
        self.declare_parameter('right_goal_topic', '/right_arm/goal_pose')
        self.declare_parameter('navigation_timeout', 180.0)
        self.declare_parameter('interface_timeout', 15.0)
        self.declare_parameter('pose_wait_timeout', 30.0)
        self.declare_parameter('arm_selection_timeout', 4.0)
        self.declare_parameter('commander_wait_timeout', 5.0)
        self.declare_parameter('tf_timeout', 5.0)

        self.lock = threading.Lock()
        self.target = None
        self.selected = None
        self.selected_at = 0.0
        self.selected_metadata = None
        self.target_generation = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, qos)
        self.create_subscription(PoseStamped, self.p('selected_pose_topic'), self.on_selected, qos)
        diagnostic_topic = str(self.p('selection_diagnostics_topic')).strip()
        result_topic = str(self.p('selection_result_topic')).strip()
        for topic in dict.fromkeys((diagnostic_topic, result_topic)):
            if topic:
                self.create_subscription(String, topic, self.on_diagnostics, qos)

        self.nav = ActionClient(self, NavigateToPose, self.p('navigation_action'))
        self.arm_pubs = {
            'left': self.create_publisher(PoseStamped, self.p('left_goal_topic'), qos),
            'right': self.create_publisher(PoseStamped, self.p('right_goal_topic'), qos),
        }
        self.get_logger().info('Ready: selector arm -> Nav2 -> latest map-to-mur TF -> MoveIt commander.')
        self.get_logger().info('Arm dispatch is automatic after successful navigation; no arm confirmation.')

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def valid_pose(pose):
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:]) - 1.0) < 0.02

    @staticmethod
    def same_target(a, b):
        if a.header.frame_id != b.header.frame_id:
            return False
        if not MurTest.valid_pose(a.pose) or not MurTest.valid_pose(b.pose):
            return False
        pa, pb = a.pose.position, b.pose.position
        if math.dist((pa.x, pa.y, pa.z), (pb.x, pb.y, pb.z)) > 1e-6:
            return False
        qa, qb = a.pose.orientation, b.pose.orientation
        dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
        return abs(abs(dot) - 1.0) < 1e-6

    @staticmethod
    def yaw(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def on_target(self, msg):
        with self.lock:
            # The selector periodically republishes the same target: ignore duplicates.
            if self.target is not None and self.same_target(self.target, msg):
                return
            self.target = copy.deepcopy(msg)
            self.selected = None
            self.selected_at = 0.0
            self.selected_metadata = None
            self.target_generation += 1
        self.get_logger().info('TCP target received in %s; waiting for base selection.' % msg.header.frame_id)

    def on_selected(self, msg):
        with self.lock:
            if self.target is None:
                return
            self.selected = copy.deepcopy(msg)
            self.selected_at = time.monotonic()
        self.get_logger().info('Selected base: %s (%.3f, %.3f)' % (
            msg.header.frame_id, msg.pose.position.x, msg.pose.position.y))

    def on_diagnostics(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('valid') is False:
                return
            if data.get('arm') not in ('left', 'right', 'both'):
                return
            coords = data.get('pose')
            if not isinstance(coords, list) or len(coords) < 3:
                return
            if not all(math.isfinite(float(v)) for v in coords[:3]):
                return
        except (TypeError, ValueError, OverflowError):
            return
        with self.lock:
            if self.target is not None:
                self.selected_metadata = (data, time.monotonic())

    def matching_arm(self, selected):
        with self.lock:
            info, selected_at = self.selected_metadata, self.selected_at
        if info is None or selected_at <= 0:
            return None
        data, metadata_at = info
        if abs(metadata_at - selected_at) > 5.0:
            return None
        if data.get('frame_id', selected.header.frame_id) != selected.header.frame_id:
            return None
        x, y, theta = (float(v) for v in data['pose'][:3])
        if math.hypot(x - selected.pose.position.x, y - selected.pose.position.y) > 0.03:
            return None
        diff = theta - self.yaw(selected.pose.orientation)
        if abs(math.atan2(math.sin(diff), math.cos(diff))) > math.radians(5.0):
            return None
        return data['arm']

    def require_selected_arm(self, selected):
        deadline = time.monotonic() + float(self.p('arm_selection_timeout'))
        while rclpy.ok() and time.monotonic() < deadline:
            arm = self.matching_arm(selected)
            if arm is not None:
                if arm == 'both':
                    raise RuntimeError('Selector chose both arms; single-arm dispatcher will not guess.')
                self.get_logger().info('Locked selected arm: %s' % arm.upper())
                return arm
            time.sleep(0.05)
        raise RuntimeError('No matching selector arm metadata for selected base pose; refusing to guess.')

    def wait_future(self, future, timeout, label):
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError('%s timed out after %.1f s' % (label, timeout))
        return future.result()

    def wait_for_selection(self):
        deadline = time.monotonic() + float(self.p('pose_wait_timeout'))
        while rclpy.ok() and time.monotonic() < deadline:
            with self.lock:
                if self.target is not None and self.selected is not None:
                    return (self.target_generation, copy.deepcopy(self.target), copy.deepcopy(self.selected))
            time.sleep(0.1)
        raise TimeoutError('Waiting for target and selected base pose')

    def check_snapshot(self, generation):
        with self.lock:
            if self.target_generation != generation:
                raise RuntimeError('New TCP target received during navigation; stale arm command cancelled')

    def discard(self, generation):
        with self.lock:
            if self.target_generation == generation:
                self.target = None
                self.selected = None
                self.selected_metadata = None
                self.selected_at = 0.0

    def transform_to(self, pose, frame):
        if not pose.header.frame_id or not self.valid_pose(pose.pose):
            raise ValueError('Pose has an invalid frame or quaternion')
        if pose.header.frame_id == frame:
            return copy.deepcopy(pose)
        deadline = time.monotonic() + float(self.p('tf_timeout'))
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                # Latest TF is intentional: recalculate using the actual post-Nav2 base pose.
                tf = self.tf_buffer.lookup_transform(frame, pose.header.frame_id, rclpy.time.Time())
                result = PoseStamped()
                result.header.frame_id = frame
                result.header.stamp = self.get_clock().now().to_msg()
                result.pose = do_transform_pose(pose.pose, tf)
                return result
            except TransformException:
                time.sleep(0.1)
        raise RuntimeError('Missing TF %s <- %s' % (frame, pose.header.frame_id))

    def navigate(self, base_map):
        if not self.nav.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('Nav2 NavigateToPose action unavailable')
        request = NavigateToPose.Goal()
        request.pose = copy.deepcopy(base_map)
        request.pose.header.stamp = self.get_clock().now().to_msg()
        handle = self.wait_future(self.nav.send_goal_async(request), self.p('interface_timeout'), 'Nav2 goal response')
        if handle is None or not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        print('Navigation accepted; waiting for Nav2 to finish...')
        try:
            response = self.wait_future(handle.get_result_async(), self.p('navigation_timeout'), 'Navigation')
        except (TimeoutError, KeyboardInterrupt):
            handle.cancel_goal_async()
            raise
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Nav2 ended with status=%d (arm goal not sent)' % response.status)
        self.get_logger().info('Nav2 succeeded.')

    def log_actual_arrival(self, base_map):
        origin = PoseStamped()
        origin.header.frame_id = str(self.p('base_frame'))
        origin.pose.orientation.w = 1.0
        actual = self.transform_to(origin, str(self.p('base_goal_frame')))
        desired, reached = base_map.pose, actual.pose
        xy = math.hypot(reached.position.x-desired.position.x, reached.position.y-desired.position.y)
        angle = self.yaw(reached.orientation) - self.yaw(desired.orientation)
        angle = abs(math.atan2(math.sin(angle), math.cos(angle)))
        self.get_logger().info('Actual base arrival: XY error=%.1f mm, yaw error=%.2f deg' % (
            xy*1000, math.degrees(angle)))
        # Deliberately do not reject a successful Nav2 arrival based on XY/yaw error.
        # The TCP target is transformed from the actual robot pose below.

    def publish_commander_goal(self, arm, tcp_map):
        pub = self.arm_pubs[arm]
        deadline = time.monotonic() + float(self.p('commander_wait_timeout'))
        while rclpy.ok() and pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.1)
        if pub.get_subscription_count() < 1:
            raise RuntimeError('No MoveIt commander subscriber on %s' % pub.topic_name)
        goal = self.transform_to(tcp_map, str(self.p('planning_frame')))
        goal.header.stamp = self.get_clock().now().to_msg()
        p, q = goal.pose.position, goal.pose.orientation
        self.get_logger().info('Dispatching %s TCP goal to %s in %s: '
                               'xyz=(%.4f, %.4f, %.4f), xyzw=(%.5f, %.5f, %.5f, %.5f)' % (
                                   arm.upper(), pub.topic_name, goal.header.frame_id,
                                   p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        pub.publish(goal)
        self.get_logger().info('Goal published to existing MoveIt commander. '
                               'Commander is responsible for MoveIt planning, configured collision policy, '
                               'and execution; topic delivery does not confirm motion completion.')

    @staticmethod
    def confirm(question):
        return input('\n%s [y/N]: ' % question).strip().lower() in ('y', 'yes')

    def run_console(self):
        print('\n=== mur_test: Nav2 -> selected arm commander goal ===')
        print('Nav confirmation is manual; arm goal publishes automatically after Nav2 success.')
        print('Press Ctrl+C to exit.\n')
        while rclpy.ok():
            generation = None
            try:
                generation, target, base = self.wait_for_selection()
                if not self.valid_pose(target.pose) or not self.valid_pose(base.pose):
                    raise RuntimeError('Invalid target or base quaternion')
                print('\nTCP target: %s (%.3f, %.3f, %.3f)' % (
                    target.header.frame_id, target.pose.position.x,
                    target.pose.position.y, target.pose.position.z))
                print('Selected base: %s (%.3f, %.3f)' % (
                    base.header.frame_id, base.pose.position.x, base.pose.position.y))
                if not self.confirm('Navigate MiR to the selected base pose?'):
                    print('Navigation skipped; waiting for a new target.')
                    self.discard(generation)
                    continue
                self.check_snapshot(generation)
                arm = self.require_selected_arm(base)  # Lock before moving; no opposite-arm fallback.
                # Anchor the requested TCP goal to the map BEFORE driving.
                tcp_map = self.transform_to(target, str(self.p('base_goal_frame')))
                base_map = self.transform_to(base, str(self.p('base_goal_frame')))
                self.check_snapshot(generation)
                self.navigate(base_map)
                self.check_snapshot(generation)
                self.log_actual_arrival(base_map)
                self.check_snapshot(generation)
                self.publish_commander_goal(arm, tcp_map)
                self.discard(generation)
            except (KeyboardInterrupt, EOFError):
                break
            except TimeoutError as exc:
                if 'Waiting for target' not in str(exc):
                    self.get_logger().error('Test stopped: %s' % exc)
                if generation is not None:
                    self.discard(generation)
            except Exception as exc:
                self.get_logger().error('Test stopped: %s' % exc)
                if generation is not None:
                    self.discard(generation)
                print('Re-publish the TCP target to start a new test.')


def main(args=None):
    rclpy.init(args=args)
    node = MurTest()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run_console()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
