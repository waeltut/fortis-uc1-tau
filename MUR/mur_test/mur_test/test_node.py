"""Collect a workspace, select a shared base via service, navigate and visit points.

MoveIt planning and execution belong to dual_arm_commander.
"""
import copy
import math
import threading
import time
import select
import sys
from mur_base_selector_interfaces.srv import SelectBase

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from dual_arm_interfaces.action import MoveArms
from .action_runner import run_action
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class MurTest(Node):
    def __init__(self):
        super().__init__('mur_test')
        self.declare_parameter('target_topic', '/mur_test/point')
        self.declare_parameter('selection_service', '/base_selection/select')
        self.declare_parameter('selection_timeout', 60.0)
        self.declare_parameter('navigation_action', '/navigate_to_pose')
        self.declare_parameter('base_goal_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('planning_frame', 'mur')
        self.declare_parameter('commander_action', '/arms/move')
        self.declare_parameter('arm_timeout', 180.0)
        self.declare_parameter('cancel_timeout', 20.0)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('navigation_timeout', 180.0)
        self.declare_parameter('interface_timeout', 15.0)
        self.declare_parameter('commander_wait_timeout', 5.0)
        self.declare_parameter('tf_timeout', 5.0)

        self.motion_fault = None
        for name in ('navigation_timeout', 'interface_timeout', 'selection_timeout', 'commander_wait_timeout', 'tf_timeout',
                     'arm_timeout', 'cancel_timeout'):
            if not math.isfinite(float(self.p(name))) or float(self.p(name)) <= 0:
                raise ValueError(name + ' must be finite and positive')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not 0.0 <= float(self.p(name)) <= 1.0:
                raise ValueError(name + ' must be in [0, 1]')
        self.lock = threading.Lock()
        self.target_generation = 0

        self.pending_points = []
        self.collecting = True
        self.selector = self.create_client(SelectBase, self.p('selection_service'))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, qos)
        self.nav = ActionClient(self, NavigateToPose, self.p('navigation_action'))
        self.commander = ActionClient(self, MoveArms, self.p('commander_action'))
        self.get_logger().info('Ready: place points on /mur_test/point, then confirm the workspace in this terminal.')

    def p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def valid_pose(pose):
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:]) - 1.0) < 0.02

    @staticmethod
    def yaw(q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def on_target(self, msg):
        if not msg.header.frame_id or not self.valid_pose(msg.pose):
            self.get_logger().error('Point rejected: frame and unit quaternion required')
            return
        with self.lock:
            if not self.collecting:
                self.get_logger().warning('Workspace is frozen; point ignored. Use new to collect another workspace.')
                return
            self.pending_points.append(copy.deepcopy(msg))
        self.get_logger().info('New point received; queued for arm assignment in the terminal.')

    def check_snapshot(self, generation):
        with self.lock:
            if self.target_generation != generation:
                raise RuntimeError('New TCP target received during navigation; stale arm command cancelled')

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

    def fault(self, message):
        self.motion_fault = message
        self.get_logger().error(message)

    def run_motion(self, client, request, timeout, generation, feedback=None):
        def stopped():
            if not rclpy.ok():
                return 'ROS shutdown'
            with self.lock:
                if generation != self.target_generation:
                    return 'New TCP target received'
            return None
        self.check_snapshot(generation)
        return run_action(client, request, float(self.p('interface_timeout')),
                          float(timeout), float(self.p('cancel_timeout')),
                          stopped, self.fault, feedback)

    def navigate(self, base_map, generation):
        if not self.nav.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('Nav2 NavigateToPose action unavailable')
        request = NavigateToPose.Goal()
        request.pose = copy.deepcopy(base_map)
        request.pose.header.stamp = self.get_clock().now().to_msg()
        response = self.run_motion(self.nav, request, self.p('navigation_timeout'), generation)
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

    def execute_commander_goal(self, arm, tcp_map, generation):
        if not self.commander.wait_for_server(timeout_sec=float(self.p('commander_wait_timeout'))):
            raise RuntimeError('MoveArms action unavailable: %s' % self.p('commander_action'))
        pose = self.transform_to(tcp_map, str(self.p('planning_frame')))
        # MoveArms explicitly rejects nonzero timestamps on either target.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        request = MoveArms.Goal()
        request.goal_type = 'explicit'
        request.mode = arm
        request.target_type = 'pose'
        setattr(request, arm + '_target', pose)
        request.velocity_scaling = float(self.p('velocity_scaling'))
        request.acceleration_scaling = float(self.p('acceleration_scaling'))
        request.plan_only = bool(self.p('plan_only'))
        self.get_logger().info('Dispatching %s TCP target in %s to %s (plan_only=%s)' % (
            arm.upper(), pose.header.frame_id, self.p('commander_action'), request.plan_only))
        response = self.run_motion(self.commander, request, self.p('arm_timeout'),
                                   generation, self.on_arm_feedback)
        result = response.result
        detail = ('status=%d, error=%d, MoveIt=%d: %s; executed=%s, '
                  'left_completed=%s, right_completed=%s' % (
                      response.status, result.error_code, result.moveit_error_code,
                      result.message, result.executed, result.left_completed, result.right_completed))
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != MoveArms.Result.OK:
            raise RuntimeError('Arm action failed: ' + detail)
        if not request.plan_only and (not result.executed or not getattr(result, arm + '_completed')):
            raise RuntimeError('Arm action did not confirm selected-arm execution: ' + detail)
        self.get_logger().info(('Arm plan ready: ' if request.plan_only else 'Arm motion completed: ') + detail)

    def on_arm_feedback(self, message):
        feedback = message.feedback
        self.get_logger().info('Commander %s [%s]: %s' % (
            feedback.stage, feedback.active_arms, feedback.detail))

    @staticmethod
    def confirm(question):
        return input('\n%s [y/N]: ' % question).strip().lower() in ('y', 'yes')

    def collect_points(self):
        points, arms = [], []
        with self.lock:
            self.collecting = True
        print('Place points. Each point will ask for any/left/right/both (Enter = any).')
        print('Type done to confirm, clear to reset, or quit to exit.', flush=True)
        while rclpy.ok():
            with self.lock:
                pending = self.pending_points.pop(0) if self.pending_points else None
            if pending is not None:
                try:
                    anchored = self.transform_to(pending, str(self.p('base_goal_frame')))
                except Exception as exc:
                    print('Point rejected: %s' % exc)
                    continue
                number = len(points) + 1
                p = anchored.pose.position
                print('P%d added: (%.3f, %.3f, %.3f) in %s' % (number,p.x,p.y,p.z,anchored.header.frame_id))
                while True:
                    arm = input('P%d arm [any/left/right/both; default any]: ' % number).strip().lower() or 'any'
                    if arm in ('any','left','right','both'): break
                    print('Choose any, left, right or both.')
                points.append(anchored); arms.append(arm)
                continue
            if not select.select([sys.stdin], [], [], 0.2)[0]: continue
            line = sys.stdin.readline()
            if not line: raise EOFError()
            command = line.strip().lower()
            if command in ('quit','q'): return None
            if command == 'clear':
                points.clear(); arms.clear()
                with self.lock: self.pending_points.clear()
                print('Points cleared.'); continue
            if command == 'done':
                with self.lock:
                    if self.pending_points:
                        print('Assign the queued points first.'); continue
                    if not points:
                        print('Add at least one point.'); continue
                    self.collecting = False
                    self.target_generation += 1
                return points, arms

    def select_workspace(self, points, arms):
        if not self.selector.wait_for_service(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('Base selection service unavailable')
        request = SelectBase.Request()
        request.targets = points
        request.arms = arms
        future = self.selector.call_async(request)
        deadline = time.monotonic() + float(self.p('selection_timeout'))
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                raise TimeoutError('Selection timed out; selector may still be busy. No motion dispatched.')
            time.sleep(0.05)
        if not future.done(): raise RuntimeError('ROS shutdown')
        result = future.result()
        if not result.success: raise RuntimeError(result.message)
        if len(result.point_arms) != len(points):
            raise RuntimeError('Selector returned incomplete point arm assignments')
        for required, chosen in zip(arms, result.point_arms):
            if chosen not in ('left','right','both') or (required != 'any' and chosen != required):
                raise RuntimeError('Selector returned an incompatible arm assignment')
        return result

    def home(self, generation):
        if not self.commander.wait_for_server(timeout_sec=float(self.p('commander_wait_timeout'))):
            raise RuntimeError('MoveArms action unavailable')
        request = MoveArms.Goal()
        request.goal_type = 'preset'
        request.preset = 'home'
        request.plan_only = bool(self.p('plan_only'))
        result = self.run_motion(self.commander, request, self.p('arm_timeout'), generation, self.on_arm_feedback)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != MoveArms.Result.OK:
            raise RuntimeError('Home failed: ' + result.result.message)
        if not request.plan_only and not (result.result.executed and result.result.left_completed and result.result.right_completed):
            raise RuntimeError('Home did not confirm completion of both arms')
        print('Home completed.' if not request.plan_only else 'Home plan ready.')

    def run_console(self):
        print('=== MUR multi-point workspace ===')
        try:
            while rclpy.ok() and self.motion_fault is None:
                collected = self.collect_points()
                if collected is None: break
                points, arms = collected
                generation = self.target_generation
                try:
                    result = self.select_workspace(points, arms)
                    base = self.transform_to(result.navigation_goal, str(self.p('base_goal_frame')))
                    expiry = time.monotonic() + result.valid_for_seconds
                    print('Shared base: (%.3f, %.3f). Assignments: %s' % (
                        base.pose.position.x, base.pose.position.y,
                        ', '.join('P%d=%s' % (i+1,a) for i,a in enumerate(result.point_arms))))
                    if not self.confirm('Navigate MiR to this destination?'):
                        continue
                    if time.monotonic() >= expiry:
                        print('Selection expired. Recalculating; confirm the fresh destination.')
                        result = self.select_workspace(points, arms)
                        base = self.transform_to(result.navigation_goal, str(self.p('base_goal_frame')))
                        expiry = time.monotonic() + result.valid_for_seconds
                        print('Fresh shared base: (%.3f, %.3f)' % (base.pose.position.x,base.pose.position.y))
                        if not self.confirm('Navigate MiR to this fresh destination?'): continue
                        if time.monotonic() >= expiry: raise RuntimeError('Selection expired again; retry workspace')
                    self.navigate(base, generation)
                    self.log_actual_arrival(base)
                    while rclpy.ok() and self.motion_fault is None:
                        print('Points: ' + ', '.join('P%d (%s)' % (i+1,a) for i,a in enumerate(result.point_arms)))
                        choice = input('Move to P1/P2/..., home, new workspace, or quit: ').strip().lower()
                        if choice in ('quit','q'): return
                        if choice == 'new': break
                        try:
                            if choice == 'home':
                                self.home(generation); continue
                            try: index = int((choice[1:] if choice.startswith('p') else choice)) - 1
                            except ValueError:
                                print('Choose a listed point, home, new or quit.'); continue
                            if not 0 <= index < len(points):
                                print('Point does not exist.'); continue
                            arm = result.point_arms[index]
                            if arm == 'both':
                                arm = input('Both arms can reach this point. Move left or right: ').strip().lower()
                                if arm not in ('left','right'):
                                    print('Choose left or right.'); continue
                            self.execute_commander_goal(arm, points[index], generation)
                        except Exception as exc:
                            self.get_logger().error('Motion failed: %s' % exc)
                except Exception as exc:
                    self.get_logger().error('Workspace stopped: %s' % exc)
        except (KeyboardInterrupt, EOFError):
            pass


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
