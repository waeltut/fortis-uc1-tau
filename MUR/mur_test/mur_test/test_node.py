"""Manually gated MiR navigation -> fresh arm IK -> MoveIt plan -> arm execution."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import RobotState, MoveItErrorCodes, JointConstraint, Constraints
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose


class MurTest(Node):
    def __init__(self):
        super().__init__('mur_test')
        self.declare_parameter('target_topic', '/base_selection/target')
        self.declare_parameter('selected_pose_topic', '/base_selection/selected_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('navigation_action', '/navigate_to_pose')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('execute_action', '/execute_trajectory')
        self.declare_parameter('arm_group', 'left_ur_manipulator')
        self.declare_parameter('tcp_link', 'left_tcp')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('base_goal_frame', 'map')
        self.declare_parameter('planning_pipeline_id', '')
        self.declare_parameter('planner_id', '')
        self.declare_parameter('allowed_planning_time', 10.0)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.declare_parameter('ik_timeout', 3.0)
        self.declare_parameter('ik_attempts', 5)
        self.declare_parameter('joint_goal_tolerance', 0.01)
        self.declare_parameter('interface_timeout', 15.0)
        self.declare_parameter('navigation_timeout', 180.0)
        self.declare_parameter('planning_timeout', 45.0)
        self.declare_parameter('execution_timeout', 120.0)
        self.declare_parameter('pose_wait_timeout', 30.0)
        self.declare_parameter('tf_timeout', 5.0)
        self.declare_parameter('max_joint_state_age', 3.0)
        self.declare_parameter('max_planned_start_deviation', 0.10)
        self.declare_parameter('ik_avoid_collisions', True)
        self.lock = threading.Lock()
        self.target = None
        self.selected = None
        self.selection_time = 0.0
        self.target_generation = 0
        self.joints = None
        self.joints_time = 0.0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, qos)
        self.create_subscription(PoseStamped, self.p('selected_pose_topic'), self.on_selected, qos)
        self.create_subscription(JointState, self.p('joint_states_topic'), self.on_joints, 20)
        self.nav = ActionClient(self, NavigateToPose, self.p('navigation_action'))
        self.ik = self.create_client(GetPositionIK, self.p('ik_service'))
        self.plan = ActionClient(self, MoveGroup, self.p('move_group_action'))
        self.execute = ActionClient(self, ExecuteTrajectory, self.p('execute_action'))
        self.get_logger().info('Ready. Console prompts control all motion; no automatic navigation or execution.')
        self.get_logger().info('Target: %s | selected pose: %s (both PoseStamped)' %
                               (self.p('target_topic'), self.p('selected_pose_topic')))

    def p(self, key):
        return self.get_parameter(key).value

    def on_target(self, msg):
        with self.lock:
            self.target = msg
            self.target_generation += 1
            self.selected = None  # never navigate to a stale goal from the previous target
            self.selection_time = 0.0
        self.get_logger().info('TCP target received in %s. Waiting for a newly selected base pose.' % msg.header.frame_id)

    def on_selected(self, msg):
        with self.lock:
            if self.target is None:
                return
            self.selected = msg
            self.selection_time = time.monotonic()
        self.get_logger().info('Selected base pose received in %s: x=%.3f y=%.3f' %
                               (msg.header.frame_id, msg.pose.position.x, msg.pose.position.y))

    def on_joints(self, msg):
        with self.lock:
            self.joints = msg
            self.joints_time = time.monotonic()

    def wait_future(self, future, timeout, what):
        end = time.monotonic() + float(timeout)
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not future.done():
            raise TimeoutError('%s timed out after %.1f seconds' % (what, timeout))
        return future.result()  # propagate service/action errors

    def wait_for(self, condition, timeout, message):
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            with self.lock:
                value = condition()
            if value:
                return value
            time.sleep(0.1)
        raise TimeoutError(message)

    def confirm(self, question):
        while True:
            response = input('\n%s [y/N]: ' % question).strip().lower()
            if response in ('y', 'yes'):
                return True
            if response in ('', 'n', 'no'):
                return False
            print('Enter y or n.')

    @staticmethod
    def pose_ok(pose):
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return all(math.isfinite(v) for v in xyz + quat) and abs(sum(v*v for v in quat) - 1.0) < 0.02

    def transform_to(self, pose, frame):
        if not pose.header.frame_id:
            raise ValueError('Incoming pose has no frame_id')
        if not self.pose_ok(pose.pose):
            raise ValueError('Invalid pose or non-unit quaternion')
        if pose.header.frame_id == frame:
            return pose
        end = time.monotonic() + float(self.p('tf_timeout'))
        while time.monotonic() < end:
            try:
                # Use latest available TF for execution-time recalculation. The original target
                # coordinates are preserved, but its ROS timestamp may be old after navigation.
                tf = self.tf.lookup_transform(frame, pose.header.frame_id, rclpy.time.Time())
                transformed = PoseStamped()
                transformed.header.frame_id = frame
                transformed.header.stamp = self.get_clock().now().to_msg()
                transformed.pose = do_transform_pose(pose.pose, tf)
                return transformed
            except TransformException:
                time.sleep(0.1)
        raise RuntimeError('No transform %s <- %s' % (frame, pose.header.frame_id))

    def current_state(self):
        with self.lock:
            js = self.joints
            age = time.monotonic() - self.joints_time
        if js is None or age > float(self.p('max_joint_state_age')):
            raise RuntimeError('Missing/stale /joint_states (age %.2fs)' % age)
        if len(js.name) != len(js.position):
            raise RuntimeError('JointState names/positions mismatch')
        state = RobotState()
        state.joint_state = js
        state.is_diff = False
        return state

    def use_snapshot(self):
        with self.lock:
            return self.target_generation, self.target, self.selected

    def check_snapshot(self, generation):
        with self.lock:
            if self.target_generation != generation:
                raise RuntimeError('A new target was received; cancel this run and restart')

    def go_nav(self, base_pose):
        if not self.nav.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('Nav2 NavigateToPose action unavailable')
        goal = NavigateToPose.Goal()
        goal.pose = base_pose
        handle = self.wait_future(self.nav.send_goal_async(goal), self.p('interface_timeout'), 'Nav goal response')
        if not handle or not handle.accepted:
            raise RuntimeError('Nav2 rejected the navigation goal')
        print('Navigation accepted; monitoring until result...')
        try:
            result = self.wait_future(handle.get_result_async(), self.p('navigation_timeout'), 'Navigation')
        except (TimeoutError, KeyboardInterrupt):
            handle.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Navigation failed, status=%d' % result.status)
        print('Nav2 succeeded.')

    def get_ik(self, target):
        if not self.ik.wait_for_service(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('MoveIt /compute_ik service unavailable')
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.p('arm_group')
        if hasattr(req.ik_request, 'attempts'):
            req.ik_request.attempts = int(self.p('ik_attempts'))
        req.ik_request.ik_link_name = self.p('tcp_link')
        req.ik_request.pose_stamped = target
        req.ik_request.robot_state = self.current_state()
        req.ik_request.avoid_collisions = bool(self.p('ik_avoid_collisions'))
        req.ik_request.timeout = rclpy.duration.Duration(seconds=float(self.p('ik_timeout'))).to_msg()
        response = self.wait_future(self.ik.call_async(req),
                                    float(self.p('ik_timeout')) + float(self.p('interface_timeout')),
                                    'MoveIt IK')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('IK failed, MoveIt code=%d' % response.error_code.val)
        current = set(self.current_state().joint_state.name)
        positions = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return {name: positions[name] for name in positions if name in current and
                (name.startswith('left_') if self.p('arm_group').startswith('left_') else
                 name.startswith('right_') if self.p('arm_group').startswith('right_') else True)}

    def build_plan_request(self, goal_joints):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.p('arm_group')
        req.pipeline_id = self.p('planning_pipeline_id')
        req.planner_id = self.p('planner_id')
        req.num_planning_attempts = 5
        req.allowed_planning_time = float(self.p('allowed_planning_time'))
        req.max_velocity_scaling_factor = float(self.p('velocity_scaling'))
        req.max_acceleration_scaling_factor = float(self.p('acceleration_scaling'))
        req.start_state = self.current_state()
        constraint = Constraints()
        constraint.name = 'mur_test_ik_joint_goal'
        for name, value in sorted(goal_joints.items()):
            j = JointConstraint()
            j.joint_name = name
            j.position = float(value)
            j.tolerance_above = float(self.p('joint_goal_tolerance'))
            j.tolerance_below = float(self.p('joint_goal_tolerance'))
            j.weight = 1.0
            constraint.joint_constraints.append(j)
        req.goal_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def plan_arm(self, goals):
        if not self.plan.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('MoveGroup /move_action unavailable')
        request = self.build_plan_request(goals)
        handle = self.wait_future(self.plan.send_goal_async(request), self.p('interface_timeout'), 'MoveGroup goal response')
        if not handle or not handle.accepted:
            raise RuntimeError('MoveGroup rejected the request')
        result = self.wait_future(handle.get_result_async(), self.p('planning_timeout'), 'MoveGroup planning')
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('MoveIt plan failed: status=%d error=%d' %
                               (result.status, result.result.error_code.val))
        trajectory = result.result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError('MoveIt returned an empty joint trajectory')
        return trajectory

    def validate_start(self, trajectory):
        # Prevent execution if the robot has moved away from the state at plan time.
        js = self.current_state().joint_state
        positions = dict(zip(js.name, js.position))
        traj = trajectory.joint_trajectory
        first = traj.points[0]
        threshold = float(self.p('max_planned_start_deviation'))
        for i, name in enumerate(traj.joint_names):
            if name not in positions:
                raise RuntimeError('Missing current joint %s' % name)
            error = abs(positions[name] - first.positions[i])
            if error > threshold:
                raise RuntimeError('Joint %s changed by %.3f rad since planning' % (name, error))

    def execute_arm(self, trajectory):
        self.validate_start(trajectory)
        if not self.execute.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('MoveIt /execute_trajectory action unavailable')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self.wait_future(self.execute.send_goal_async(goal), self.p('interface_timeout'), 'Execute goal response')
        if not handle or not handle.accepted:
            raise RuntimeError('MoveIt rejected the execute request')
        print('Arm trajectory executing...')
        try:
            result = self.wait_future(handle.get_result_async(), self.p('execution_timeout'), 'Arm execution')
        except (TimeoutError, KeyboardInterrupt):
            handle.cancel_goal_async()
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError('Arm execution failed: status=%d code=%d' %
                               (result.status, result.result.error_code.val))
        print('Arm execution succeeded.')

    def run_console(self):
        print('\n=== mur_test: gated Nav2 -> IK -> MoveIt plan -> execution ===')
        print('Publish the TCP target and let mur_base_selector publish its selected base pose.')
        print('Press Ctrl+C to exit. Responses default to NO.\n')
        while rclpy.ok():
            try:
                generation, target, base = self.wait_for(
                    lambda: (self.target_generation, self.target, self.selected)
                    if self.target is not None and self.selected is not None else None,
                    self.p('pose_wait_timeout'), 'Waiting for target and selected base pose')
                if not self.pose_ok(target.pose) or not self.pose_ok(base.pose):
                    raise ValueError('Target or base pose contains invalid quaternion')
                print('\nTCP: %s (%.3f, %.3f, %.3f)' %
                      (target.header.frame_id, target.pose.position.x, target.pose.position.y, target.pose.position.z))
                print('Base: %s (%.3f, %.3f, %.3f)' %
                      (base.header.frame_id, base.pose.position.x, base.pose.position.y, base.pose.position.z))
                if not self.confirm('Navigate MiR to this selected base pose?'):
                    print('Navigation skipped. Waiting for next target.')
                    self.discard(generation)
                    continue
                self.check_snapshot(generation)
                base = self.transform_to(base, self.p('base_goal_frame'))
                self.go_nav(base)
                self.check_snapshot(generation)
                # IMPORTANT: world/chest poses transformed only AFTER Nav2 reaches its goal.
                # Do not reuse pre-navigation arm geometry or cached reachability IK.
                target_arm = self.transform_to(target, self.p('planning_frame'))
                print('Post-navigation TCP target in %s: xyz=(%.4f, %.4f, %.4f)' %
                      (target_arm.header.frame_id, target_arm.pose.position.x,
                       target_arm.pose.position.y, target_arm.pose.position.z))
                solution = self.get_ik(target_arm)
                if not solution:
                    raise RuntimeError('IK returned no applicable arm joints. Check arm_group and naming.')
                print('IK solution: ' + ', '.join('%s=%.3f' % p for p in sorted(solution.items())))
                trajectory = self.plan_arm(solution)
                points = trajectory.joint_trajectory.points
                duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
                print('MoveIt plan successful: %d points, %.2f s, %d joints' %
                      (len(points), duration, len(trajectory.joint_trajectory.joint_names)))
                if not self.confirm('Execute this planned ARM trajectory?'):
                    print('Arm execution skipped.')
                    self.discard(generation)
                    continue
                self.check_snapshot(generation)
                self.execute_arm(trajectory)
                self.discard(generation)
            except TimeoutError as exc:
                # The initial wait timeout is normal when there is no target.
                if 'Waiting for target' not in str(exc):
                    self.get_logger().error(str(exc))
            except (KeyboardInterrupt, EOFError):
                break
            except Exception as exc:
                self.get_logger().error('Test stopped safely: %s' % exc)
                with self.lock:
                    self.selected = None
                print('Re-publish the TCP target to start a new test.')

    def discard(self, generation):
        with self.lock:
            if self.target_generation == generation:
                self.target = None
                self.selected = None


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
