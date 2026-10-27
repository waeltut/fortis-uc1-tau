"""Manually gated MiR navigation -> fresh arm IK -> MoveIt plan -> arm execution."""
import math
import json
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
from std_msgs.msg import String
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
        self.declare_parameter('selection_diagnostics_topic', '/base_selection/candidate_diagnostics')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('navigation_action', '/navigate_to_pose')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('execute_action', '/execute_trajectory')
        self.declare_parameter('arm_group', 'auto')  # auto, left_ur_manipulator, right_ur_manipulator
        self.declare_parameter('tcp_link', '')  # empty = link chosen from arm_group
        self.declare_parameter('planning_frame', 'mur')
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
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_base_xy_error', 0.15)  # m, 0 disables check
        self.declare_parameter('max_base_yaw_error_deg', 15.0)  # degrees, 0 disables check
        self.lock = threading.Lock()
        self.target = None
        self.selected = None
        self.selection_time = 0.0
        self.selected_metadata = None
        self.target_generation = 0
        self.joints = None
        self.joints_time = 0.0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, qos)
        self.create_subscription(PoseStamped, self.p('selected_pose_topic'), self.on_selected, qos)
        self.create_subscription(String, self.p('selection_diagnostics_topic'), self.on_diagnostics, qos)
        self.create_subscription(JointState, self.p('joint_states_topic'), self.on_joints, 20)
        self.nav = ActionClient(self, NavigateToPose, self.p('navigation_action'))
        self.ik = self.create_client(GetPositionIK, self.p('ik_service'))
        self.plan = ActionClient(self, MoveGroup, self.p('move_group_action'))
        self.execute = ActionClient(self, ExecuteTrajectory, self.p('execute_action'))
        self.get_logger().info('Ready. Console prompts control all motion; no automatic navigation or execution.')
        self.get_logger().info('MoveIt IK/planning target frame: %s' % self.p('planning_frame'))
        self.get_logger().info('Arm selection mode: %s (auto follows base selector result)' % self.p('arm_group'))
        if str(self.p('arm_group')) != 'auto':
            self.get_logger().warning('Arm is manually forced. Set arm_group:=auto to follow the selector.')
        self.get_logger().info('Target: %s | selected pose: %s (both PoseStamped)' %
                               (self.p('target_topic'), self.p('selected_pose_topic')))

    def p(self, key):
        return self.get_parameter(key).value

    def on_target(self, msg):
        with self.lock:
            # The selector can publish the same target more than once. A duplicate
            # must not invalidate an already selected base or an in-progress run.
            if self.target is not None and self.same_target(self.target, msg):
                return
            self.target = msg
            self.target_generation += 1
            self.selected = None  # never navigate to a stale goal from the previous target
            self.selection_time = 0.0
            self.selected_metadata = None
        self.get_logger().info('TCP target received in %s. Waiting for a newly selected base pose.' % msg.header.frame_id)

    def on_selected(self, msg):
        with self.lock:
            if self.target is None:
                return
            self.selected = msg
            self.selection_time = time.monotonic()
        self.get_logger().info('Selected base pose received in %s: x=%.3f y=%.3f' %
                               (msg.header.frame_id, msg.pose.position.x, msg.pose.position.y))
        # A selected PoseStamped contains no arm identity. The matching selector
        # diagnostics JSON supplies that identity; never infer it from pose Y.

    def on_diagnostics(self, msg):
        """Record the arm chosen by selector, when available in its JSON diagnostics.

        In particular, candidate reachability does not provide an exact IK
        solution and the selector may select the *right* arm. Diagnostic status
        messages without a selected pose are ignored.
        """
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            arm = data.get('arm')
            pose = data.get('pose')
            if arm not in ('left', 'right', 'both') or not isinstance(pose, list) or len(pose) < 2:
                return
            if not data.get('valid', True):
                return
            if not all(math.isfinite(float(v)) for v in pose[:2]):
                return
        except (ValueError, TypeError, OverflowError):
            return
        with self.lock:
            if self.target is not None:
                self.selected_metadata = (data, time.monotonic())

    def chosen_arm(self, selected):
        with self.lock:
            info = self.selected_metadata
            selected_at = self.selection_time
        if not info:
            return None
        data, metadata_at = info
        # The selector publishes its result and PoseStamped near-simultaneously.
        # An old latched diagnostic must not select the arm for a new request.
        if selected_at <= 0 or abs(metadata_at - selected_at) > 5.0:
            return None
        if data.get('frame_id', selected.header.frame_id) != selected.header.frame_id:
            return None
        xyz = data.get('pose', [])
        if len(xyz) < 3:
            return None
        # Compare position AND heading; two candidate locations may share XY.
        if math.hypot(float(xyz[0])-selected.pose.position.x,
                      float(xyz[1])-selected.pose.position.y) > 0.03:
            return None
        if len(xyz) >= 3 and math.isfinite(float(xyz[2])):
            dyaw = float(xyz[2]) - self.yaw(selected.pose.orientation)
            if abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) > math.radians(5):
                return None
        return data.get('arm')

    def arm_candidates(self, selected):
        mode = str(self.p('arm_group'))
        explicitly = {'left_ur_manipulator': ['left'], 'right_ur_manipulator': ['right'],
                      'left': ['left'], 'right': ['right']}
        if mode in explicitly:
            return explicitly[mode]
        if mode != 'auto':
            raise ValueError('arm_group must be auto, left_ur_manipulator, or right_ur_manipulator')
        selected_arm = self.chosen_arm(selected)
        if selected_arm in ('left', 'right'):
            self.get_logger().info('Matched base selector result: %s arm. IK + MoveIt will use %s_tcp.' %
                                   (selected_arm.upper(), selected_arm))
            return [selected_arm]
        if selected_arm == 'both':
            self.get_logger().info('Base selector chose BOTH arms; testing single-arm IK on left and right.')
        else:
            self.get_logger().warning('No matching arm metadata for the selected base pose. '
                                      'Testing both arms using exact collision-aware IK; '
                                      'will only plan using an arm whose IK succeeds.')
        return ['left', 'right']

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

    @staticmethod
    def same_target(a, b):
        """Treat identical repeated PoseStamped targets as one request.

        A target received after completion/failure remains a new request, since
        discard() clears the stored target. Quaternions q and -q are equivalent.
        """
        if a.header.frame_id != b.header.frame_id:
            return False
        if not MurTest.pose_ok(a.pose) or not MurTest.pose_ok(b.pose):
            return False
        pa, pb = a.pose.position, b.pose.position
        if math.dist((pa.x, pa.y, pa.z), (pb.x, pb.y, pb.z)) > 1e-6:
            return False
        qa, qb = a.pose.orientation, b.pose.orientation
        dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
        return abs(abs(dot) - 1.0) < 1e-6

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

    def get_ik(self, target, arm):
        if not self.ik.wait_for_service(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('MoveIt /compute_ik service unavailable')
        req = GetPositionIK.Request()
        req.ik_request.group_name = '%s_ur_manipulator' % arm
        if hasattr(req.ik_request, 'attempts'):
            req.ik_request.attempts = int(self.p('ik_attempts'))
        # In auto mode the selected arm defines both the group and TCP link.
        # Do not allow an old left_tcp parameter to override a RIGHT selection.
        tcp_link = ('%s_tcp' % arm if self.p('arm_group') == 'auto'
                    else (self.p('tcp_link') or '%s_tcp' % arm))
        if tcp_link != '%s_tcp' % arm:
            raise RuntimeError('tcp_link=%s conflicts with chosen %s arm' %
                               (tcp_link, arm))
        req.ik_request.ik_link_name = tcp_link
        self.get_logger().info('Requesting exact IK: group=%s, ik_link=%s, frame=%s' %
                               (req.ik_request.group_name, tcp_link, target.header.frame_id))
        req.ik_request.pose_stamped = target
        req.ik_request.robot_state = self.current_state()
        req.ik_request.avoid_collisions = bool(self.p('ik_avoid_collisions'))
        req.ik_request.timeout = rclpy.duration.Duration(seconds=float(self.p('ik_timeout'))).to_msg()
        response = self.wait_future(self.ik.call_async(req),
                                    float(self.p('ik_timeout')) + float(self.p('interface_timeout')),
                                    'MoveIt IK')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            reason = 'NO_IK_SOLUTION' if response.error_code.val == -31 else 'MoveIt error'
            raise RuntimeError('%s IK failed: %s (%d)' % (arm, reason, response.error_code.val))
        current = set(self.current_state().joint_state.name)
        positions = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return {name: positions[name] for name in positions
                if name in current and name.startswith('%s_' % arm)}

    def build_plan_request(self, goal_joints, arm):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = '%s_ur_manipulator' % arm
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

    def plan_arm(self, goals, arm):
        if not self.plan.wait_for_server(timeout_sec=float(self.p('interface_timeout'))):
            raise RuntimeError('MoveGroup /move_action unavailable')
        request = self.build_plan_request(goals, arm)
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

    @staticmethod
    def yaw(q):
        # atan2 representation of yaw from a valid unit quaternion.
        return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def verify_base_arrival(self, selected_map):
        """Do not treat a coarse Nav2 SUCCESS tolerance as manipulation accuracy."""
        origin = PoseStamped()
        origin.header.frame_id = self.p('base_frame')
        origin.pose.orientation.w = 1.0
        actual = self.transform_to(origin, self.p('base_goal_frame'))
        requested = selected_map.pose
        observed = actual.pose
        err = math.hypot(observed.position.x-requested.position.x,
                         observed.position.y-requested.position.y)
        diff = self.yaw(observed.orientation) - self.yaw(requested.orientation)
        yaw_err = abs(math.atan2(math.sin(diff), math.cos(diff)))
        self.get_logger().info('Base arrival error: XY=%.3f m, yaw=%.1f deg '
                               '(requested=(%.3f, %.3f), actual=(%.3f, %.3f))' %
                               (err, math.degrees(yaw_err), requested.position.x,
                                requested.position.y, observed.position.x, observed.position.y))
        xy_max = float(self.p('max_base_xy_error'))
        yaw_max = float(self.p('max_base_yaw_error_deg'))
        if (xy_max > 0 and err > xy_max) or (yaw_max > 0 and math.degrees(yaw_err) > yaw_max):
            raise RuntimeError('Nav2 reported success, but the selected base pose was not '
                               'reached accurately enough for manipulation. Tune controller '
                               'goal_checker.xy_goal_tolerance / yaw_goal_tolerance; '
                               'do not proceed with arm IK from this position.')

    def run_console(self):
        print('\n=== mur_test: gated Nav2 -> IK -> MoveIt plan -> execution ===')
        print('Publish the TCP target and let mur_base_selector publish its selected base pose.')
        print('Press Ctrl+C to exit. Responses default to NO.\n')
        while rclpy.ok():
            generation = None
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
                self.verify_base_arrival(base)
                # Convert the map-fixed TCP goal to MoveIt's model frame (mur) only
                # AFTER Nav2 succeeds, using the latest actual map -> mur TF.
                # Do not reuse pre-navigation arm geometry or cached reachability IK.
                target_arm = self.transform_to(target, self.p('planning_frame'))
                p, q = target_arm.pose.position, target_arm.pose.orientation
                print('Post-navigation TCP in %s: xyz=(%.4f, %.4f, %.4f), xyzw=(%.5f, %.5f, %.5f, %.5f)' %
                      (target_arm.header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                attempts = []
                solution = None
                chosen = None
                for arm in self.arm_candidates(base):
                    self.check_snapshot(generation)
                    try:
                        candidate = self.get_ik(target_arm, arm)
                        if not candidate:
                            raise RuntimeError('%s arm IK returned no relevant joints' % arm)
                        chosen, solution = arm, candidate
                        break
                    except RuntimeError as error:
                        attempts.append(str(error))
                        self.get_logger().warning(str(error))
                if solution is None:
                    raise RuntimeError('No exact collision-aware IK for this TCP target: ' +
                                       '; '.join(attempts) +
                                       '. Cached volume checks are approximate; '
                                       'check selected arm, Nav2 arrival, and full TCP orientation.')
                print('Selected arm: %s | MoveIt group: %s_ur_manipulator | TCP: %s_tcp' %
                      (chosen.upper(), chosen, chosen))
                print('IK solution: ' + ', '.join('%s=%.3f' % p for p in sorted(solution.items())))
                trajectory = self.plan_arm(solution, chosen)
                points = trajectory.joint_trajectory.points
                duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
                print('MoveIt plan successful: %d points, %.2f s, %d joints' %
                      (len(points), duration, len(trajectory.joint_trajectory.joint_names)))
                if not self.confirm('Execute planned %s ARM trajectory?' % chosen.upper()):
                    print('Arm execution skipped.')
                    self.discard(generation)
                    continue
                self.check_snapshot(generation)
                self.execute_arm(trajectory)
                self.discard(generation)
            except TimeoutError as exc:
                # The initial wait timeout is normal when there is no target.
                if 'Waiting for target' not in str(exc):
                    self.get_logger().error('Test stopped safely: %s' % exc)
                if generation is not None:
                    self.discard(generation)
            except (KeyboardInterrupt, EOFError):
                break
            except Exception as exc:
                self.get_logger().error('Test stopped safely: %s' % exc)
                # Clear only the failed request. Do not discard a newer target
                # that arrived while the old action/service was running.
                if generation is not None:
                    self.discard(generation)
                print('Re-publish the TCP target to start a new test.')

    def discard(self, generation):
        with self.lock:
            if self.target_generation == generation:
                self.target = None
                self.selected = None
                self.selected_metadata = None


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
