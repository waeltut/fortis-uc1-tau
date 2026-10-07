"""ROS 2 Humble selector. It never sends navigation or arm execution goals."""
import copy
from dataclasses import dataclass, field
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PolygonStamped
from sensor_msgs.msg import JointState, LaserScan
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from mur_reachability.srv import FindBaseCandidates
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .core import (Grid, hull, padded, transform_points, compose, angle,
                   swept_polygons, path_length, score, diverse_order)


DEFAULTS = {
    'fixed_frame': 'map', 'base_frame': 'base_link', 'reachability_frame': 'odom',
    'arm_mode': 'either', 'reachability_service': '/find_base_candidates',
    'time_budget': 20.0, 'max_verified_per_arm': 150,
    'reachability_timeout': 30.0, 'service_timeout': 3.0,
    'planner_action': '/compute_path_to_pose', 'planner_id': 'GridBased',
    'global_costmap_service': '/global_costmap/get_costmap',
    'local_costmap_service': '/local_costmap/get_costmap',
    'footprint_topic': '/local_costmap/published_footprint',
    'joint_states_topic': '/joint_states',
    'scan_topic': '/scan',
    'lifecycle_services': ['/planner_server/get_state', '/controller_server/get_state'],
    'arm_joints': [f'{side}_{joint}_joint' for side in ('left', 'right')
                   for joint in ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')],
    'max_data_age': 3.0, 'tf_timeout': 0.5,
    'footprint_padding': 0.02, 'allow_unknown': False,
    'shortlist_size': 15, 'max_path_checks': 60,
    'planning_budget': 30.0, 'path_timeout': 3.0, 'cancel_timeout': 2.0,
    'diversity_distance': 0.20, 'diversity_angle': 0.35,
    'path_linear_step': 0.05, 'path_angular_step': 0.08,
    'endpoint_xy_tolerance': 0.02, 'endpoint_yaw_tolerance': 0.03,
    'base_motion_tolerance': 0.03, 'base_rotation_tolerance': 0.03,
    'joint_motion_tolerance': 0.04,
    'target_drift_tolerance': 0.005, 'target_rotation_drift_tolerance': 0.01,
    'length_scale': 5.0, 'weight_path_length': 1.0,
    'weight_cost': 2.0, 'weight_heading': 0.2, 'weight_joint_motion': 0.2,
    'selection_ttl': 15.0,
}


class SelectionError(RuntimeError):
    pass


class Cancelled(SelectionError):
    pass


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def quaternion(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    n = np.linalg.norm(v)
    if not np.isfinite(v).all() or n < 1e-9 or abs(n-1) > .05:
        raise SelectionError('Invalid quaternion (must be unit length)')
    return v/n


def matrix(q):
    x, y, z, w = quaternion(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def planar(pose):
    r = matrix(pose.orientation)
    if abs(r[2, 0]) > .02 or abs(r[2, 1]) > .02 or r[2, 2] < .999:
        raise SelectionError('Non-planar base pose or transform')
    v = (pose.position.x, pose.position.y, math.atan2(r[1, 0], r[0, 0]))
    if not np.isfinite(v).all():
        raise SelectionError('Non-finite pose')
    return v


def pose_msg(p, frame):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y = float(p[0]), float(p[1])
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(p[2]/2), math.cos(p[2]/2)
    return msg


@dataclass
class Candidate:
    id: int
    pose: tuple
    arm: str
    solutions: list
    joint_cost: float
    preliminary: float = 0.0
    score: float = math.inf
    state: str = 'unexamined'
    path: list = field(default_factory=list)
    length: float = 0.0
    cost: float = 0.0


class BaseSelector(Node):
    def __init__(self):
        super().__init__('mur_base_selector')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in DEFAULTS}
        self.validate_config()
        # Config is deliberately immutable during a query (restart to change it).
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.group = ReentrantCallbackGroup()
        self.tf = Buffer(cache_time=Duration(seconds=120.0))
        self.listener = TransformListener(self.tf, self)
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.busy = False
        self.worker = None
        self.last_target = None
        self.footprint = None
        self.joints = {}
        self.scan_seen = None
        self.inflight_reach = None
        self.inflight_plan = None
        self.plan_pending = False
        self.selection_deadline = 0.0
        self.sequence = 0
        self.selected_context = None
        self.reach = self.create_client(FindBaseCandidates, self.cfg['reachability_service'], callback_group=self.group)
        self.maps = [self.create_client(GetCostmap, self.cfg[key], callback_group=self.group)
                     for key in ('global_costmap_service', 'local_costmap_service')]
        self.planner = ActionClient(self, ComputePathToPose, self.cfg['planner_action'], callback_group=self.group)
        self.lifecycle = [self.create_client(GetState, name, callback_group=self.group)
                          for name in self.cfg['lifecycle_services']]
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Volatile best effort subscriber is compatible with reliable/transient publishers too.
        sensors = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PolygonStamped, self.cfg['footprint_topic'], self.on_footprint, sensors)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self.on_joints, sensors)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, sensors)
        self.create_subscription(PoseStamped, '/base_selection/target', self.on_target, 10)
        self.create_service(Trigger, '/base_selection/reselect', self.reselect)
        self.create_service(Trigger, '/base_selection/cancel', self.cancel)
        self.status_pub = self.create_publisher(String, '/base_selection/status', transient)
        self.valid_pub = self.create_publisher(Bool, '/base_selection/selected_valid', transient)
        # Pose is volatile: a late subscriber must not receive an old actionable pose.
        self.pose_pub = self.create_publisher(PoseStamped, '/base_selection/selected_pose', 10)
        self.path_pub = self.create_publisher(Path, '/base_selection/selected_path', transient)
        self.result_pub = self.create_publisher(String, '/base_selection/result', transient)
        self.markers_pub = self.create_publisher(MarkerArray, '/base_selection/candidate_markers', transient)
        self.diagnostics_pub = self.create_publisher(String, '/base_selection/candidate_diagnostics', transient)
        self.create_timer(.5, self.expire)
        self.invalidate()
        self.status('idle', 'Ready. Selection only; no motion commands are sent.')

    def validate_config(self):
        c = self.cfg
        if c['arm_mode'] not in ('left', 'right', 'either', 'both'):
            raise ValueError('arm_mode must be left, right, either or both')
        for k, v in c.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool) and (not math.isfinite(v) or v < 0):
                raise ValueError(f'{k} must be finite and non-negative')
        positive = ('time_budget', 'reachability_timeout', 'service_timeout', 'max_data_age', 'tf_timeout',
                    'planning_budget', 'path_timeout', 'cancel_timeout', 'path_linear_step',
                    'path_angular_step', 'length_scale', 'selection_ttl', 'max_verified_per_arm',
                    'shortlist_size', 'max_path_checks')
        if any(c[k] <= 0 for k in positive):
            raise ValueError('Budgets, timeouts, counts, sampling steps and scales must be positive')
        if c['time_budget'] > 60 or c['max_verified_per_arm'] > 1000:
            raise ValueError('Reachability server supports <=60 s and <=1000 poses per arm')
        if c['reachability_timeout'] <= c['time_budget']:
            raise ValueError('reachability_timeout must exceed time_budget')
        if c['max_path_checks'] < c['shortlist_size'] or c['max_path_checks'] > 2000:
            raise ValueError('max_path_checks must be >= shortlist_size and <=2000')
        if not c['arm_joints'] or len(set(c['arm_joints'])) != len(c['arm_joints']):
            raise ValueError('arm_joints must be nonempty and unique')
        if not c['lifecycle_services']:
            raise ValueError('At least one Nav2 lifecycle service is required')

    def parameters_changed(self, params):
        if any(p.name in DEFAULTS or p.name == 'use_sim_time' for p in params):
            return SetParametersResult(successful=False, reason='Restart selector to change configuration')
        return SetParametersResult(successful=True)

    def status(self, state, message, **extra):
        data = {'request_id': self.sequence, 'state': state, 'message': message, **extra}
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        self.get_logger().info(f'[{state}] {message}')

    def invalidate(self):
        with self.lock:
            self.selection_deadline = 0.0
            self.selected_context = None
        self.valid_pub.publish(Bool(data=False))
        self.result_pub.publish(String(data=json.dumps({'request_id': self.sequence, 'valid': False})))
        self.path_pub.publish(Path())
        self.markers_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        self.diagnostics_pub.publish(String(data=json.dumps({'request_id': self.sequence, 'candidates': []})))

    def expire(self):
        with self.lock:
            expired = self.selection_deadline and time.monotonic() > self.selection_deadline
            context = self.selected_context
        if expired:
            self.invalidate()
            self.status('expired', 'Selection validity interval expired; submit target or reselect.')
        elif context is not None:
            try:
                self.guard_motion(*context)
            except Exception as e:
                with self.lock:
                    if self.selected_context is not context:
                        return
                    self.invalidate()
                self.status('invalidated', str(e))

    def on_footprint(self, msg):
        with self.lock:
            self.footprint = (copy.deepcopy(msg), time.monotonic())

    def on_joints(self, msg):
        if len(msg.name) != len(msg.position):
            return
        now = time.monotonic()
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if math.isfinite(value):
                    self.joints[name] = (value, now)

    def on_scan(self, msg):
        if msg.ranges and msg.header.frame_id:
            with self.lock:
                self.scan_seen = (copy.deepcopy(msg.header.stamp), time.monotonic())

    def on_target(self, msg):
        ok, message = self.submit(msg)
        if not ok:
            self.get_logger().warning(message)

    def reselect(self, req, res):
        with self.lock:
            target = copy.deepcopy(self.last_target)
        if target is None:
            res.success, res.message = False, 'Publish /base_selection/target first'
        else:
            # The stored target is anchored in fixed_frame with zero stamp.
            res.success, res.message = self.submit(target)
        return res

    def cancel(self, req, res):
        with self.lock:
            self.stop.set()
            self.invalidate()
        res.success = True
        res.message = 'Selection cancelled. An outstanding reachability call must finish before another query.'
        return res

    def submit(self, target):
        with self.lock:
            if self.busy:
                return False, 'Selector busy; cancel or wait. New target was NOT queued.'
            if self.plan_pending:
                return False, 'Previous planner action is not terminal yet; wait for cancellation/completion.'
            if any(f is not None and not f.done() for f in (self.inflight_reach, self.inflight_plan)):
                return False, 'Previous server request still outstanding; wait for it to finish.'
            if not target.header.frame_id:
                return False, 'Target requires header.frame_id'
            try:
                quaternion(target.pose.orientation)
                p = target.pose.position
                if not np.isfinite([p.x, p.y, p.z]).all():
                    raise ValueError('Non-finite target')
            except (ValueError, SelectionError) as e:
                return False, str(e)
            self.busy = True
            self.stop.clear()
            self.sequence += 1
            self.invalidate()
            self.worker = threading.Thread(target=self.run, args=(copy.deepcopy(target),), daemon=True)
            self.worker.start()
            return True, f'Selection {self.sequence} started'

    def checkpoint(self):
        if self.stop.is_set() or not rclpy.ok():
            raise Cancelled('Selection cancelled')

    def wait(self, future, timeout, cancellable=True):
        end = time.monotonic()+timeout
        while not future.done():
            if cancellable:
                self.checkpoint()
            if not rclpy.ok() or time.monotonic() >= end:
                raise SelectionError('Request timed out')
            time.sleep(.02)
        if future.cancelled():
            raise SelectionError('ROS future cancelled')
        return future.result()

    def lookup(self, target, source, stamp=None):
        if not source:
            raise SelectionError('Missing frame_id')
        return self.tf.lookup_transform(target, source, Time.from_msg(stamp) if stamp else Time(),
                                        timeout=Duration(seconds=self.cfg['tf_timeout']))

    def tf_planar(self, target, source, stamp=None):
        if target == source:
            return (0., 0., 0.)
        tf = self.lookup(target, source, stamp)
        p = PoseStamped().pose
        p.position.x, p.position.y, p.position.z = (tf.transform.translation.x,
                                                  tf.transform.translation.y, tf.transform.translation.z)
        p.orientation = tf.transform.rotation
        return planar(p)

    def transform_target(self, msg, frame):
        if msg.header.frame_id == frame:
            result = copy.deepcopy(msg)
        else:
            tf = self.lookup(frame, msg.header.frame_id, msg.header.stamp)
            a = quaternion(tf.transform.rotation)
            b = quaternion(msg.pose.orientation)
            qv = a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3], b[:3])
            qw = a[3]*b[3]-np.dot(a[:3], b[:3])
            p = msg.pose.position
            t = tf.transform.translation
            v = matrix(tf.transform.rotation) @ [p.x, p.y, p.z] + [t.x, t.y, t.z]
            result = PoseStamped()
            result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, v)
            result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z = map(float, qv)
            result.pose.orientation.w = float(qw)
        result.header.frame_id = frame
        result.header.stamp = Time().to_msg()
        return result

    def robot_pose(self):
        tf = self.lookup(self.cfg['fixed_frame'], self.cfg['base_frame'])
        self.check_age(tf.header.stamp, 'base TF')
        p = PoseStamped().pose
        p.position.x, p.position.y = tf.transform.translation.x, tf.transform.translation.y
        p.orientation = tf.transform.rotation
        return planar(p)

    def check_age(self, stamp, label):
        age = self.get_clock().now().nanoseconds/1e9-stamp_seconds(stamp)
        if age < -.5 or age > self.cfg['max_data_age']:
            raise SelectionError(f'{label} stale or future-dated: age={age:.2f} s')

    def joint_snapshot(self):
        with self.lock:
            data = dict(self.joints)
        missing = [n for n in self.cfg['arm_joints'] if n not in data or
                   time.monotonic()-data[n][1] > self.cfg['max_data_age']]
        if missing:
            raise SelectionError('Missing/stale arm joints: '+', '.join(missing))
        return {n: data[n][0] for n in self.cfg['arm_joints']}

    def footprint_snapshot(self):
        with self.lock:
            data = copy.deepcopy(self.footprint)
        if data is None or time.monotonic()-data[1] > self.cfg['max_data_age']:
            raise SelectionError('Missing/stale published footprint')
        msg = data[0]
        self.check_age(msg.header.stamp, 'footprint')
        tf = self.tf_planar(self.cfg['base_frame'], msg.header.frame_id, msg.header.stamp)
        poly = transform_points([(p.x, p.y) for p in msg.polygon.points], tf)
        return padded(hull(poly), self.cfg['footprint_padding'])

    def guard_motion(self, start, joints):
        self.checkpoint()
        now = self.robot_pose()
        if math.hypot(now[0]-start[0], now[1]-start[1]) > self.cfg['base_motion_tolerance'] or \
                abs(angle(now[2]-start[2])) > self.cfg['base_rotation_tolerance']:
            raise SelectionError('Base moved/localisation changed during selection; reselect while stationary')
        current = self.joint_snapshot()
        if any(abs(current[n]-q) > self.cfg['joint_motion_tolerance'] for n, q in joints.items()):
            raise SelectionError('Arm posture changed during selection; reselect')

    def guard_target(self, requested, fixed):
        current = self.transform_target(requested, self.cfg['fixed_frame'])
        a, b = current.pose.position, fixed.pose.position
        distance = math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)
        dot = abs(float(np.dot(quaternion(current.pose.orientation), quaternion(fixed.pose.orientation))))
        rotation = 2*math.acos(min(1., dot))
        if distance > self.cfg['target_drift_tolerance'] or rotation > self.cfg['target_rotation_drift_tolerance']:
            raise SelectionError('Reachability frame drifted relative to target frame; reselect')

    def costmaps(self):
        with self.lock:
            scan = self.scan_seen
        if scan is None or time.monotonic()-scan[1] > self.cfg['max_data_age']:
            raise SelectionError('Missing/stale scan stream')
        self.check_age(scan[0], 'scan')
        for client in self.lifecycle:
            if not client.wait_for_service(timeout_sec=self.cfg['service_timeout']):
                raise SelectionError(f'Unavailable lifecycle service: {client.srv_name}')
            state = self.wait(client.call_async(GetState.Request()), self.cfg['service_timeout'])
            if state.current_state.id != 3:
                raise SelectionError(f'Nav2 node is not active: {client.srv_name}')
        futures = []
        for client in self.maps:
            if not client.wait_for_service(timeout_sec=self.cfg['service_timeout']):
                raise SelectionError(f'Unavailable costmap service: {client.srv_name}')
            futures.append(client.call_async(GetCostmap.Request()))
        result = []
        for f in futures:
            msg = self.wait(f, self.cfg['service_timeout']).map
            self.check_age(msg.header.stamp, 'costmap')
            if stamp_seconds(msg.metadata.update_time) > 0:
                self.check_age(msg.metadata.update_time, 'costmap update')
            m = msg.metadata
            grid = Grid(msg.data, m.size_x, m.size_y, m.resolution, planar(m.origin))
            transform = self.tf_planar(msg.header.frame_id, self.cfg['fixed_frame'])
            result.append((grid, transform))
        return result

    def polygon_check(self, polygon, maps):
        costs = []
        for i, (grid, tf) in enumerate(maps):
            c = grid.check(transform_points(polygon, tf), partial=i == 1,
                           allow_unknown=self.cfg['allow_unknown'])
            if not c.valid:
                return False, ('global_' if i == 0 else 'local_')+c.reason, 0.
            costs.append(c.cost)
        return True, '', max(costs)

    def candidates(self, response, current_joints):
        arms = []
        for arm in ('left', 'right'):
            poses, solutions = getattr(response, arm+'_verified'), getattr(response, arm+'_solutions')
            if len(poses.poses) != len(solutions):
                raise SelectionError('Reachability pose/solution array length mismatch')
            tf = self.tf_planar(self.cfg['fixed_frame'], poses.header.frame_id)
            group = []
            for p, solution in zip(poses.poses, solutions):
                if not solution.name or len(solution.name) != len(solution.position) or \
                        len(set(solution.name)) != len(solution.name):
                    raise SelectionError('Invalid reachability joint solution')
                if any(n not in current_joints for n in solution.name) or not np.isfinite(solution.position).all():
                    raise SelectionError('Solution contains unknown joints or non-finite positions')
                delta = np.mean([abs(q-current_joints[n]) for n, q in zip(solution.name, solution.position)])
                group.append(Candidate(0, compose(tf, planar(p)), arm, [solution], float(delta/math.pi)))
            arms.append(group)
        mode = self.cfg['arm_mode']
        if mode == 'both':
            # Only virtually identical poses qualify; never merge neighbouring grid cells.
            out = []
            for l in arms[0]:
                for r in arms[1]:
                    if math.hypot(l.pose[0]-r.pose[0], l.pose[1]-r.pose[1]) < 1e-5 and \
                            abs(angle(l.pose[2]-r.pose[2])) < 1e-5:
                        out.append(Candidate(0, l.pose, 'both', l.solutions+r.solutions,
                                             max(l.joint_cost, r.joint_cost)))
                        break
        else:
            out = arms[0] if mode == 'left' else arms[1] if mode == 'right' else arms[0]+arms[1]
        # Same base pose needs only one Nav2 query in either mode; retain lower joint travel.
        unique = {}
        for c in out:
            key = (round(c.pose[0], 5), round(c.pose[1], 5), round(angle(c.pose[2]), 5))
            if key not in unique or c.joint_cost < unique[key].joint_cost:
                unique[key] = c
        out = list(unique.values())
        for i, c in enumerate(out):
            c.id = i
        return out

    def compute_path(self, candidate, start, deadline):
        goal = ComputePathToPose.Goal()
        goal.goal = pose_msg(candidate.pose, self.cfg['fixed_frame'])
        goal.start = pose_msg(start, self.cfg['fixed_frame'])
        goal.use_start = True
        goal.planner_id = self.cfg['planner_id']
        send = self.planner.send_goal_async(goal)
        self.plan_pending = True
        self.inflight_plan = send
        def track_terminal(f):
            try:
                h = f.result()
                if not h.accepted:
                    self.plan_pending = False
                    return
                terminal = h.get_result_async()
                self.inflight_plan = terminal
                def mark_terminal(result):
                    try:
                        if result.result().status in (4, 5, 6):
                            self.plan_pending = False
                    except Exception:
                        pass  # A transport exception does not establish action completion.
                terminal.add_done_callback(mark_terminal)
            except Exception:
                # Keep the gate closed on an unknown transport state.
                pass
        send.add_done_callback(track_terminal)
        handle = None
        result_future = None
        limit = min(self.cfg['path_timeout'], deadline-time.monotonic())
        try:
            if limit <= 0:
                raise SelectionError('Planning budget exhausted')
            begin = time.monotonic()
            handle = self.wait(send, limit)
            if not handle.accepted:
                return None, 'planner_rejected'
            result_future = handle.get_result_async()
            self.inflight_plan = result_future
            remaining = max(.001, limit-(time.monotonic()-begin))
            result = self.wait(result_future, remaining)
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                return None, f'planner_status_{result.status}'
            path = result.result.path
            if not path.poses:
                return None, 'empty_path'
            # Humble has no result.error_code; use the action terminal status.
            poses = []
            frame = path.header.frame_id
            tf = self.tf_planar(self.cfg['fixed_frame'], frame)
            for p in path.poses:
                if p.header.frame_id and p.header.frame_id != frame:
                    raise SelectionError('Inconsistent path frames')
                poses.append(compose(tf, planar(p.pose)))
            for actual, wanted, name in ((poses[0], start, 'start'), (poses[-1], candidate.pose, 'endpoint')):
                if math.hypot(actual[0]-wanted[0], actual[1]-wanted[1]) > self.cfg['endpoint_xy_tolerance']:
                    return None, name+'_position_mismatch'
            if abs(angle(poses[-1][2]-candidate.pose[2])) > self.cfg['endpoint_yaw_tolerance']:
                return None, 'endpoint_heading_mismatch'
            # Explicitly validate the short start connector and any initial rotation.
            return [start]+poses+[candidate.pose], ''
        except Exception:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
                if result_future is not None:
                    # Drain terminal result so the next request cannot preempt this one.
                    try:
                        self.wait(result_future, self.cfg['cancel_timeout'], cancellable=False)
                    except Exception:
                        pass
            elif handle is None:
                # A late accepted goal must also be cancelled. Track its result until terminal.
                def cancel_late(f):
                    try:
                        h = f.result()
                        if h.accepted:
                            self.inflight_plan = h.get_result_async()
                            h.cancel_goal_async()
                    except Exception:
                        pass
                send.add_done_callback(cancel_late)
            raise

    def check_path(self, poses, footprint, maps, deadline=None):
        costs = []
        step = min(self.cfg['path_linear_step'], min(g.resolution for g, _ in maps)*.5)
        for polygon in swept_polygons(footprint, poses, step, self.cfg['path_angular_step']):
            self.checkpoint()
            if deadline is not None and time.monotonic() > deadline:
                raise SelectionError('Planning/checking budget exhausted')
            valid, reason, cost = self.polygon_check(polygon, maps)
            if not valid:
                return False, reason, 0.
            costs.append(cost)
        return True, '', float(np.mean(costs))

    def weights(self):
        return [self.cfg['weight_'+k] for k in ('path_length', 'cost', 'heading', 'joint_motion')]

    def run(self, target):
        started = time.monotonic()
        candidates = []
        try:
            self.status('checking', 'Checking TF, footprint, joint states and services')
            frozen = self.transform_target(target, self.cfg['fixed_frame'])
            with self.lock:
                self.last_target = copy.deepcopy(frozen)
            start = self.robot_pose()
            joints = self.joint_snapshot()
            self.footprint_snapshot()
            if not self.reach.wait_for_service(timeout_sec=self.cfg['service_timeout']):
                raise SelectionError('Reachability service unavailable')
            if not self.planner.wait_for_server(timeout_sec=self.cfg['service_timeout']):
                raise SelectionError('Nav2 planner action unavailable')
            self.costmaps()
            request = FindBaseCandidates.Request()
            request.target = self.transform_target(frozen, self.cfg['reachability_frame'])
            request.time_budget = self.cfg['time_budget']
            request.max_verified_per_arm = self.cfg['max_verified_per_arm']
            self.checkpoint()
            self.status('reachability', f'Requesting verified poses: {request.time_budget:.1f} s, {request.max_verified_per_arm} per arm')
            self.inflight_reach = self.reach.call_async(request)
            # This service has no cancellation. Drain it even if the user cancels locally.
            response = self.wait(self.inflight_reach, self.cfg['reachability_timeout'], cancellable=False)
            self.checkpoint()
            if not response.success:
                raise SelectionError('Reachability failed: '+response.message)
            self.guard_motion(start, joints)
            self.guard_target(request.target, frozen)
            candidates = self.candidates(response, joints)
            if not candidates:
                raise SelectionError('No verified candidates for arm_mode; service response may be truncated')
            maps = self.costmaps()
            footprint = self.footprint_snapshot()
            survivors = []
            for c in candidates:
                self.checkpoint()
                valid, reason, cost = self.polygon_check(transform_points(footprint, c.pose), maps)
                c.state = 'destination_clear' if valid else reason
                if valid:
                    distance = math.hypot(c.pose[0]-start[0], c.pose[1]-start[1])
                    c.preliminary = score(distance, cost, c.pose[2]-start[2], c.joint_cost,
                                          self.weights(), self.cfg['length_scale'])
                    survivors.append(c)
            self.publish_markers(candidates)
            if not survivors:
                raise SelectionError('All returned candidates failed destination footprint checks')
            ordered = diverse_order(survivors, self.cfg['diversity_distance'], self.cfg['diversity_angle'])
            deadline = time.monotonic()+self.cfg['planning_budget']
            feasible = []
            checked = 0
            for c in ordered[:self.cfg['max_path_checks']]:
                if checked >= self.cfg['shortlist_size'] and feasible:
                    break
                if time.monotonic() >= deadline:
                    break
                self.guard_motion(start, joints)
                self.status('planning', f'Checking candidate {checked+1}/{min(len(ordered), self.cfg["max_path_checks"])}',
                            feasible=len(feasible), candidates=len(candidates))
                try:
                    poses, reason = self.compute_path(c, start, deadline)
                except Cancelled:
                    raise
                except Exception as e:
                    c.state = 'planning_timeout_or_error'
                    # Abort this batch rather than racing an unresolved Nav2 action.
                    raise SelectionError(f'Planning interrupted: {e}') from e
                checked += 1
                if poses is None:
                    c.state = reason
                    continue
                valid, reason, cost = self.check_path(poses, footprint, maps, deadline)
                if not valid:
                    c.state = 'path_'+reason
                    continue
                c.path, c.length, c.cost = poses, path_length(poses), cost
                c.score = score(c.length, cost, c.pose[2]-start[2], c.joint_cost,
                                self.weights(), self.cfg['length_scale'])
                c.state = 'path_checked'
                feasible.append(c)
                # First shortlist is evaluated fully; expand only if none passed.
                if checked >= self.cfg['shortlist_size'] and feasible:
                    break
            if not feasible:
                raise SelectionError('No path-checked candidate within the selection budget; not proof of impossibility')
            self.status('revalidating', 'Refreshing costmaps and checking finalists')
            self.guard_motion(start, joints)
            maps = self.costmaps()
            footprint = self.footprint_snapshot()
            fresh_at = time.monotonic()
            chosen = None
            for c in sorted(feasible, key=lambda c: c.score):
                valid, reason, cost = self.check_path(c.path, footprint, maps,
                                                     time.monotonic()+self.cfg['planning_budget'])
                if valid:
                    c.cost = cost
                    c.score = score(c.length, cost, c.pose[2]-start[2], c.joint_cost,
                                    self.weights(), self.cfg['length_scale'])
                    chosen = c
                    break
                c.state = 'revalidation_'+reason
            if chosen is None:
                raise SelectionError('Finalists became invalid on fresh costmaps; reselect')
            if time.monotonic()-fresh_at > self.cfg['max_data_age']:
                raise SelectionError('Final costmap snapshot aged out during checking; reselect')
            self.guard_motion(start, joints)
            self.guard_target(request.target, frozen)
            chosen.state = 'selected'
            now = self.get_clock().now().to_msg()
            pose = pose_msg(chosen.pose, self.cfg['fixed_frame'])
            pose.header.stamp = now
            path = Path()
            path.header = copy.deepcopy(pose.header)
            path.poses = [pose_msg(p, self.cfg['fixed_frame']) for p in chosen.path]
            result = {'request_id': self.sequence, 'valid': True, 'frame_id': self.cfg['fixed_frame'],
                      'arm': chosen.arm, 'pose': list(chosen.pose), 'score': chosen.score,
                      'path_length_m': chosen.length, 'path_cost': chosen.cost,
                      'joint_motion_score': chosen.joint_cost, 'cache_id': response.cache_id,
                      'reachability_budget_exhausted': response.budget_exhausted,
                      'reachability_search_truncated': response.search_truncated,
                      'returned_left': len(response.left_verified.poses), 'returned_right': len(response.right_verified.poses),
                      'path_checks': checked, 'feasible_paths': len(feasible),
                      'elapsed_seconds': time.monotonic()-started, 'valid_for_seconds': self.cfg['selection_ttl'],
                      'environment_arm_collision_checked': False, 'combined_arm_collision_checked': False,
                      'joint_solutions': [{'names': list(s.name), 'positions': list(s.position)} for s in chosen.solutions]}
            with self.lock:
                self.checkpoint()
                self.selection_deadline = time.monotonic()+self.cfg['selection_ttl']
                self.selected_context = (start, joints)
                self.result_pub.publish(String(data=json.dumps(result, allow_nan=False)))
                self.path_pub.publish(path)
                self.pose_pub.publish(pose)
                self.valid_pub.publish(Bool(data=True))
                self.publish_markers(candidates)
                self.status('selected', f'{chosen.arm}: {chosen.length:.2f} m path, score {chosen.score:.3f}', **result)
        except Exception as e:
            if rclpy.ok():
                self.invalidate()
                self.publish_markers(candidates)
                self.status('cancelled' if isinstance(e, Cancelled) else 'failed', str(e))
        finally:
            with self.lock:
                self.busy = False

    def publish_markers(self, candidates):
        data = [{'id': c.id, 'arm': c.arm, 'pose': list(c.pose), 'state': c.state,
                 'score': c.score if math.isfinite(c.score) else None} for c in candidates]
        self.diagnostics_pub.publish(String(data=json.dumps({'request_id': self.sequence,
                                                           'candidates': data}, allow_nan=False)))
        markers = [Marker(action=Marker.DELETEALL)]
        for c in candidates:
            m = Marker()
            m.header.frame_id = self.cfg['fixed_frame']
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'candidates', c.id, Marker.ARROW, Marker.ADD
            m.pose = pose_msg(c.pose, self.cfg['fixed_frame']).pose
            m.pose.position.z = .05
            m.scale.x, m.scale.y, m.scale.z = (.35 if c.state == 'selected' else .15), .025, .025
            colour = ((0., 1., 1.) if c.state == 'selected' else (0., 1., 0.) if c.state == 'path_checked'
                      else (1., .65, 0.) if c.state == 'destination_clear' else (.6, .6, .6)
                      if c.state == 'unexamined' else (1., .1, .1))
            m.color.r, m.color.g, m.color.b = colour
            m.color.a = .9
            m.lifetime = Duration(seconds=self.cfg['selection_ttl']).to_msg()
            markers.append(m)
        self.markers_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = BaseSelector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        # Shutdown wakes worker waits; leave node alive until its worker has stopped.
        if rclpy.ok():
            rclpy.shutdown()
        if node.worker:
            node.worker.join(timeout=5.0)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
