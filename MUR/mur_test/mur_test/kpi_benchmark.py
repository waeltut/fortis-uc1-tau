#!/usr/bin/env python3
"""Non-moving ROS 2 Humble benchmark for a configurable custom base-selection service.

Only communicates with the configured selection service, optionally the Nav2
ComputePathToPose *planning* action, TF and diagnostics. No motion commands.
"""

import argparse
import csv
import json
import math
import random
import statistics
import string
import sys
import time
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import yaml


PLACEHOLDER_POINTS = '__POINTS__'
PLACEHOLDER_TARGET = '__TARGET__'
PLACEHOLDER_ARMS = '__ARMS__' 


def render(value, env, points=None, point_template=None):
    """Substitute placeholders while preserving int/float/bool types."""
    if isinstance(value, str):
        if value in (PLACEHOLDER_POINTS, PLACEHOLDER_TARGET, PLACEHOLDER_ARMS):
            if points is None or point_template is None or not points:
                raise ValueError('Target placeholders require generated points and point_template')
            if value == PLACEHOLDER_POINTS:
                return [render(deepcopy(point_template), p) for p in points]
            if value == PLACEHOLDER_TARGET:
                return render(deepcopy(point_template), points[0])
            return [str(p['arm']) for p in points]
        if value.startswith('${') and value.endswith('}') and value.count('${') == 1:
            key = value[2:-1]
            if key in env:
                return env[key]
        return string.Template(value).safe_substitute({k: str(v) for k, v in env.items()})
    if isinstance(value, list):
        return [render(x, env, points, point_template) for x in value]
    if isinstance(value, dict):
        return {k: render(v, env, points, point_template) for k, v in value.items()}
    return value


def dotted_get(value, key):
    if not key:
        return None
    cur = value
    for part in str(key).split('.'):
        if isinstance(cur, dict):
            if part not in cur:
                return None
            cur = cur[part]
        elif isinstance(cur, list) and part.isdigit() and int(part) < len(cur):
            cur = cur[int(part)]
        else:
            return None
    return cur


def find_first_key(value, names, depth=0):
    if depth > 9:
        return None
    if isinstance(value, dict):
        for k in names:
            if k in value and value[k] is not None:
                return value[k]
        for v in value.values():
            found = find_first_key(v, names, depth + 1)
            if found is not None:
                return found
    elif isinstance(value, list):
        for v in value:
            found = find_first_key(v, names, depth + 1)
            if found is not None:
                return found
    return None


def normalise_pose(value, default_frame):
    """Get PoseStamped dictionary from nested message-to-dict output."""
    if not isinstance(value, dict):
        return None
    if 'pose' in value and isinstance(value['pose'], dict):
        if 'position' in value['pose']:
            return {'header': value.get('header', {'frame_id': default_frame}),
                    'pose': value['pose']}
        return normalise_pose(value['pose'], default_frame)
    if 'position' in value and 'orientation' in value:
        return {'header': {'frame_id': default_frame}, 'pose': value}
    return None


def discover_pose(value, default_frame, depth=0):
    if depth > 9:
        return None
    pose = normalise_pose(value, default_frame)
    if pose is not None:
        return pose
    if isinstance(value, dict):
        for k in ('selected_pose', 'base_pose', 'selected_base_pose', 'pose',
                  'best_pose', 'candidate', 'result'):
            if k in value:
                found = discover_pose(value[k], default_frame, depth+1)
                if found is not None:
                    return found
        for v in value.values():
            found = discover_pose(v, default_frame, depth+1)
            if found is not None:
                return found
    if isinstance(value, list):
        for v in value:
            found = discover_pose(v, default_frame, depth+1)
            if found is not None:
                return found
    return None


def sample_tasks(cfg, center):
    rng = random.Random(int(cfg.get('seed', 2026)))
    common = cfg.get('sampling', {})
    frame = cfg.get('frame', 'map')
    fixed_q = list(common.get('tcp_quaternion', [0, 0, 0, 1]))
    if len(fixed_q) != 4:
        raise ValueError('tcp_quaternion must have four elements [x,y,z,w]')
    cases = []
    counter = 0
    for scenario in cfg['scenarios']:
        label = str(scenario['name'])
        n = int(scenario.get('count', 1))
        npoints = int(scenario.get('points', 1))
        if npoints < 1 or n < 0:
            raise ValueError('count must be non-negative and points >= 1')
        min_radius = float(scenario.get('min_radius_m', common.get('min_radius_m', .6)))
        max_radius = float(scenario.get('max_radius_m', common.get('max_radius_m', 2.3)))
        if min_radius < 0 or max_radius < min_radius:
            raise ValueError('Invalid sampling radii')
        jitter = float(scenario.get('point_spread_m', .2))
        height = float(scenario.get('height_m', common.get('height_m', .95)))
        arm_value = str(scenario.get('arm', 'any'))
        point_arm_choices = scenario.get('point_arms', [arm_value])
        if not isinstance(point_arm_choices, list) or not point_arm_choices:
            raise ValueError('scenario.point_arms must be a nonempty list')
        for rep in range(n):
            counter += 1
            angle = rng.uniform(-math.pi, math.pi)
            r = math.sqrt(rng.uniform(min_radius**2, max_radius**2))
            anchor_x = float(center[0]) + r * math.cos(angle)
            anchor_y = float(center[1]) + r * math.sin(angle)
            points = []
            for i in range(npoints):
                x = anchor_x + (rng.uniform(-jitter, jitter) if i else 0)
                y = anchor_y + (rng.uniform(-jitter, jitter) if i else 0)
                if common.get('orientation_mode', 'fixed') == 'random_yaw':
                    yaw = rng.uniform(-math.pi, math.pi)
                    qx, qy, qz, qw = 0., 0., math.sin(yaw/2), math.cos(yaw/2)
                else:
                    qx, qy, qz, qw = map(float, fixed_q)
                choice = point_arm_choices[i % len(point_arm_choices)]
                constraint = cfg.get('arm_values', {}).get(choice, choice)
                points.append(dict(x=x, y=y, z=height, qx=qx, qy=qy,
                                   qz=qz, qw=qw, arm=constraint, frame=frame))
            if npoints > 1:
                template = scenario.get('request_template', cfg.get('multi_request_template', cfg.get('request_template')))
            else:
                template = scenario.get('request_template', cfg.get('single_request_template', cfg.get('request_template')))
            if template is None:
                raise ValueError('Missing single_request_template/multi_request_template in YAML')
            env = dict(points[0], npoints=npoints)
            request = render(template, env, points, cfg['point_template'])
            cases.append(dict(id=counter, scenario=label, expected_feasible=scenario.get('expected_feasible', True),
                              npoints=npoints, arm_constraint=','.join(p['arm'] for p in points),
                              targets=points, request=request))
    if cfg.get('shuffle', True):
        rng.shuffle(cases)
    return cases


def sample_reference_tasks(cfg, tcp_poses):
    """Kinematically reachable-at-current-base positive controls from real TCP TF.

    Each tcp_poses entry is a map-frame TCP pose dictionary, not a commanded
    robot position. Reachability of the observed pose does not guarantee that
    the selector will accept a navigable base position in the current map.
    """
    frame = cfg.get('frame', 'map')
    point_template = cfg['point_template']
    specs = [
        ('reference_left', [('left', 'left')]),
        ('reference_right', [('right', 'right')]),
        ('reference_left_any', [('left', 'any')]),
        ('reference_right_any', [('right', 'any')]),
        ('reference_both_points', [('left', 'left'), ('right', 'right')]),
    ]
    cases = []
    for case_id, (label, specs_for_case) in enumerate(specs, 1):
        points = []
        for tcp_arm, requested_arm in specs_for_case:
            pose = tcp_poses[tcp_arm]
            position = pose['pose']['position']
            orientation = pose['pose']['orientation']
            constraint = cfg.get('arm_values', {}).get(requested_arm, requested_arm)
            points.append(dict(
                x=float(position['x']), y=float(position['y']), z=float(position['z']),
                qx=float(orientation['x']), qy=float(orientation['y']),
                qz=float(orientation['z']), qw=float(orientation['w']),
                arm=constraint, frame=frame))
        if len(points) > 1:
            template = cfg.get('multi_request_template', cfg.get('request_template'))
        else:
            template = cfg.get('single_request_template', cfg.get('request_template'))
        if template is None:
            raise ValueError('Missing single_request_template or multi_request_template')
        cases.append(dict(id=case_id, scenario=label,
                          expected_feasible=None, reference_type='observed_current_TCP',
                          npoints=len(points), arm_constraint=','.join(p['arm'] for p in points),
                          targets=points,
                          request=render(template, points[0], points, point_template)))
    return cases


def compact_diagnostic(value, max_chars=1100):
    """Show the actual selector diagnostics rather than guessing a failure cause."""
    if not value:
        return ''
    try:
        parsed = json.loads(value) if isinstance(value, str) else value
        return json.dumps(parsed, ensure_ascii=False, separators=(',', ':'))[:max_chars]
    except (ValueError, TypeError):
        return str(value)[:max_chars]

def numeric_summary(vals):
    data = sorted(float(v) for v in vals if v is not None and math.isfinite(float(v)))
    if not data:
        return {'median': None, 'mean': None, 'p95': None}
    p = .95 * (len(data) - 1)
    lo = int(math.floor(p)); hi = int(math.ceil(p))
    return {'median': statistics.median(data), 'mean': statistics.mean(data),
            'p95': (data[lo] if lo == hi else data[lo] * (hi - p) + data[hi] * (p - lo))}


def default_config_path():
    """Use installed ROS package configuration, falling back to source layout."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('mur_test')) / 'config' / 'benchmark_config.yaml'
        if installed.is_file():
            return str(installed)
    except (ImportError, LookupError, OSError):
        pass
    source = Path(__file__).resolve().parent.parent / 'config' / 'benchmark_config.yaml'
    return str(source)


def read_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf8'))
    if not isinstance(data, dict):
        raise ValueError('Configuration root must be a mapping')
    return data


def response_info(response, cfg):
    flag = dotted_get(response, cfg.get('response_success_path'))
    if flag is None:
        flag = find_first_key(response, ('success', 'found', 'is_valid', 'valid'))
    if flag is not None and not isinstance(flag, bool):
        flag = None  # do not confuse numeric scores or strings with boolean validity
    explicit_pose = dotted_get(response, cfg.get('response_pose_path'))
    pose = normalise_pose(explicit_pose, cfg.get('frame', 'map')) if explicit_pose is not None else None
    if pose is None:
        pose = discover_pose(response, cfg.get('frame', 'map'))
    arm = dotted_get(response, cfg.get('response_arm_path'))
    if arm is None:
        arm = find_first_key(response, ('selected_arm', 'arm_name', 'arm', 'chosen_arm'))
    message = dotted_get(response, cfg.get('response_message_path'))
    if message is None:
        message = find_first_key(response, ('message', 'reason', 'error_message'))
    # If the service does not expose a reliable success flag, do not silently
    # count a default/zero pose as success. User config must set an explicit path.
    if flag is True:
        status = 'success'
    elif flag is False:
        status = 'no_solution'
    else:
        status = 'unknown_result'
    return status, pose, str(arm) if arm is not None else '', str(message or '')


def pose_geometry(pose):
    if not pose:
        return None, None, None
    p = pose['pose'].get('position', {})
    q = pose['pose'].get('orientation', {})
    yaw = math.atan2(2 * (q.get('w', 1) * q.get('z', 0) + q.get('x', 0) * q.get('y', 0)),
                     1 - 2 * (q.get('y', 0)**2 + q.get('z', 0)**2))
    return p.get('x'), p.get('y'), math.degrees(yaw)


class RosInterface:
    def __init__(self, service_name, diagnostics_topic=None):
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_service
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.set_message import set_message_fields
        import tf2_ros

        self.rclpy = rclpy
        self.get_service = get_service
        self.message_to_dict = message_to_ordereddict
        self.set_fields = set_message_fields
        rclpy.init()
        self.node = Node('mur_kpi_benchmark')
        self.service_name = service_name
        types = []
        for _ in range(30):
            for name, ts in self.node.get_service_names_and_types():
                if name.rstrip('/') == service_name.rstrip('/'):
                    types = ts
                    break
            if types:
                break
            rclpy.spin_once(self.node, timeout_sec=.2)
        if not types:
            raise RuntimeError('Service not found: %s. Try ros2 service list -t' % service_name)
        self.service_type_name = types[0]
        self.service_type = self.get_service(types[0])
        self.client = self.node.create_client(self.service_type, service_name)
        if not self.client.wait_for_service(timeout_sec=8.):
            raise RuntimeError('Service visible but not accepting requests: ' + service_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.diagnostic = None
        self.diagnostic_time = 0.0
        if diagnostics_topic:
            from std_msgs.msg import String
            self.node.create_subscription(String, diagnostics_topic, self.on_diag, 10)
        self.nav_client = None

    def on_diag(self, msg):
        self.diagnostic = msg.data
        self.diagnostic_time = time.monotonic()

    def await_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and self.rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self.rclpy.spin_once(self.node, timeout_sec=min(.1, remaining))
        return future.done()

    def get_tf_xy(self, frame, source, wait_sec=5):
        from rclpy.time import Time
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline:
            try:
                trans = self.tf_buffer.lookup_transform(frame, source, Time())
                p = trans.transform.translation
                return [p.x, p.y]
            except Exception:
                self.rclpy.spin_once(self.node, timeout_sec=.15)
        raise RuntimeError('No TF transform %s <- %s; set sampling.center_xy manually.' % (frame, source))

    def get_tf_pose(self, frame, source, wait_sec=8):
        """Read observed TCP position AND orientation without sending commands."""
        from rclpy.time import Time
        deadline = time.monotonic() + wait_sec
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(frame, source, Time())
                t = transform.transform.translation
                q = transform.transform.rotation
                return {'header': {'frame_id': frame}, 'pose': {
                    'position': {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
                    'orientation': {'x': float(q.x), 'y': float(q.y),
                                    'z': float(q.z), 'w': float(q.w)}}}
            except Exception as exc:
                last_error = exc
                self.rclpy.spin_once(self.node, timeout_sec=.15)
        raise RuntimeError('No TF transform %s <- %s; check TCP frame names in '
                           'sampling.reference_tcp_frames. Last error: %s' %
                           (frame, source, last_error))

    def call(self, request_dict, timeout):
        req = self.service_type.Request()
        # Humble's set_message_fields may replace nested dictionary values with
        # ROS message instances (e.g. PoseStamped). Never pass the canonical
        # benchmark request directly: it is also written to cases.json later.
        self.set_fields(req, deepcopy(request_dict))
        t0 = time.monotonic()
        future = self.client.call_async(req)
        if not self.await_future(future, timeout):
            return None, time.monotonic() - t0, 'timeout'
        response = future.result()  # propagates raised service exception
        return self.message_to_dict(response), time.monotonic() - t0, ''

    def compute_nav_path(self, pose_dict, timeout):
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        if self.nav_client is None:
            self.nav_client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return 'unavailable', None, None
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        self.set_fields(goal.goal, pose_dict)
        goal.use_start = False  # Nav2 uses current robot pose; no motion requested
        future = self.nav_client.send_goal_async(goal)
        if not self.await_future(future, timeout):
            return 'timeout', None, None
        handle = future.result()
        if not handle or not handle.accepted:
            return 'rejected', None, None
        result_future = handle.get_result_async()
        if not self.await_future(result_future, timeout):
            handle.cancel_goal_async()
            return 'timeout', None, None
        completed = result_future.result()
        if completed.status != GoalStatus.STATUS_SUCCEEDED:
            return 'failed', None, None
        poses = completed.result.path.poses
        length = sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                b.pose.position.y-a.pose.position.y)
                     for a,b in zip(poses, poses[1:]))
        if not poses:
            return 'empty_path', None, None
        first = poses[0].pose.position
        last = poses[-1].pose.position
        straight = math.hypot(last.x-first.x, last.y-first.y)
        efficiency = min(1.0, straight / length) if length > 1e-8 else None
        return 'success', length, efficiency

    def shutdown(self):
        try:
            self.node.destroy_node()
        finally:
            self.rclpy.shutdown()


def json_default(value):
    """Convert incidental ROS 2 message objects into JSON-compatible values."""
    if hasattr(value, 'get_fields_and_field_types'):
        from rosidl_runtime_py.convert import message_to_ordereddict
        return message_to_ordereddict(value)
    raise TypeError('Object of type %s is not JSON serializable' %
                    type(value).__name__)


def write_csv(path, rows):
    if not rows:
        return
    cols = list(dict.fromkeys(k for row in rows for k in row))
    with open(path, 'w', newline='', encoding='utf8') as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        writer.writerows(rows)


def make_summary(rows, cfg, service_type):
    def summarize(xs):
        counts = Counter(row['status'] for row in xs)
        successes = counts['success']
        navs = [row for row in xs if row.get('nav_status') and row.get('nav_status') != 'not_checked']
        n_pass = sum(row.get('nav_status') == 'success' for row in navs)
        times = numeric_summary([row.get('time_s') for row in xs])
        return dict(total=len(xs), statuses=dict(counts), success_percent=round(100*successes/len(xs), 2) if xs else None,
                    latency_s=times,
                    nav_path_success_percent=round(100*n_pass/len(navs),2) if navs else None,
                    nav_attempts=len(navs),
                    path_efficiency_mean=statistics.mean([float(row['path_efficiency']) for row in navs
                                      if row.get('path_efficiency') is not None]) if any(row.get('path_efficiency') is not None for row in navs) else None,
                    selected_arms=dict(Counter(row.get('selected_arm') for row in xs if row.get('status')=='success' and row.get('selected_arm'))),
                    score=numeric_summary([row.get('score') for row in xs if row.get('status') == 'success']),
                    per_point_arm_assignments=dict(Counter(arm for row in xs if row.get('status') == 'success'
                        for arm in json.loads(row.get('point_arms') or '[]'))))
    categories = sorted(set(row['scenario'] for row in rows))
    return dict(generated_utc=datetime.now(timezone.utc).isoformat(),
                service=cfg['service_name'], service_type=service_type, seed=cfg.get('seed', 2026),
                all_requests=summarize(rows), scenarios={s:summarize([r for r in rows if r['scenario']==s]) for s in categories},
                caveats=['Only the configured custom selection service is called; no robot motion is commanded.',
                         'Nav2 path success is planning-only, not physical navigation success.',
                         'Unknown-result responses are not counted as selection successes.',
                         'MoveIt plan feasibility and base-pose robustness are not measured by this script.',
                         'Observed TCP reference targets are kinematically reachable at the present base but may fail path/collision/FOV validation.',
                         'Success percentage includes all attempted requests, including timeouts/errors.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description='MUR read-only base selector KPI benchmark')
    parser.add_argument('mode', choices=['discover', 'inspect', 'dry-run', 'run'])
    parser.add_argument('--config', default=None, help='YAML config (defaults to installed mur_test config)')
    parser.add_argument('--service', help='Service name override (especially useful for inspect)')
    parser.add_argument('--limit', type=int, default=0, help='Max cases to execute/preview (0=all)')
    parser.add_argument('--out', default='benchmark_results')
    parser.add_argument('--nav2', action='store_true', help='Optional planning-only Nav2 validation')
    parser.add_argument('--center', type=float, nargs=2, metavar=('MAP_X', 'MAP_Y'),
                        help='Override sampling centre XY in map metres (also enables dry-run with center_xy: null)')
    parser.add_argument('--save-raw', action='store_true', help='Save entire service responses in JSONL')
    parser.add_argument('--reference', action='store_true',
                        help='Use observed map-frame left/right TCP poses for 5 positive-control cases instead of random targets')
    parser.add_argument('--discovery-timeout', type=float, default=10.0,
                        help='Time to discover selection services (seconds, default: 10)')
    args = parser.parse_args(argv)
    config_file = args.config or default_config_path()
    cfg = read_config(config_file) if Path(config_file).exists() else {}
    if args.mode in ('run', 'dry-run') and not cfg:
        parser.error('Configuration missing: %s. Use --config PATH.' % config_file)
    if args.mode == 'discover':
        try:
            import rclpy
            from rclpy.node import Node
            rclpy.init()
            node = Node('mur_kpi_discovery')
            try:
                # ROS 2 graph discovery is asynchronous. A single 0.4 s spin can
                # miss an otherwise healthy selector, particularly on a loaded Jetson.
                timeout = max(0.0, args.discovery_timeout)
                deadline = time.monotonic() + timeout
                services = []
                names = []
                while True:
                    services = node.get_service_names_and_types()
                    # Ignore parameter services belonging to mur_base_selector:
                    # those may appear before the actual SelectBase service.
                    names = [
                        (n, t) for n, t in services
                        if not all(x.startswith('rcl_interfaces/srv/') for x in t)
                        and (
                            'selection' in n.lower()
                            or 'candidate' in n.lower()
                            or 'reach' in n.lower()
                            or any('SelectBase' in x or 'FindBase' in x for x in t)
                        )
                    ]
                    if names or time.monotonic() >= deadline:
                        break
                    rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
                print('Candidate selection services:')
                for name, types in names:
                    print('  %s: %s' % (name, ', '.join(types)))
                if not names:
                    print('  None found after %.1f seconds.' % timeout)
                    print('  ROS graph contained %d other service(s).' % len(services))
                    for name, types in services[:20]:
                        print('    %s: %s' % (name, ', '.join(types)))
                    if len(services) > 20:
                        print('    ... and %d more.' % (len(services) - 20))
                    print('  Compare: ros2 service list -t; ros2 node list; echo $ROS_DOMAIN_ID')
            finally:
                node.destroy_node()
                rclpy.shutdown()
            return 0
        except Exception as exc:
            print('Unable to discover ROS services: %s' % exc, file=sys.stderr)
            return 2
    service = args.service or cfg.get('service_name')
    if not service:
        parser.error('Set service_name in config or use --service')

    if args.mode == 'run' and not cfg.get('response_success_path'):
        print('Warning: success response field is not configured; results will not be counted as successful without a boolean.', file=sys.stderr)

    if args.reference and args.mode != 'run':
        parser.error('--reference is supported only in run mode because it requires live TF')

    if args.mode == 'dry-run':
        center = args.center if args.center is not None else cfg.get('sampling', {}).get('center_xy')
        if center is None:
            print('dry-run with center_xy: null requires --center X Y (no ROS accessed)', file=sys.stderr)
            return 2
        cases = sample_tasks(cfg, center)
        if args.limit:
            cases = cases[:args.limit]
        for case in cases:
            print(yaml.safe_dump({'scenario':case['scenario'], 'request':case['request']}, sort_keys=False))
        print('Generated %d request(s); no ROS contacted.' % len(cases))
        return 0

    try:
        ros = RosInterface(service, cfg.get('diagnostics_topic') if args.mode == 'run' else None)
    except Exception as e:
        print('ROS interface error: %s' % e, file=sys.stderr)
        print('Run inside the sourced ROS 2 Humble environment and verify service name/type. Try: ros2 run mur_test mur_kpi_benchmark discover', file=sys.stderr)
        return 2
    try:
        print('Service:', service, 'type:', ros.service_type_name)
        expected_type = cfg.get('expected_service_type')
        if args.mode == 'run' and expected_type and ros.service_type_name != expected_type:
            print('ERROR: expected %s but discovered %s' % (expected_type, ros.service_type_name), file=sys.stderr)
            return 2
        if args.mode == 'inspect':
            from rosidl_runtime_py.convert import message_to_yaml
            print('\n=== Request fields ===')
            print(ros.service_type.Request.get_fields_and_field_types())
            print('\n=== Request prototype ===')
            print(message_to_yaml(ros.service_type.Request()))
            print('\n=== Response fields ===')
            print(ros.service_type.Response.get_fields_and_field_types())
            print('\n=== Response prototype ===')
            print(message_to_yaml(ros.service_type.Response()))
            return 0

        center = args.center if args.center is not None else cfg.get('sampling', {}).get('center_xy')
        if center is None:
            center = ros.get_tf_xy(cfg.get('frame','map'), cfg.get('sampling',{}).get('center_tf_frame', 'base_link'))
        if args.reference:
            reference_frames = cfg.get('sampling', {}).get('reference_tcp_frames', {})
            sources = {
                'left': reference_frames.get('left', 'left_tcp'),
                'right': reference_frames.get('right', 'right_tcp'),
            }
            poses = {arm: ros.get_tf_pose(cfg.get('frame','map'), source)
                     for arm, source in sources.items()}
            cases = sample_reference_tasks(cfg, poses)
            print('Reference mode: using observed map-frame TCP poses from TF:', sources)
            print('These are kinematically reachable at the CURRENT base; navigation/path '
                  'and selector acceptance are NOT guaranteed.')
        else:
            cases = sample_tasks(cfg, center)
            print('Random mode: targets are NOT guaranteed reachable. '
                  'Do not interpret no_solution as a false negative without reference validation.')
        if args.limit:
            cases = cases[:args.limit]
        print('Benchmarking %d cases, sampling around (%.3f, %.3f) in %s' % (
            len(cases), center[0], center[1], cfg.get('frame','map')))
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        results = []
        response_dump = open(out/'raw_responses.jsonl','w',encoding='utf8') if args.save_raw else None
        try:
            for i, case in enumerate(cases, 1):
                row = dict(id=case['id'], scenario=case['scenario'], expected_feasible=case['expected_feasible'],
                           reference_type=case.get('reference_type',''),
                           npoints=case['npoints'], arm_constraint=case['arm_constraint'],
                           target_x=round(case['targets'][0]['x'], 5), target_y=round(case['targets'][0]['y'], 5),
                           target_z=case['targets'][0]['z'], status='', time_s=None, selected_arm='',
                           selected_x=None, selected_y=None, selected_yaw_deg=None,
                           nav_status='not_checked', path_length_m=None, path_efficiency=None,
                           diagnostics='', message='', request_id=None, score=None,
                           valid_for_seconds=None, planning_groups='', tcp_frames='',
                           point_arms='', diagnostics_json='')
                started = time.monotonic()
                try:
                    response, dt, error = ros.call(case['request'], float(cfg.get('service_timeout_s', 40)))
                    row['time_s'] = round(dt, 4)
                    if error:
                        row['status'] = error
                    else:
                        status, pose, arm, message = response_info(response, cfg)
                        row['status'], row['selected_arm'], row['message'] = status, arm, message
                        row['request_id'] = response.get('request_id')
                        row['score'] = response.get('score') if status == 'success' else None
                        row['valid_for_seconds'] = response.get('valid_for_seconds')
                        row['planning_groups'] = json.dumps(response.get('planning_groups', []))
                        row['tcp_frames'] = json.dumps(response.get('tcp_frames', []))
                        row['point_arms'] = json.dumps(response.get('point_arms', []))
                        # JSON emitted by the selector is a request-specific diagnostic;
                        # store it separately from the legacy diagnostics topic.
                        diag_string = str(response.get('diagnostics_json', '') or '')
                        if diag_string:
                            try:
                                row['diagnostics_json'] = json.dumps(json.loads(diag_string), separators=(',', ':'))
                            except json.JSONDecodeError:
                                row['diagnostics_json'] = diag_string
                        x, y, yaw = pose_geometry(pose)
                        row['selected_x'], row['selected_y'], row['selected_yaw_deg'] = x, y, yaw
                        if response_dump:
                            response_dump.write(json.dumps({'id':case['id'], 'response':response}, default=str)+'\n')
                        if args.nav2 and status=='success':
                            if pose and pose['header'].get('frame_id', cfg.get('frame','map')) == cfg.get('frame','map'):
                                nav_status, plen, eff = ros.compute_nav_path(pose, float(cfg.get('nav_timeout_s', 20)))
                                row['nav_status'], row['path_length_m'], row['path_efficiency'] = nav_status, plen, eff
                            else:
                                row['nav_status'] = 'pose_unavailable_or_nonmap_frame'
                except KeyboardInterrupt:
                    print('\nInterrupted; writing completed results.')
                    break
                except Exception as e:
                    row['status'] = 'error'
                    row['message'] = repr(e)
                if ros.diagnostic_time >= started and ros.diagnostic:
                    row['diagnostics'] = ros.diagnostic[:5000]
                results.append(row)
                write_csv(out/'results.csv', results)  # preserve partial results after every case
                print('[%d/%d] %-26s %-16s %7s s | %s' % (
                    i,len(cases),case['scenario'],row['status'],row['time_s'],row['nav_status']), flush=True)
                if row['status'] != 'success':
                    if row['message']:
                        print('    Selector message: %s' % row['message'][:1000], flush=True)
                    if row['diagnostics_json']:
                        print('    Selector diagnostics: %s' %
                              compact_diagnostic(row['diagnostics_json']), flush=True)
                    elif row['diagnostics']:
                        print('    Topic diagnostics (not request-correlated): %s' %
                              compact_diagnostic(row['diagnostics'], 600), flush=True)
                if row['status'] == 'timeout':
                    print('Stopping: timed-out service request may still be running on the server.')
                    break
        finally:
            if response_dump:
                response_dump.close()
        summary = make_summary(results, cfg, ros.service_type_name)
        (out/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
        # Belt-and-braces conversion for any ROS messages introduced by future
        # service/schema changes. Requests should still be plain dictionaries
        # because call() uses a deep copy above.
        (out/'cases.json').write_text(
            json.dumps(cases, indent=2, default=json_default), encoding='utf8')
        print('\nSummary:', json.dumps(summary['all_requests'], indent=2))
        print('Saved:', out/'results.csv', 'and', out/'summary.json')
        return 0
    finally:
        ros.shutdown()


if __name__ == '__main__':
    sys.exit(main())
