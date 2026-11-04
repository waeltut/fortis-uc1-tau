"""Explicitly triggered diagnostic node. Never publishes actionable selections."""
import copy
import colorsys
import json
import math
import threading
import time
from collections import Counter

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped, PolygonStamped
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState
from visualization_msgs.msg import Marker, MarkerArray

from .navigation import DEFAULTS as NAV_DEFAULTS, BaseSelector, Candidate, TfReceiver, SelectionError, Cancelled, CheckingBudgetExhausted, quaternion, pose_msg
from .volume_node import DEFAULTS as VOLUME_DEFAULTS, VolumeSelector
from .cache_startup import load_selector_volume
from .robot_config import monitored_joints
from .volume import TravelField
from .core import transform_points, path_length
from .score_grid import COMPONENTS, ScoreRaster, components, heading_matches

DIAGNOSTICS = {
    'mode': 'fast', 'grid_resolution': 0.10, 'score_max': 10.0,
    'max_cells': 1000000, 'max_markers': 1500,
    'calculation_budget': 60.0, 'travel_budget': 15.0,
    'max_detailed_candidates': 200, 'heading_filter': False,
    'heading_deg': 0.0, 'heading_tolerance_deg': 7.5,
}
PREFIX = '/base_selection/score_map'


class ScoreVisualizer(VolumeSelector):
    # Reuse computation and ROS-client helpers, but deliberately DO NOT call
    # either selector constructor: no selection publishers/services/timers exist.
    def __init__(self):
        Node.__init__(self, 'mur_score_visualizer')
        self.cfg = {}
        for key, value in NAV_DEFAULTS.items():
            self.declare_parameter(key, value)
            self.cfg[key] = self.get_parameter(key).value
        self.validate_config()
        for key, value in VOLUME_DEFAULTS.items():
            self.declare_parameter(key, value)
            self.cfg[key] = self.get_parameter(key).value
        self.diag = {}
        for key, value in DIAGNOSTICS.items():
            self.declare_parameter('diagnostic.'+key, value)
            self.diag[key] = self.get_parameter('diagnostic.'+key).value
        self.volume = load_selector_volume(self.cfg['volume_file'])
        if self.cfg['arm_mode'] not in ('either', 'both', *self.volume.arm_names):
            raise ValueError('Unknown arm_mode')
        required = monitored_joints(self.volume.meta)
        if self.cfg['arm_joints'] == ['auto']: self.cfg['arm_joints'] = required
        elif not set(required).issubset(self.cfg['arm_joints']):
            raise ValueError('arm_joints must include all cached joints')
        for name in self.volume.arm_names:
            for suffix, value in (('_preferred_distance', .65), ('_facing_min_deg', -60.), ('_facing_max_deg', 60.)):
                key = name+suffix
                self.declare_parameter(key, value)
                self.cfg[key] = self.get_parameter(key).value
        self.validate_volume()
        self.validate_diagnostics()
        self.volume.configure_profiles(self.cfg['orientation_profiles_deg'])
        if self.volume.meta['context']['base_frame'] != self.cfg['base_frame']:
            raise ValueError('Cache base_frame mismatch')
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='Restart diagnostic node to change configuration'))
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.busy = False
        self.worker = None
        self.last_target = None
        self.joints = {}
        self.footprint = None
        self.scan_seen = None
        self.sequence = 0
        self.inflight_plan = None
        self.plan_pending = False
        self.group = ReentrantCallbackGroup()
        self.tf_receiver = TfReceiver(self)
        self.tf = self.tf_receiver.buffer
        self.maps = [self.create_client(GetCostmap, self.cfg[k], callback_group=self.group)
                     for k in ('global_costmap_service', 'local_costmap_service')]
        self.lifecycle = [self.create_client(GetState, name, callback_group=self.group)
                          for name in self.cfg['lifecycle_services']]
        self.planner = ActionClient(self, ComputePathToPose, self.cfg['planner_action'], callback_group=self.group)
        source = self.volume.meta['context']['source_node'].rstrip('/')
        self.model_client = self.create_client(GetParameters, source+'/get_parameters', callback_group=self.group)
        sensors = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self.on_joints, sensors)
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, sensors)
        self.create_subscription(PolygonStamped, self.cfg['footprint_topic'], self.on_footprint, sensors)
        self.create_subscription(PoseStamped, '/base_selection/target', self.on_target, 10)
        self.create_subscription(PoseStamped, '/base_selection/score_target', self.on_target, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pubs = {mode: {key: self.create_publisher(OccupancyGrid, PREFIX+'/'+mode+'/'+key, qos)
                                for key in (*COMPONENTS, 'state')} for mode in ('fast', 'detailed')}
        self.marker_pubs = {mode: self.create_publisher(MarkerArray, PREFIX+'/'+mode+'/winners', qos)
                            for mode in ('fast', 'detailed')}
        self.status_pub = self.create_publisher(String, PREFIX+'/status', qos)
        self.metadata_pub = self.create_publisher(String, PREFIX+'/metadata', qos)
        self.create_service(Trigger, PREFIX, self.trigger)
        self.create_service(Trigger, PREFIX+'/cancel', self.cancel_diagnostic)
        self.status('idle', 'Diagnostic node idle. Publish a target, then call '+PREFIX)

    def validate_diagnostics(self):
        d = self.diag
        if d['mode'] not in ('fast', 'detailed'): raise ValueError('diagnostic.mode must be fast or detailed')
        for k in ('grid_resolution', 'score_max', 'calculation_budget', 'travel_budget'):
            if not math.isfinite(d[k]) or d[k] <= 0: raise ValueError('Invalid diagnostic.'+k)
        for k in ('max_cells', 'max_markers', 'max_detailed_candidates'):
            if not isinstance(d[k], int) or d[k] < 1: raise ValueError('Invalid diagnostic.'+k)
        if not math.isfinite(d['heading_deg']) or not 0 <= d['heading_tolerance_deg'] <= 180:
            raise ValueError('Invalid heading filter')

    def on_target(self, msg):
        try:
            if not msg.header.frame_id: raise ValueError('Target requires frame_id')
            quaternion(msg.pose.orientation)
            p = msg.pose.position
            if not np.isfinite([p.x, p.y, p.z]).all(): raise ValueError('Nonfinite target')
            with self.lock: self.last_target = copy.deepcopy(msg)
            self.get_logger().info('Diagnostic target stored; calculation remains idle until triggered.')
        except (ValueError, SelectionError) as error:
            self.get_logger().warning(str(error))

    def trigger(self, request, response):
        with self.lock:
            if self.busy or self.plan_pending or (self.inflight_plan is not None and not self.inflight_plan.done()):
                response.success = False; response.message = 'Diagnostic calculation/planner request still busy'
            elif self.last_target is None:
                response.success = False; response.message = 'Publish /base_selection/score_target or /base_selection/target first'
            else:
                self.sequence += 1
                self.stop.clear(); self.busy = True
                target = copy.deepcopy(self.last_target)
                self.worker = threading.Thread(target=self.calculate, args=(target,), daemon=True)
                self.worker.start()
                response.success = True; response.message = f'Calculation {self.sequence} started; watch {PREFIX}/status'
        return response

    def cancel_diagnostic(self, request, response):
        with self.lock: self.stop.set()
        response.success = True; response.message = 'Diagnostic cancellation requested'
        return response

    def clear_outputs(self, stamp):
        raster = ScoreRaster([], self.diag['grid_resolution'], self.diag['max_cells'])
        for mode in self.grid_pubs: self.publish_raster(mode, raster, [], stamp)
        self.metadata_pub.publish(String(data=json.dumps({'request_id': self.sequence, 'state': 'calculating', 'valid_for_navigation': False})))

    def publish_raster(self, mode, raster, records, stamp):
        layers, winners, ranges = raster.render(records, self.diag['score_max'])
        for key, data in layers.items():
            msg = OccupancyGrid()
            msg.header.frame_id = self.cfg['fixed_frame']; msg.header.stamp = stamp
            msg.info.map_load_time = stamp
            msg.info.resolution = float(raster.resolution)
            msg.info.width = raster.width; msg.info.height = raster.height
            msg.info.origin.position.x = float(raster.origin[0]); msg.info.origin.position.y = float(raster.origin[1])
            msg.info.origin.orientation.w = 1.0
            msg.data = data.tolist()
            self.grid_pubs[mode][key].publish(msg)
        markers = [Marker(action=Marker.DELETEALL)]
        ordered = sorted(winners.items())
        stride = max(1, math.ceil(len(ordered)/self.diag['max_markers']))
        names = [*self.volume.arm_names, 'both']
        for cell, record in ordered[::stride]:
            m = Marker(); m.header.frame_id = self.cfg['fixed_frame']; m.header.stamp = stamp
            m.ns = record['arm']; m.id = cell; m.type = Marker.ARROW; m.action = Marker.ADD
            m.pose = pose_msg(record['pose'], self.cfg['fixed_frame']).pose
            m.pose.position.z = .06
            m.scale.x = max(.08, raster.resolution*.8); m.scale.y = .015; m.scale.z = .025
            rgb = colorsys.hsv_to_rgb(names.index(record['arm'])/len(names), .8, 1.)
            m.color.r, m.color.g, m.color.b = rgb; m.color.a = .9
            markers.append(m)
        self.marker_pubs[mode].publish(MarkerArray(markers=markers))
        return {'scored_cells': len(winners), 'ranges': ranges,
                'states': dict(Counter(r['state'] for r in records)), 'displayed_arrows': len(markers)-1}

    def calculate(self, target):
        started = time.monotonic()
        deadline = started+self.diag['calculation_budget']
        stamp = self.get_clock().now().to_msg()
        try:
            self.clear_outputs(stamp)
            self.status('calculating', 'Acquiring diagnostic target, robot state and costmap snapshot')
            frozen = self.transform_target(target, self.cfg['fixed_frame'])
            self.check_model(deadline); self.cache_context()
            start, joints = self.robot_pose(), self.joint_snapshot()
            maps = self.costmaps(deadline)
            footprint = self.footprint_snapshot() if self.diag['mode'] == 'detailed' else None
            stamp = self.get_clock().now().to_msg()  # snapshot acquisition time, never refreshed for display
            base = self.lookup(self.cfg['fixed_frame'], self.cfg['base_frame']).transform
            p = frozen.pose.position
            goal = [p.x, p.y, p.z, *quaternion(frozen.pose.orientation)]
            def checkpoint(): self.remaining(deadline, self.diag['calculation_budget'])
            rows = self.volume.query(goal, base.translation.z, self.cfg['yaw_bins'],
                self.cfg['minimum_orientation_coverage'], self.cfg['max_height_error'],
                self.cfg['max_orientation_error_deg'], self.cfg['front_fov_deg'],
                self.cfg['front_yaw_offset_deg'], self.cfg['arm_mode'], checkpoint)
            rows = [r for r in rows if heading_matches(r[0][2], self.diag['heading_filter'],
                    self.diag['heading_deg'], self.diag['heading_tolerance_deg'])]
            raster = ScoreRaster(rows, self.diag['grid_resolution'], self.diag['max_cells'])
            records = []
            truncated = False
            if rows:
                grid, tf = maps[0]
                field = TravelField(grid, transform_points([start[:2]], tf)[0],
                    self.cfg['travel_inflation_weight'], self.cfg['allow_unknown'],
                    min(deadline, time.monotonic()+self.diag['travel_budget']), self.checkpoint)
                truncated = field.truncated
                xy = transform_points([r[0][:2] for r in rows], tf)
                travel = field.lookup(xy); cells = field.cells(xy)
                for i, (row, cost) in enumerate(zip(rows, travel)):
                    if i % 256 == 0: checkpoint()
                    pose, arm, bearing = row[:3]
                    record = dict(pose=pose, arm=arm, bearing=bearing, state='no_estimate', scores=None)
                    if math.isfinite(cost):
                        x, y = cells[i]
                        centre_cost = min(int(grid.data[y, x]), 253)/253
                        record.update(state='estimated', scores=components(pose, arm, bearing, goal, float(cost), centre_cost, self.cfg))
                    records.append(record)
            self.guard_motion(start, joints)
            fast_summary = self.publish_raster('fast', raster, records, stamp)
            detailed_summary = None
            checked = paths = 0
            partial = False
            if self.diag['mode'] == 'detailed':
                detailed = [dict(r, state='unchecked', scores=None) for r in records]
                # Check promising candidates first, then remaining cache candidates.
                # No travel estimate is not proof of unreachability.
                order = sorted(range(len(records)), key=lambda i: records[i]['scores']['total'] if records[i]['scores'] else math.inf)
                if not self.planner.wait_for_server(timeout_sec=self.remaining(deadline, self.cfg['service_timeout'])):
                    raise SelectionError('Nav2 planner unavailable')
                for i in order:
                    if checked >= self.diag['max_detailed_candidates'] or time.monotonic() >= deadline:
                        partial = True; break
                    self.guard_motion(start, joints)
                    r = detailed[i]; checked += 1
                    ok, reason, destination = self.polygon_check(transform_points(footprint, r['pose']), maps)
                    if not ok:
                        r['state'] = 'destination_'+reason; continue
                    c = Candidate(i, r['pose'], r['arm'])
                    try:
                        paths += 1
                        poses, reason = self.compute_path(c, start, deadline)
                        if poses is None:
                            r['state'] = 'planner_'+reason; continue
                        ok, reason, path_cost = self.check_path(poses, footprint, maps, deadline)
                    except Cancelled: raise
                    except (CheckingBudgetExhausted, SelectionError):
                        # Timeout/transport errors do not establish unreachability.
                        partial = True
                        break
                    if not ok:
                        r['state'] = 'path_'+reason; continue
                    travel_cost = path_length(poses)*(1+self.cfg['travel_inflation_weight']*path_cost)
                    r.update(state='path_checked', scores=components(r['pose'], r['arm'], r['bearing'], goal, travel_cost, destination, self.cfg))
                self.guard_motion(start, joints)
                detailed_summary = self.publish_raster('detailed', raster, detailed, stamp)
            metadata = {
                'request_id': self.sequence, 'state': 'complete', 'valid_for_navigation': False,
                'frame': self.cfg['fixed_frame'], 'snapshot_stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                'target': goal, 'base_start': list(start), 'cache_id': self.volume.meta['cache_id'],
                'mode': self.diag['mode'], 'candidate_count': len(rows), 'travel_field_truncated': truncated,
                'detailed_partial': partial, 'destination_checks': checked, 'path_requests': paths,
                'fast': fast_summary, 'detailed': detailed_summary,
                'grid': {'origin': raster.origin, 'width': raster.width, 'height': raster.height, 'resolution': raster.resolution},
                'score_max': self.diag['score_max'], 'component_values': 'weighted contributions of minimum-total winner',
                'arm_order': self.volume.arm_names, 'selector_configuration': self.cfg, 'diagnostic_configuration': self.diag,
                'elapsed_seconds': time.monotonic()-started,
                'note': 'Historical diagnostic snapshot, not a navigation goal. Detailed Nav2 paths are planned against live Nav2 maps and checked against the captured maps; no final fresh-map validation.'}
            with self.lock:
                self.checkpoint()
                self.metadata_pub.publish(String(data=json.dumps(metadata, allow_nan=False)))
                self.status('complete', f'Score maps ready: {len(rows)} sampled candidates; detailed_partial={partial}',
                            travel_field_truncated=truncated)
        except Exception as error:
            if rclpy.ok():
                self.clear_outputs(stamp)
                self.metadata_pub.publish(String(data=json.dumps({'request_id': self.sequence, 'state': 'failed', 'message': str(error), 'valid_for_navigation': False})))
                self.status('cancelled' if isinstance(error, Cancelled) else 'failed', str(error))
        finally:
            with self.lock: self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = ScoreVisualizer(); executor.add_node(node); executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.stop.set()
        if rclpy.ok(): rclpy.shutdown()
        if node:
            if node.worker: node.worker.join()
            node.tf_receiver.close(); node.destroy_node()
        executor.shutdown(timeout_sec=2.)
