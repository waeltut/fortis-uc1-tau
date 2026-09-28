#!/usr/bin/env python3
"""Collision-checked sampled TCP reachability for Lichtblick (ROS 2 / MoveIt).
No motion commands. Other joints stay at a captured current configuration.
Uses MoveIt's live scene and allowed-collision matrix. Keep robot/scene still
while calculating; restart after changing either. This is endpoint reachability,
not path feasibility, and unsampled positions are not proven unreachable.
"""
import copy
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import String
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity, GetPositionFK


class WorkspaceViewer(Node):
    def __init__(self):
        super().__init__('workspace_viewer')
        defaults = {'samples': 3000, 'cross_margin': 0.20, 'voxel_size': 0.015,
                    'reference_frame': 'chest', 'left_tip': 'left_tcp', 'right_tip': 'right_tcp',
                    'description_topic': '/robot_description', 'joint_states_topic': '/joint_states',
                    'urdf_file': '', 'scene_service': '/get_planning_scene',
                    'validity_service': '/check_state_validity', 'fk_service': '/compute_fk'}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.xml = None
        self.joints = {}
        self.points = {'left': [], 'right': []}
        self.baseline = None
        self.stale = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pubs = {s: self.create_publisher(PointCloud2, '/workspace/'+s, qos) for s in self.points}
        self.desc_sub = self.create_subscription(String, self.p('description_topic'), self.description, qos)
        self.js_sub = self.create_subscription(JointState, self.p('joint_states_topic'), self.joint_state, 10)
        self.scene = self.create_client(GetPlanningScene, self.p('scene_service'))
        self.validity = self.create_client(GetStateValidity, self.p('validity_service'))
        self.fk = self.create_client(GetPositionFK, self.p('fk_service'))
        self.timer = self.create_timer(2.0, self.publish)
        if self.p('urdf_file'):
            self.xml = Path(self.p('urdf_file')).read_text()

    def p(self, key):
        return self.get_parameter(key).value

    def description(self, msg):
        self.xml = msg.data

    def joint_state(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        if self.baseline is not None and not self.stale:
            if any(abs(self.joints.get(k, v)-v) > 0.02 for k, v in self.baseline.items()):
                self.stale = True
                self.get_logger().error('Robot configuration changed. Clearing clouds; restart to regenerate.')
                self.points = {'left': [], 'right': []}
                self.publish()

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        if future.exception():
            raise future.exception()
        return future.result()

    def chain_limits(self, robot, side):
        parents = {j.find('child').get('link'): j for j in robot.findall('joint')}
        cursor = self.p(side+'_tip')
        links = {l.get('name') for l in robot.findall('link')}
        if cursor not in links:
            raise ValueError(f'Unknown TCP link: {cursor}')
        joints = []
        while cursor in parents:
            j = parents[cursor]
            name, kind = j.get('name'), j.get('type')
            if kind != 'fixed' and name.startswith(side+'_'):
                if kind not in ('revolute', 'continuous', 'prismatic') or j.find('mimic') is not None:
                    raise ValueError(f'Unsupported sampled joint: {name}')
                if kind == 'continuous':
                    lo, hi = -np.pi, np.pi
                else:
                    lim = j.find('limit')
                    lo, hi = float(lim.get('lower')), float(lim.get('upper'))
                    safety = j.find('safety_controller')
                    if safety is not None:
                        lo = max(lo, float(safety.get('soft_lower_limit', lo)))
                        hi = min(hi, float(safety.get('soft_upper_limit', hi)))
                if lo > hi or not np.isfinite([lo, hi]).all():
                    raise ValueError(f'Invalid limits: {name}')
                joints.append((name, lo, hi))
            cursor = j.find('parent').get('link')
        if len(joints) != 6:
            raise ValueError(f'Expected 6 {side}_ arm joints on TCP chain; found {len(joints)}')
        return joints

    def generate(self):
        self.get_logger().info('Waiting for robot_description and joint_states...')
        deadline = time.monotonic()+30
        while rclpy.ok() and (self.xml is None or not self.joints):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() > deadline:
                raise RuntimeError('Missing robot_description or joint_states; check topic parameters.')
        for client in (self.scene, self.validity, self.fk):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f'MoveIt service unavailable: {client.srv_name}')
        robot = ET.fromstring(self.xml)
        chains = {s: self.chain_limits(robot, s) for s in self.points}
        missing = [n for chain in chains.values() for n, _, _ in chain if n not in self.joints]
        if missing:
            raise RuntimeError(f'Missing current arm joint states: {missing}')
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ROBOT_STATE |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        state = self.call(self.scene, req).scene.robot_state
        snapshot = dict(zip(state.joint_state.name, state.joint_state.position))
        snapshot.update(self.joints)
        state.joint_state.name = list(snapshot)
        state.joint_state.position = list(snapshot.values())
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.is_diff = False
        self.baseline = dict(self.joints)
        index = {n: i for i, n in enumerate(state.joint_state.name)}
        count, margin = int(self.p('samples')), float(self.p('cross_margin'))
        if count < 1 or margin < 0:
            raise ValueError('samples must be positive and cross_margin nonnegative')
        rng = np.random.default_rng(42)
        self.get_logger().info(f'Snapshot captured. Frame={self.p("reference_frame")}; crossing margin={margin} m. Keep robot and scene still.')
        for side, chain in chains.items():
            for i in range(count):
                if self.stale or not rclpy.ok():
                    return
                candidate = copy.deepcopy(state)
                for name, lo, hi in chain:
                    candidate.joint_state.position[index[name]] = float(rng.uniform(lo, hi))
                check = GetStateValidity.Request()
                check.robot_state = candidate
                check.group_name = ''  # Whole-robot collision check, including the parked arm.
                if self.call(self.validity, check).valid:
                    fk = GetPositionFK.Request()
                    fk.header.frame_id = self.p('reference_frame')
                    fk.fk_link_names = [self.p(side+'_tip')]
                    fk.robot_state = candidate
                    result = self.call(self.fk, fk)
                    if result.error_code.val != 1 or len(result.pose_stamped) != 1:
                        raise RuntimeError(f'FK failed ({result.error_code.val}); check reference_frame and TCP link.')
                    pose = result.pose_stamped[0]
                    if pose.header.frame_id.lstrip('/') != self.p('reference_frame').lstrip('/'):
                        raise RuntimeError('FK returned an unexpected frame')
                    p = pose.pose.position
                    if (side == 'left' and p.y >= -margin) or (side == 'right' and p.y <= margin):
                        if not self.stale:
                            self.points[side].append((p.x, p.y, p.z))
                if (i+1) % 250 == 0:
                    self.get_logger().info(f'{side}: checked {i+1}/{count}; accepted {len(self.points[side])}')
                    self.publish()
            self.publish()
        self.get_logger().info('Done. Republishing clouds. Restart after changing robot pose or scene.')

    def publish(self):
        for side, values in self.points.items():
            pts = np.asarray(values, dtype='<f4').reshape(-1, 3)
            voxel = float(self.p('voxel_size'))
            if len(pts) and voxel > 0:
                _, indices = np.unique(np.floor(pts/voxel).astype(np.int64), axis=0, return_index=True)
                pts = pts[indices]
            cloud = PointCloud2()
            cloud.header.frame_id = self.p('reference_frame')
            cloud.header.stamp = self.get_clock().now().to_msg()
            cloud.height, cloud.width = 1, len(pts)
            cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, n in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian, cloud.is_dense = False, True
            cloud.point_step, cloud.row_step = 12, 12*len(pts)
            cloud.data = pts.tobytes()
            self.pubs[side].publish(cloud)


def main():
    rclpy.init()
    node = WorkspaceViewer()
    try:
        node.generate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
