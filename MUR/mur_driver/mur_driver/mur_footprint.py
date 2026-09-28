#!/usr/bin/env python3
"""Conservative URDF collision-envelope footprint for ROS 2 Humble."""
import itertools
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def vector(text, default):
    out = np.asarray([float(v) for v in text.split()] if text else default, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError('Expected three finite coordinates')
    return out


def rotation_rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def corners(low, high):
    return np.array(list(itertools.product(*zip(low, high))), dtype=float)


def mesh_path(uri):
    if uri.startswith('package://'):
        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    if uri.startswith('file://'):
        return Path(uri[len('file://'):])
    path = Path(uri)
    if not path.is_absolute():
        raise ValueError(f'Mesh must have package://, file:// or absolute path: {uri}')
    return path


def collision_points(collision):
    geometry = collision.find('geometry')
    if geometry is None or len(geometry) != 1:
        raise ValueError('Collision requires exactly one geometry')
    shape = geometry[0]
    if shape.tag == 'box':
        half = vector(shape.get('size'), []) / 2.0
    elif shape.tag == 'cylinder':
        half = np.array([float(shape.attrib['radius'])]*2 + [float(shape.attrib['length'])/2])
    elif shape.tag == 'sphere':
        half = np.full(3, float(shape.attrib['radius']))
    elif shape.tag == 'mesh':
        import trimesh
        mesh = trimesh.load(str(mesh_path(shape.attrib['filename'])), force='mesh', process=False)
        vertices = np.asarray(mesh.vertices, dtype=float) * vector(shape.get('scale'), [1, 1, 1])
        if vertices.size == 0 or not np.isfinite(vertices).all():
            raise ValueError('Mesh has no finite vertices')
        points = corners(vertices.min(axis=0), vertices.max(axis=0))
    else:
        raise ValueError(f'Unsupported collision geometry: {shape.tag}')
    if shape.tag != 'mesh':
        if not np.isfinite(half).all() or (half <= 0).any():
            raise ValueError('Collision dimensions must be positive and finite')
        points = corners(-half, half)
    origin = collision.find('origin')
    if origin is not None:
        points = points @ rotation_rpy(vector(origin.get('rpy'), [0, 0, 0])).T
        points += vector(origin.get('xyz'), [0, 0, 0])
    return points


def parse_description(xml, required_links):
    root = ET.fromstring(xml)
    links = {link.attrib['name']: link for link in root.findall('link')}
    missing = set(required_links) - set(links)
    if missing:
        raise ValueError(f'Description missing required links: {sorted(missing)}')
    models = {}
    for name, link in links.items():
        parts = [collision_points(c) for c in link.findall('collision')]
        if parts:
            models[name] = np.concatenate(parts)
    if not models:
        raise ValueError('No collision geometry in description')
    for name in required_links:
        if name not in models:
            raise ValueError(f'Required link has no collision geometry: {name}')
    return models


def convex_hull(points):
    pts = sorted(set(map(tuple, np.asarray(points, dtype=float))))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1])
    if len(hull) < 3 or not np.isfinite(hull).all():
        raise ValueError('Degenerate footprint')
    return hull


def project_points(points, transform):
    t, q = transform.translation, transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = quaternion / norm
    rot = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return (points @ rot.T + [t.x, t.y, t.z])[:, :2]


class MurFootprint(Node):
    def __init__(self):
        super().__init__('mur_footprint')
        defaults = {
            'base_frame': 'base_link',
            'robot_description_topic': '/robot_description',
            'robot_description_file': '',
            'required_links': ['mur', 'left_upper_arm_link', 'right_upper_arm_link'],
            'publish_rate': 10.0,
            'max_tf_age': 0.5,
            'padding': 0.03,
            'local_footprint_topic': '/local_costmap/footprint',
            'global_footprint_topic': '/global_costmap/footprint',
            'visualization_topic': '/mur/footprint',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.base = self.get_parameter('base_frame').value
        self.padding = float(self.get_parameter('padding').value)
        self.max_age = float(self.get_parameter('max_tf_age').value)
        rate = float(self.get_parameter('publish_rate').value)
        if not all(math.isfinite(v) for v in (rate, self.max_age, self.padding)) or rate <= 0 or self.max_age <= 0 or self.padding < 0:
            raise ValueError('Rate and TF age must be positive; padding must be nonnegative')
        self.models = None
        self.xml = None
        self.last_warning = -math.inf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.outputs = [self.create_publisher(Polygon, self.get_parameter(p).value, 1)
                        for p in ('local_footprint_topic', 'global_footprint_topic')]
        self.visual = self.create_publisher(PolygonStamped, self.get_parameter('visualization_topic').value, 1)
        self.valid = self.create_publisher(Bool, '/mur/footprint_valid', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        path = self.get_parameter('robot_description_file').value
        if path:
            self.load_description(Path(path).read_text())
        else:
            self.description_sub = self.create_subscription(
                String, self.get_parameter('robot_description_topic').value,
                lambda msg: self.load_description(msg.data), qos)
        self.timer = self.create_timer(1.0/rate, self.update)

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning > 5.0:
            self.get_logger().warning(message)
            self.last_warning = now

    def load_description(self, xml):
        if xml == self.xml:
            return
        # Never continue publishing an old model after a rejected replacement.
        self.models = None
        try:
            models = parse_description(xml, self.get_parameter('required_links').value)
            self.models, self.xml = models, xml
            self.get_logger().info(f'Loaded collision geometry for {len(models)} links; footprint frame: {self.base}')
        except Exception as exc:
            self.xml = None
            self.get_logger().error(f'Cannot load footprint geometry: {exc}')

    def update(self):
        try:
            if self.models is None:
                raise ValueError('Waiting for a valid robot description containing base and both arms')
            latest = {}
            stamps = []
            for link in self.models:
                tf = self.buffer.lookup_transform(self.base, link, Time())
                latest[link] = tf
                stamp = Time.from_msg(tf.header.stamp).nanoseconds
                if stamp:  # A wholly static TF chain has stamp zero.
                    stamps.append(stamp)
            now = self.get_clock().now()
            common = min(stamps) if stamps else now.nanoseconds
            age = (now.nanoseconds-common)/1e9
            if age > self.max_age or age < -0.1:
                raise ValueError(f'TF data age {age:.3f} s outside allowed range')
            # Sample all moving links at a common time, avoiding mixed-time geometry.
            sample = Time(nanoseconds=common, clock_type=now.clock_type)
            projected = []
            for link, points in self.models.items():
                tf = self.buffer.lookup_transform(self.base, link, sample) if stamps else latest[link]
                projected.append(project_points(points, tf.transform))
            hull = convex_hull(np.concatenate(projected))
            # Minkowski sum with a square: encloses a circular margin of this radius.
            if self.padding:
                offsets = np.array(list(itertools.product([-self.padding, self.padding], repeat=2)))
                hull = convex_hull((hull[:, None, :] + offsets).reshape(-1, 2))
            polygon = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in hull])
            for publisher in self.outputs:
                publisher.publish(polygon)
            stamped = PolygonStamped()
            stamped.header.frame_id = self.base
            stamped.header.stamp = sample.to_msg()
            stamped.polygon = polygon
            self.visual.publish(stamped)
            self.valid.publish(Bool(data=True))
        except Exception as exc:
            self.valid.publish(Bool(data=False))
            self.warn(f'Footprint not updated: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MurFootprint()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
