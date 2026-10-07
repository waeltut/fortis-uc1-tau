"""ROS-independent geometry. All costmap values are RAW Nav2 uint8 costs.

Polygon/cell overlap uses the separating axis theorem, including cell edges
and the polygon interior. Concave input footprints are conservatively hulled.
"""
from dataclasses import dataclass
import math
import numpy as np


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def hull(points):
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or not np.isfinite(pts).all():
        raise ValueError('Footprint must contain finite XY points')
    pts = sorted(set(map(tuple, pts)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out
    out = np.array(half(pts)[:-1] + half(pts[::-1])[:-1])
    if len(out) < 3:
        raise ValueError('Degenerate footprint')
    return out


def padded(poly, margin):
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('Invalid footprint margin')
    if margin == 0:
        return hull(poly)
    # Circumscribed regular polygon contains the requested circular padding.
    a = np.arange(16)*2*math.pi/16
    offsets = np.column_stack((np.cos(a), np.sin(a)))*margin/math.cos(math.pi/16)
    return hull((np.asarray(poly)[:, None, :] + offsets).reshape(-1, 2))


def transform_points(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + [x, y]


def compose(a, b):
    xy = transform_points([[b[0], b[1]]], a)[0]
    return float(xy[0]), float(xy[1]), angle(a[2]+b[2])


@dataclass
class Check:
    valid: bool
    reason: str = ''
    cost: float = 0.0
    covered: bool = True


class Grid:
    def __init__(self, data, width, height, resolution, origin=(0., 0., 0.)):
        if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid costmap dimensions/resolution')
        raw = np.asarray(data)
        if raw.size != width*height or not np.isfinite(raw).all() or (raw < 0).any() or (raw > 255).any():
            raise ValueError('Expected complete RAW Nav2 costmap, values 0..255')
        if not np.isfinite(origin).all():
            raise ValueError('Invalid costmap origin')
        self.data = raw.astype(np.uint8).reshape(height, width)
        self.width, self.height, self.resolution, self.origin = width, height, resolution, origin

    def check(self, polygon, partial=False, allow_unknown=False):
        x, y, yaw = self.origin
        poly = transform_points(np.asarray(polygon)-[x, y], (0, 0, -yaw))/self.resolution
        lo, hi = poly.min(axis=0), poly.max(axis=0)
        covered = bool(lo[0] >= 0 and lo[1] >= 0 and hi[0] < self.width and hi[1] < self.height)
        if not covered and not partial:
            return Check(False, 'outside_global_costmap', covered=False)
        # Include all cells touched, even only at a shared boundary.
        start = np.maximum(np.floor(lo-1e-9).astype(int), [0, 0])
        end = np.minimum(np.floor(hi+1e-9).astype(int), [self.width-1, self.height-1])
        if (start > end).any():
            return Check(True, covered=False)
        xx, yy = np.meshgrid(np.arange(start[0], end[0]+1), np.arange(start[1], end[1]+1))
        centres = np.column_stack((xx.ravel()+.5, yy.ravel()+.5))
        edges = np.roll(poly, -1, axis=0)-poly
        axes = np.vstack(([1., 0.], [0., 1.], np.column_stack((-edges[:, 1], edges[:, 0]))))
        hit = np.ones(len(centres), dtype=bool)
        for axis in axes:
            pp = poly @ axis
            cp = centres @ axis
            radius = .5*np.abs(axis).sum()
            hit &= (cp+radius >= pp.min()-1e-9) & (cp-radius <= pp.max()+1e-9)
        values = self.data[yy.ravel()[hit], xx.ravel()[hit]]
        if not len(values):
            return Check(True, covered=False)
        if (values == 254).any():
            return Check(False, 'lethal_obstacle', covered=covered)
        if (values == 255).any() and not allow_unknown:
            return Check(False, 'unknown_space', covered=covered)
        # 253 is inflated cost, not a second full-footprint collision envelope.
        return Check(True, cost=float(np.minimum(values.astype(float), 253).mean()/253), covered=covered)


def swept_polygons(footprint, poses, linear_step, angular_step):
    """Conservative swept hulls for piecewise linear XY / shortest-yaw motion.

    Chord error padding encloses the vertex arcs between each pair of samples.
    This validates the given path model, not the controller's future trajectory.
    """
    if not poses or linear_step <= 0 or angular_step <= 0:
        raise ValueError('Invalid path or sampling step')
    radius = float(np.linalg.norm(footprint, axis=1).max())
    yield transform_points(footprint, poses[0])
    for a, b in zip(poses, poses[1:]):
        dyaw = angle(b[2]-a[2])
        n = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/linear_step),
                math.ceil(abs(dyaw)/angular_step))
        prev = transform_points(footprint, a)
        error = radius*(1-math.cos(abs(dyaw)/n/2)) + 1e-9
        for i in range(1, n+1):
            t = i/n
            p = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*dyaw)
            curr = transform_points(footprint, p)
            yield padded(hull(np.vstack((prev, curr))), error)
            prev = curr


def diverse_order(items, distance=.20, heading=.35):
    """Items must have pose and preliminary; preserve all deferred alternatives."""
    ordered = sorted(items, key=lambda c: (c.preliminary, c.id))
    chosen, deferred = [], []
    for c in ordered:
        if any(math.hypot(c.pose[0]-p.pose[0], c.pose[1]-p.pose[1]) < distance
               and abs(angle(c.pose[2]-p.pose[2])) < heading for p in chosen):
            deferred.append(c)
        else:
            chosen.append(c)
    return chosen + deferred


def path_length(poses):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:]))


def score(length, cost, turn, joint_motion, weights, length_scale):
    # Fixed scales avoid changing rankings when unrelated candidates are added.
    return (weights[0]*(length/length_scale) + weights[1]*cost
            + weights[2]*abs(angle(turn))/math.pi + weights[3]*joint_motion)
