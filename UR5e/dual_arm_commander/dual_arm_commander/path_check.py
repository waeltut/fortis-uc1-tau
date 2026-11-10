"""Bounded sampling of the controller's linear/cubic/quintic joint splines."""
import math
from .lifecycle import MotionError


def coefficients(p0, p1, v0=None, v1=None, a0=None, a1=None, duration=1.0):
    d = p1 - p0
    if v0 is None or v1 is None:
        return (p0, d)
    v0, v1 = v0 * duration, v1 * duration
    if a0 is None or a1 is None:
        return (p0, v0, 3*d-2*v0-v1, -2*d+v0+v1)
    a0, a1 = a0*duration**2, a1*duration**2
    return (p0, v0, a0/2, 10*d-6*v0-4*v1-1.5*a0+0.5*a1,
            -15*d+8*v0+7*v1+1.5*a0-a1, 6*d-3*v0-3*v1-0.5*a0+0.5*a1)


def derivative_bound(curve):
    # The derivative lies in the convex hull of its Bernstein coefficients.
    # This avoids the very loose power-coefficient bound for quintic splines.
    power = [k*c for k,c in enumerate(curve) if k]
    degree = len(power)-1
    return max(abs(sum(power[k]*math.comb(i,k)/math.comb(degree,k)
                       for k in range(i+1))) for i in range(degree+1))


def samples(trajectory, expected_names, max_step=0.01, max_samples=5000):
    path = trajectory.joint_trajectory
    names = list(path.joint_names)
    points = path.points
    if set(names) != set(expected_names) or len(names) != len(set(names)) or not points:
        raise MotionError(4, 'Unexpected joints or empty avoidance trajectory')
    times = []
    for p in points:
        for field in ('positions', 'velocities', 'accelerations'):
            values = getattr(p, field)
            if (field == 'positions' or values) and len(values) != len(names):
                raise MotionError(4, 'Malformed avoidance trajectory')
            if not all(math.isfinite(v) for v in values):
                raise MotionError(4, 'Non-finite avoidance trajectory')
        t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        if t < 0 or (times and t <= times[-1]):
            raise MotionError(4, 'Invalid avoidance trajectory timing')
        times.append(t)
    # MoveIt should include the planning start at zero; do not guess a controller
    # transition from the measured state to a delayed first waypoint.
    if times[0] > 1e-6:
        raise MotionError(4, 'Avoidance trajectory must start at time zero')
    count = 1
    yield dict(zip(names, points[0].positions))
    for index, (a, b) in enumerate(zip(points, points[1:])):
        curves = []
        for j in range(len(names)):
            curves.append(coefficients(a.positions[j], b.positions[j],
                a.velocities[j] if a.velocities else None,
                b.velocities[j] if b.velocities else None,
                a.accelerations[j] if a.accelerations else None,
                b.accelerations[j] if b.accelerations else None,
                times[index+1]-times[index]))
        # Bound |dq/du|, including overshoot between nearby endpoints.
        bound = max(derivative_bound(curve) for curve in curves)
        steps = max(1, math.ceil(bound / max_step))
        if count + steps > max_samples:
            raise MotionError(4, 'Avoidance path sample limit exceeded')
        count += steps
        for i in range(1, steps+1):
            u = i / steps
            yield {name: sum(c*u**k for k, c in enumerate(curve))
                   for name, curve in zip(names, curves)}
