from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_commander.path_check import coefficients, samples
from dual_arm_commander.lifecycle import MotionError


def trajectory(p0, p1, v0=None, v1=None, a0=None, a1=None):
    def point(p, v, a, t):
        return NS(positions=[p], velocities=[] if v is None else [v],
                  accelerations=[] if a is None else [a], time_from_start=NS(sec=t, nanosec=0))
    return NS(joint_trajectory=NS(joint_names=['joint'], points=[point(p0,v0,a0,0),point(p1,v1,a1,1)]))


@pytest.mark.parametrize('velocity,acceleration', [(False,False),(True,False),(True,True)])
def test_spline_endpoints_and_sample_spacing(velocity, acceleration):
    path = trajectory(0, 0.5, 0 if velocity else None, 0 if velocity else None,
                      0 if acceleration else None, 0 if acceleration else None)
    points = [p['joint'] for p in samples(path, ['joint'])]
    assert points[0] == 0 and points[-1] == pytest.approx(0.5)
    assert max(abs(b-a) for a,b in zip(points,points[1:])) <= 0.010000001


def test_cubic_overshoot_is_not_missed_when_endpoints_equal():
    points = [p['joint'] for p in samples(trajectory(0,0,1,-1), ['joint'])]
    assert max(points) == pytest.approx(0.25, abs=0.001)


def test_quintic_obeys_position_velocity_acceleration_boundaries():
    c = coefficients(0.2,0.6,0.1,-0.2,0.3,-0.4,2.0)
    assert sum(c) == pytest.approx(0.6)
    assert sum(k*v for k,v in enumerate(c))/2 == pytest.approx(-0.2)
    assert sum(k*(k-1)*v for k,v in enumerate(c))/4 == pytest.approx(-0.4)
    assert c[0] == 0.2 and c[1]/2 == 0.1 and 2*c[2]/4 == 0.3


def test_sample_budget_rejects_instead_of_skipping_checks():
    with pytest.raises(MotionError, match='sample limit'):
        list(samples(trajectory(0,1), ['joint'], max_samples=5))


def test_wrong_arm_and_invalid_data_rejected():
    with pytest.raises(MotionError, match='Unexpected joints'):
        list(samples(trajectory(0,1), ['other']))
    with pytest.raises(MotionError, match='Non-finite'):
        list(samples(trajectory(0,float('nan')), ['joint']))
    t=trajectory(0,1)
    t.joint_trajectory.points[0].time_from_start.sec=1
    with pytest.raises(MotionError, match='timing'):
        list(samples(t,['joint']))
