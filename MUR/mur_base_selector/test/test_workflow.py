"""Exercise actual node methods with transport/message doubles, without ROS.

This checks request fields, result handling and fail-closed behaviour. It is
not a DDS, TF2, rclpy executor, colcon, or physical robot integration test.
"""
import ast
import copy
import importlib.util
import json
from pathlib import Path as FilePath
import sys
import threading
import types
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class PoseStamped:
    def __init__(self):
        self.header = NS(frame_id='map', stamp=NS(sec=0, nanosec=0))
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class Marker(Message):
    DELETEALL, ARROW, ADD = 3, 0, 0
    def __init__(self, **kwargs):
        self.header = NS(frame_id='', stamp=None)
        self.scale = NS(x=0., y=0., z=0.)
        self.color = NS(r=0., g=0., b=0., a=0.)
        super().__init__(**kwargs)


class Future:
    def __init__(self, value):
        self.value = value
    def done(self):
        return True
    def cancelled(self):
        return False
    def result(self):
        return self.value
    def add_done_callback(self, f):
        f(self)


class Pending(Future):
    def done(self):
        return False


class Publisher:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


def load_node():
    path = FilePath(__file__).parents[1]/'mur_base_selector'/'navigation.py'
    tree = ast.parse(path.read_text())
    blocked = ('rclpy', 'rcl_interfaces', 'action_msgs', 'geometry_msgs', 'sensor_msgs',
               'lifecycle_msgs', 'std_msgs', 'std_srvs', 'nav_msgs', 'nav2_msgs',
               'mur_reachability', 'tf2_ros', 'visualization_msgs')
    tree.body = [n for n in tree.body if not
                 (isinstance(n, ast.ImportFrom) and n.module and n.module.split('.')[0] in blocked)
                 and not (isinstance(n, ast.Import) and any(a.name in blocked for a in n.names))]
    mod = types.ModuleType('mur_selector_test_node')
    mod.__package__ = 'mur_base_selector'
    sys.modules[mod.__name__] = mod
    mod.__dict__.update(Node=object, PoseStamped=PoseStamped, Marker=Marker,
        MarkerArray=Message, Path=Message, String=Message, Bool=Message,
        rclpy=NS(ok=lambda: True), Duration=lambda **kw: NS(to_msg=lambda: NS(sec=1, nanosec=0)),
        Time=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)),
        ComputePathToPose=NS(Goal=Message),
        GoalStatus=NS(STATUS_SUCCEEDED=4), SetParametersResult=Message)
    exec(compile(tree, str(path), 'exec'), mod.__dict__)
    return mod


m = load_node()




def node():
    n = m.BaseSelector.__new__(m.BaseSelector)
    n.cfg = dict(m.DEFAULTS)
    n.cfg['arm_joints'] = ['left_elbow_joint']
    n.lock, n.stop = threading.RLock(), threading.Event()
    n.busy, n.sequence, n.selection_deadline = True, 1, 0.
    n.selected_context, n.last_target = None, None
    n.inflight_plan, n.plan_pending = None, False
    for name in ('status', 'valid', 'result', 'pose', 'path', 'markers', 'diagnostics'):
        setattr(n, name+'_pub', Publisher())
    n.get_logger = lambda: NS(info=lambda _: None)
    n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=123, nanosec=0)))
    n.transform_target = lambda target, frame: copy.deepcopy(target)
    n.robot_pose = lambda: (0., 0., 0.)
    n.joint_snapshot = lambda: {'left_elbow_joint': 0.}
    n.footprint_snapshot = lambda: [[-.3, -.3], [.3, -.3], [.3, .3], [-.3, .3]]
    n.costmaps = lambda: []
    n.tf_planar = lambda *a: (0., 0., 0.)
    n.guard_motion = lambda *a: n.checkpoint()
    n.polygon_check = lambda *a: (True, '', .2)
    n.compute_path = lambda c, start, deadline: ([start, c.pose], '')
    n.check_path = lambda *a: (True, '', .2)
    n.planner = NS(wait_for_server=lambda **kw: True)
    return n


class WorkflowTests(unittest.TestCase):
    def test_cleared_path_has_fixed_frame_and_timestamp(self):
        n = node()
        n.invalidate()
        cleared = n.path_pub.messages[-1]
        self.assertEqual(cleared.header.frame_id, 'map')
        self.assertEqual(cleared.header.stamp.sec, 123)



    def test_tf_receiver_owns_separate_node_and_executor(self):
        class FakeNode:
            def __init__(self, name, **kw):
                self.name, self.kw, self.destroyed = name, kw, False
            def destroy_node(self):
                self.destroyed = True
        class FakeExecutor:
            def __init__(self, **kw):
                self.nodes, self.stop, self.started = [], threading.Event(), threading.Event()
            def add_node(self, n):
                self.nodes.append(n)
            def spin(self):
                self.started.set()
                self.stop.wait(2.)
            def shutdown(self):
                self.stop.set()
        owner = NS(context=object(), get_name=lambda: 'mur_base_selector',
                   get_namespace=lambda: '/', get_parameter=lambda _: NS(value=True))
        with patch.dict(m.__dict__, Node=FakeNode, SingleThreadedExecutor=FakeExecutor,
                        Parameter=lambda name, **kw: NS(name=name, **kw),
                        Buffer=lambda **kw: NS(**kw),
                        TransformListener=lambda b, n: NS(unregister=lambda: None),
                        ExternalShutdownException=RuntimeError):
            receiver = m.TfReceiver(owner)
            try:
                self.assertTrue(receiver.executor.started.wait(1.))
                self.assertEqual(receiver.executor.nodes, [receiver.node])
                self.assertIsNot(receiver.node, owner)
                self.assertIs(receiver.buffer.node, receiver.node)
                self.assertTrue(receiver.node.kw['parameter_overrides'][0].value)
                self.assertFalse(receiver.node.kw['use_global_arguments'])
            finally:
                receiver.close()
            self.assertFalse(receiver.thread.is_alive())
            self.assertTrue(receiver.node.destroyed)

    def test_tf_diagnostics_use_same_buffer_and_ros_clock(self):
        n = node()
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
        stamps = {('map', 'odom'): 99, ('odom', 'base_link'): 92}
        n.tf = NS(lookup_transform=lambda p, c, t: NS(header=NS(stamp=NS(sec=stamps[p,c], nanosec=0))))
        report = n.tf_age_diagnostics()
        self.assertIn('map->odom: stamp=99.000, age=1.000s', report)
        self.assertIn('odom->base_link: stamp=92.000, age=8.000s', report)
        self.assertIn('ros_now=100.000', report)

    def test_freshness_limit_remains_three_seconds(self):
        n = node()
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
        n.check_age(NS(sec=99, nanosec=600_000_000), 'base TF')
        with self.assertRaises(m.SelectionError):
            n.check_age(NS(sec=92, nanosec=200_000_000), 'base TF')










    def test_pending_action_gate_blocks_resubmit(self):
        n = node()
        n.busy, n.plan_pending = False, True
        self.assertFalse(n.submit(PoseStamped())[0])

    def test_invalid_quaternion_rejected(self):
        q = NS(x=0., y=0., z=0., w=0.)
        with self.assertRaises(m.SelectionError):
            m.quaternion(q)

    def test_humble_action_status_and_endpoint(self):
        n = node()
        # Use real compute_path method, with a completed transport double.
        del n.compute_path
        path = NS(header=NS(frame_id='map'), poses=[m.pose_msg((0, 0, 0), 'map'), m.pose_msg((1, 0, 0), 'map')])
        terminal = Future(NS(status=4, result=NS(path=path)))
        handle = NS(accepted=True, get_result_async=lambda: terminal)
        n.planner.send_goal_async = lambda goal: Future(handle)
        c = m.Candidate(0, (1., 0., 0.), 'left')
        poses, reason = n.compute_path(c, (0, 0, 0), m.time.monotonic()+10)
        self.assertTrue(poses)
        self.assertEqual(reason, '')
        self.assertFalse(n.plan_pending)
        path.poses[-1].pose.position.x = .8
        self.assertEqual(n.compute_path(c, (0, 0, 0), m.time.monotonic()+10)[1], 'endpoint_position_mismatch')

    def test_reported_22mm_start_offset_accepted_but_goal_tolerance_unchanged(self):
        n = node()
        del n.compute_path
        start = (-.007, -.032, 0.)
        first = (-.020, -.050, 0.)
        goal = (1., 0., 0.)
        path = NS(header=NS(frame_id='map'),
                  poses=[m.pose_msg(first, 'map'), m.pose_msg(goal, 'map')])
        terminal = Future(NS(status=4, result=NS(path=path)))
        handle = NS(accepted=True, get_result_async=lambda: terminal)
        n.planner.send_goal_async = lambda _: Future(handle)
        c = m.Candidate(0, goal, 'left')
        poses, reason = n.compute_path(c, start, m.time.monotonic()+10)
        self.assertEqual(reason, '')
        self.assertEqual(poses[0], start)  # actual pose retained for swept collision check
        self.assertEqual(poses[1], first)
        self.assertEqual(poses[-1], goal)
        path.poses[-1].pose.position.x = 1.0222
        self.assertEqual(n.compute_path(c, start, m.time.monotonic()+10)[1], 'endpoint_position_mismatch')
        path.poses[-1].pose.position.x = 1.
        path.poses[0].pose.position.x = start[0]+.06
        self.assertEqual(n.compute_path(c, start, m.time.monotonic()+10)[1], 'start_position_mismatch')


if __name__ == '__main__':
    unittest.main()
