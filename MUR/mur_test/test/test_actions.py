"""Offline protocol and lifecycle tests; no ROS installation needed."""
import ast
from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('runner', ROOT/'mur_test/action_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def ready(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True
    def __init__(self, terminal=True):
        self.cancels = 0
        self.result = ready(NS(status=4)) if terminal else Future()
    def get_result_async(self):
        return self.result
    def cancel_goal_async(self):
        self.cancels += 1
        if not self.result.done():
            self.result.set_result(NS(status=5))


class LifecycleTests(unittest.TestCase):
    def run_goal(self, sent, stopped=lambda: None, faults=None):
        return runner.run_action(NS(send_goal_async=lambda *a, **k: sent), None,
                                 .005, .005, .005, stopped,
                                 (faults if faults is not None else []).append)
    def test_success(self):
        self.assertEqual(self.run_goal(ready(Handle())).status, 4)
    def test_rejection(self):
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.run_goal(ready(NS(accepted=False)))
    def test_timeout_cancels(self):
        handle = Handle(False)
        with self.assertRaisesRegex(RuntimeError, 'timed out.*terminated'):
            self.run_goal(ready(handle))
        self.assertEqual(handle.cancels, 1)
    def test_new_target_cancels(self):
        handle = Handle(False)
        with self.assertRaisesRegex(RuntimeError, 'New target.*terminated'):
            self.run_goal(ready(handle), lambda: 'New target')
        self.assertEqual(handle.cancels, 1)
    def test_late_acceptance(self):
        sent, faults = Future(), []
        with self.assertRaises(TimeoutError):
            self.run_goal(sent, faults=faults)
        handle = Handle(False)
        sent.set_result(handle)
        self.assertEqual(handle.cancels, 1)
        self.assertTrue(faults)
    def test_unconfirmed_cancel_faults(self):
        handle, faults = Handle(False), []
        handle.cancel_goal_async = lambda: None
        with self.assertRaisesRegex(TimeoutError, 'not confirmed'):
            self.run_goal(ready(handle), lambda: 'Stop', faults)
        self.assertTrue(faults)


class ProtocolTests(unittest.TestCase):
    def test_selected_arm_request_and_result(self):
        source = ast.parse((ROOT/'mur_test/test_node.py').read_text())
        cls = next(n for n in source.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == 'execute_commander_goal')
        env = {'MoveArms': NS(Goal=NS, Result=NS(OK=0)),
               'GoalStatus': NS(STATUS_SUCCEEDED=4)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<method>', 'exec'), env)
        for arm in ('left', 'right'):
            for plan_only in (False, True):
                params = dict(commander_wait_timeout=1., planning_frame='mur',
                              commander_action='/arms/move', velocity_scaling=.1,
                              acceleration_scaling=.1, plan_only=plan_only, arm_timeout=180.)
                pose = NS(header=NS(frame_id='mur', stamp=NS(sec=123, nanosec=456)))
                sent = []
                result = NS(error_code=0, moveit_error_code=1, message='done',
                            executed=not plan_only, left_completed=arm=='left' and not plan_only,
                            right_completed=arm=='right' and not plan_only)
                node = NS(p=params.__getitem__, commander=NS(wait_for_server=lambda **k: True),
                          transform_to=lambda *a: pose, get_logger=lambda: NS(info=lambda *a: None),
                          on_arm_feedback=None,
                          run_motion=lambda c, g, *a: (sent.append(g) or NS(status=4, result=result)))
                env['execute_commander_goal'](node, arm, object(), 1)
                goal = sent[0]
                self.assertEqual((goal.goal_type, goal.mode, goal.target_type), ('explicit', arm, 'pose'))
                self.assertEqual(getattr(goal, arm+'_target').header.stamp.sec, 0)
                self.assertEqual(getattr(goal, arm+'_target').header.stamp.nanosec, 0)
                self.assertEqual(goal.plan_only, plan_only)
                result.error_code = 3
                with self.assertRaisesRegex(RuntimeError, 'Arm action failed'):
                    env['execute_commander_goal'](node, arm, object(), 1)
                result.error_code = 0
                if not plan_only:
                    result.executed = False
                    with self.assertRaisesRegex(RuntimeError, 'did not confirm'):
                        env['execute_commander_goal'](node, arm, object(), 1)


if __name__ == '__main__':
    unittest.main()
