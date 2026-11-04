"""Exercise real execute/resolve logic with ROS imports stubbed, not a ROS integration test."""
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_commander.lifecycle import MotionError, Canceled


@pytest.fixture
def commander(monkeypatch):
    class Result:
        def __init__(self):
            self.error_code = self.moveit_error_code = 0
            self.message = ''
            self.left_completed = self.right_completed = self.executed = False
    class Status:
        STATUS_SUCCEEDED = 4
    names = {
        'rclpy': {'ok': lambda: True},
        'rclpy.action': {n: object for n in ('ActionClient', 'ActionServer', 'CancelResponse', 'GoalResponse')},
        'rclpy.callback_groups': {'ReentrantCallbackGroup': object},
        'rclpy.executors': {'MultiThreadedExecutor': object},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'action_msgs.msg': {'GoalStatus': Status},
        'geometry_msgs.msg': {'PoseStamped': object},
        'sensor_msgs.msg': {'JointState': object},
        'moveit_msgs.action': {'MoveGroup': object, 'ExecuteTrajectory': object},
        'moveit_msgs.msg': {n: object for n in ('Constraints', 'JointConstraint', 'RobotState', 'RobotTrajectory', 'PlanningSceneComponents')},
        'moveit_msgs.srv': {'GetPositionIK': object, 'GetPlanningScene': object},
        'dual_arm_interfaces.action': {'MoveArms': NS(Result=Result)},
    }
    for name, values in names.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1]/'dual_arm_commander/server.py'
    spec = importlib.util.spec_from_file_location('dual_arm_commander._test_server', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    node = object.__new__(module.Commander)
    node.lock = threading.RLock()
    node.cancel_event = threading.Event()
    node.busy = True
    node.finishing = False
    node.fault_message = ''
    node.feedback = lambda *args: None
    node.get_logger = lambda: NS(error=lambda msg: None)
    node.check_bounds = lambda data: None
    node.presets = {'home': {'mode': 'both', 'target_type': 'joints'}}
    return node


class Handle:
    def __init__(self, plan_only=False):
        self.request = NS(plan_only=plan_only)
        self.is_cancel_requested = False
        self.status = None
    def succeed(self): self.status = 'succeeded'
    def abort(self): self.status = 'aborted'
    def canceled(self): self.status = 'canceled'


@pytest.mark.parametrize('mode,order,expected', [
    ('left', 'left_first', [('left',)]),
    ('right', 'left_first', [('right',)]),
    ('both', 'left_first', [('left', 'right')]),
    ('sequential', 'left_first', [('left',), ('right',)]),
    ('sequential', 'right_first', [('right',), ('left',)])])
def test_mode_dispatch(commander, mode, order, expected):
    commander.resolve = lambda goal: {'mode': mode, 'sequential_order': order}
    calls = []
    def step(handle, data, sides, result, plan_only):
        calls.append(sides)
        for side in sides: setattr(result, side+'_completed', True)
        result.executed = True
    commander.run_step = step
    handle = Handle(); result = commander.execute(handle)
    assert calls == expected and handle.status == 'succeeded'
    assert not commander.busy and result.executed


def test_sequential_partial_failure(commander):
    commander.resolve = lambda goal: {'mode': 'sequential', 'sequential_order': 'left_first'}
    def step(handle, data, sides, result, plan_only):
        if sides == ('left',):
            result.left_completed = result.executed = True
        else:
            raise MotionError(4, 'no path', -1)
    commander.run_step = step
    handle = Handle(); result = commander.execute(handle)
    assert handle.status == 'aborted' and result.error_code == 4
    assert result.left_completed and not result.right_completed
    assert not commander.busy and not commander.fault_message


def test_first_failure_never_starts_second(commander):
    commander.resolve = lambda goal: {'mode': 'sequential', 'sequential_order': 'right_first'}
    calls = []
    def step(handle, data, sides, result, plan_only):
        calls.append(sides)
        raise MotionError(5, 'controller failure')
    commander.run_step = step
    handle = Handle(); commander.execute(handle)
    assert calls == [('right',)] and handle.status == 'aborted'


def test_cancel_between_steps(commander):
    commander.resolve = lambda goal: {'mode': 'sequential', 'sequential_order': 'left_first'}
    handle = Handle(); calls = []
    def step(handle, data, sides, result, plan_only):
        commander.check_cancel()
        calls.append(sides)
        result.left_completed = result.executed = True
        commander.cancel_event.set(); handle.is_cancel_requested = True
    commander.run_step = step
    result = commander.execute(handle)
    assert calls == [('left',)] and handle.status == 'canceled'
    assert result.left_completed and result.error_code == 6


def test_invalid_goal_releases_slot(commander):
    def bad(goal): raise ValueError('bad target')
    commander.resolve = bad
    handle = Handle(); result = commander.execute(handle)
    assert result.error_code == 1 and handle.status == 'aborted' and not commander.busy


def test_preset_resolution_and_no_override(commander):
    goal = NS(goal_type='PRESET', preset='home', mode='', target_type='',
              velocity_scaling=0.0, acceleration_scaling=0.0, plan_only=False)
    assert commander.resolve(goal)['mode'] == 'both'
    goal.mode = 'left'
    with pytest.raises(ValueError, match='cannot override'): commander.resolve(goal)


def test_unknown_preset(commander):
    with pytest.raises(ValueError, match='Unknown preset'):
        commander.resolve(NS(goal_type='preset', preset='absent'))
