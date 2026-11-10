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
        'moveit_msgs.srv': {'GetPositionIK': object, 'GetPlanningScene': object, 'GetStateValidity': object},
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


def fallback_setup(node, side='left'):
    from dual_arm_commander.config import joints
    node.p = {'auto_move_blocking_arm': True, 'avoidance_budget': 15.0,
              'avoidance_max_joint_offset': 0.7, 'avoidance_max_candidates': 96,
              'avoidance_clearance': 0.05}
    node.retreat_has_clearance = lambda *a: True
    node.path_has_clearance = lambda *a: True
    node.arm_links = {'left': {'left_hand'}, 'right': {'right_hand'}}
    node.bounds = {n: (-3.0, 3.0) for s in ('left', 'right') for n in joints(s)}
    start = NS(joint_state=NS(name=list(node.bounds), position=[0.0]*12))
    node.measured = lambda: start
    error = MotionError(4, 'blocked', -12)
    error.blocked_candidates = [{n: 0.3 for n in joints(side)}]
    def fail(*args): raise error
    node.plan = fail
    return start, error


def cross_reply():
    return NS(valid=False, contacts=[NS(contact_body_1='left_hand', contact_body_2='right_hand')])


def test_fast_path_does_not_query_validity_or_retreat(commander):
    commander.plan = lambda *a: ('trajectory', 'start')
    commander.state_validity = lambda *a: pytest.fail('extra validity query')
    commander.retreat_candidates = lambda *a: pytest.fail('extra retreat search')
    assert commander.plan_with_fallback(None, {'mode': 'left'}, ('left',)) == (
        'trajectory', 'start', ('left',))


@pytest.mark.parametrize('mode,enabled,has_candidates', [
    ('left', False, True), ('sequential', True, True), ('both', True, True),
    ('right', True, False)])
def test_fallback_gates(commander, mode, enabled, has_candidates):
    _, error = fallback_setup(commander)
    commander.p['auto_move_blocking_arm'] = enabled
    if not has_candidates: error.blocked_candidates = []
    commander.state_validity = lambda *a: pytest.fail('must not diagnose')
    with pytest.raises(MotionError) as caught:
        commander.plan_with_fallback(None, {'mode': mode}, ('left',))
    assert caught.value is error


@pytest.mark.parametrize('reply', [
    NS(valid=True, contacts=[]), NS(valid=False, contacts=[]),
    NS(valid=False, contacts=[NS(contact_body_1='left_hand', contact_body_2='table')]),
    NS(valid=False, contacts=[NS(contact_body_1='left_hand', contact_body_2='left_hand')]),
    NS(valid=False, contacts=[NS(contact_body_1='left_hand', contact_body_2='right_obstacle')])])
def test_non_cross_arm_failure_never_moves_other_arm(commander, reply):
    _, error = fallback_setup(commander)
    replies = iter([NS(valid=True), reply])
    commander.state_validity = lambda *a: next(replies)
    commander.plan_targets = lambda *a: pytest.fail('unexpected dual plan')
    with pytest.raises(MotionError) as caught:
        commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))
    assert caught.value is error


@pytest.mark.parametrize('side', ['left', 'right'])
def test_confirmed_blocker_produces_one_combined_plan(commander, side):
    start, error = fallback_setup(commander, side)
    replies = iter([NS(valid=True), cross_reply()])
    commander.state_validity = lambda *a: next(replies)
    calls = []
    def plan(data, sides, measured, target, remaining):
        calls.append(target)
        assert sides == ('left', 'right') and measured is start
        assert all(target[n] == v for n, v in error.blocked_candidates[0].items())
        assert len(target) == 12 and remaining > 0
        return 'dual trajectory'
    commander.plan_targets = plan
    result = commander.plan_with_fallback(None, {'mode': side}, (side,))
    assert result == ('dual trajectory', start, ('left', 'right'))
    assert len(calls) == 1


def test_invalid_start_never_retreats(commander):
    fallback_setup(commander)
    commander.state_validity = lambda *a: NS(valid=False)
    commander.retreat_candidates = lambda *a: pytest.fail('invalid start')
    with pytest.raises(MotionError, match='Current robot state'):
        commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))


def test_cancel_during_fallback_propagates(commander):
    fallback_setup(commander)
    def canceled(*args): raise Canceled()
    commander.state_validity = canceled
    with pytest.raises(Canceled):
        commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))


def test_retreat_order_and_bounds(commander):
    start, _ = fallback_setup(commander)
    candidates = list(commander.retreat_candidates('right', start))
    costs = [sum(v*v for v in c.values()) for c in candidates]
    assert costs == sorted(costs) and 0 < len(costs) <= 96
    assert all(abs(v) <= 0.7 for c in candidates for v in c.values())
    commander.bounds = {n: (-0.01, 0.01) for n in commander.bounds}
    assert list(commander.retreat_candidates('right', start)) == []


@pytest.mark.parametrize('plan_only', [True, False])
def test_fallback_execution_uses_one_trajectory_and_requested_completion(commander, monkeypatch, plan_only):
    module = sys.modules.get(commander.__class__.__module__)
    # The fixture loads with exec_module without registering the module.
    globals_ = commander.run_step.__func__.__globals__
    monkeypatch.setitem(globals_, 'ExecuteTrajectory', NS(Goal=lambda: NS()))
    start, _ = fallback_setup(commander)
    commander.p.update(start_tolerance=0.02, execution_timeout=180.0)
    commander.check_scene = lambda: None
    commander.plan_with_fallback = lambda *a: ('combined', start, ('left', 'right'))
    published, executed, feedback = [], [], []
    commander.plan_pub = NS(publish=published.append)
    commander.executor_client = object()
    commander.feedback = lambda h, stage, sides: feedback.append((stage, sides))
    def remote(client, goal, timeout):
        executed.append(goal.trajectory)
        return NS(status=4, result=NS(error_code=NS(val=1)))
    commander.remote = remote
    result = NS(executed=False, left_completed=False, right_completed=False)
    commander.run_step(None, {'mode': 'left'}, ('left',), result, plan_only)
    assert published == ['combined']
    assert executed == ([] if plan_only else ['combined'])
    assert result.executed == result.left_completed == (not plan_only)
    assert not result.right_completed
    if not plan_only:
        assert feedback == [('executing', ('left', 'right'))]


def test_changed_stationary_arm_prevents_execution(commander):
    import copy
    start, _ = fallback_setup(commander)
    commander.p.update(start_tolerance=0.02)
    commander.check_scene = lambda: None
    commander.plan_with_fallback = lambda *a: ('combined', start, ('left', 'right'))
    commander.plan_pub = NS(publish=lambda *a: None)
    moved = copy.deepcopy(start)
    moved.joint_state.position[-1] += 0.1
    commander.measured = lambda: moved
    commander.remote = lambda *a: pytest.fail('robot moved since planning')
    with pytest.raises(MotionError, match='Robot moved during planning'):
        commander.run_step(None, {'mode': 'left'}, ('left',), NS(), False)


def test_mixed_collision_rejected(commander):
    fallback_setup(commander)
    reply = cross_reply()
    reply.contacts.append(NS(contact_body_1='right_hand', contact_body_2='table'))
    assert not commander.only_cross_arm_contacts(reply)


@pytest.mark.parametrize('code,moveit', [(3, -31), (4, -6), (7, 0), (5, -4), (2, 0)])
def test_nonplanning_errors_do_not_trigger_fallback(commander, code, moveit):
    _, error = fallback_setup(commander)
    error.code, error.moveit_code = code, moveit
    commander.state_validity = lambda *a: pytest.fail('must not diagnose')
    with pytest.raises(MotionError) as caught:
        commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))
    assert caught.value is error


def test_combined_planner_request_keeps_all_joint_constraints(commander, monkeypatch):
    globals_ = commander.plan_targets.__func__.__globals__
    def goal():
        return NS(request=NS(), planning_options=NS(
            planning_scene_diff=NS(robot_state=NS())))
    monkeypatch.setitem(globals_, 'MoveGroup', NS(Goal=goal))
    monkeypatch.setitem(globals_, 'Constraints', lambda: NS(joint_constraints=[]))
    monkeypatch.setitem(globals_, 'JointConstraint', lambda: NS())
    start, _ = fallback_setup(commander)
    commander.p.update(dual_group='both_ur_manipulator', pipeline_id='ompl',
                       planner_id='RRTConnectkConfigDefault', planning_attempts=5,
                       planning_time=5.0, joint_tolerance=0.001)
    commander.planner = object()
    targets = {n: 0.1 for n in start.joint_state.name}
    def remote(client, goal, remaining):
        req = goal.request
        assert req.group_name == 'both_ur_manipulator'
        assert req.start_state is start
        assert req.allowed_planning_time == 2.0
        assert goal.planning_options.plan_only
        assert {c.joint_name: c.position for c in req.goal_constraints[0].joint_constraints} == targets
        return NS(status=4, result=NS(error_code=NS(val=1),
                  planned_trajectory=NS(joint_trajectory=NS(points=[1]))))
    commander.remote = remote
    commander.plan_targets({'velocity_scaling': 0.1, 'acceleration_scaling': 0.1},
                           ('left', 'right'), start, targets, 2.0)


@pytest.mark.parametrize('blocked', [False, True])
def test_generic_99999_from_normal_planning_requires_collision_evidence(commander, blocked):
    # Exercise actual plan -> retained failed candidates -> fallback, including
    # generic FAILURE on all eight single-arm attempts, as in the reported log.
    start, _ = fallback_setup(commander, 'right')
    commander.plan = commander.__class__.plan.__get__(commander)
    commander.p.update(planning_budget=30.0, ik_attempts=8)
    from dual_arm_commander.config import joints
    target = {n: 0.3 for n in joints('right')}
    commander.solve_targets = lambda *a: target.copy()
    calls, feedback = [], []
    commander.feedback = lambda h, stage, sides, detail='': feedback.append(stage)
    def plan(data, sides, measured, targets, remaining):
        calls.append(sides)
        if len(sides) == 1:
            raise MotionError(4, 'MoveIt planning failed', 99999)
        assert blocked and len(targets) == 12 and measured is start
        assert all(targets[n] == v for n, v in target.items())
        return 'dual trajectory'
    commander.plan_targets = plan
    validity_calls = []
    def validity(measured, targets, deadline):
        validity_calls.append(targets)
        if not targets or len(targets) == 12:
            return NS(valid=True)
        return cross_reply() if blocked else NS(valid=True, contacts=[])
    commander.state_validity = validity
    if blocked:
        result = commander.plan_with_fallback(None, {'mode': 'right', 'target_type': 'pose'}, ('right',))
        assert result == ('dual trajectory', start, ('left', 'right'))
        assert calls == [('right',)]*8 + [('left', 'right')]
        assert 'planning_avoidance' in feedback
    else:
        with pytest.raises(MotionError) as caught:
            commander.plan_with_fallback(None, {'mode': 'right', 'target_type': 'pose'}, ('right',))
        assert caught.value.moveit_code == 99999
        assert calls == [('right',)]*8
        assert 'avoidance_skipped' in feedback
    assert 'checking_blocker' in feedback
    assert len(validity_calls) == (2 if blocked else 9)


def test_generic_failure_of_first_dual_candidate_tries_next(commander):
    start, error = fallback_setup(commander)
    error.moveit_code = 99999
    replies = iter([NS(valid=True), cross_reply()])
    commander.state_validity = lambda *a: next(replies)
    calls = []
    def plan(*args):
        calls.append(args)
        if len(calls) == 1:
            raise MotionError(4, 'MoveIt planning failed', 99999)
        return 'second dual trajectory'
    commander.plan_targets = plan
    assert commander.plan_with_fallback(None, {'mode': 'left'}, ('left',)) == (
        'second dual trajectory', start, ('left', 'right'))
    assert len(calls) == 2


def test_expensive_contacts_stop_after_first_blocker_and_not_per_retreat(commander, monkeypatch):
    start, error = fallback_setup(commander)
    error.blocked_candidates = [dict(error.blocked_candidates[0]) for _ in range(8)]
    clock = [100.0]
    monkeypatch.setitem(commander.plan_with_fallback.__func__.__globals__,
                        'time', NS(monotonic=lambda: clock[0]))
    checks, plans = [], []
    def validity(measured, targets, deadline):
        checks.append(targets)
        assert len(checks) <= 2, 'Redundant collision-contact request'
        clock[0] += 3.356 if targets else 0.009
        return cross_reply() if targets else NS(valid=True)
    commander.state_validity = validity
    def plan(data, sides, measured, targets, remaining):
        plans.append(targets)
        assert sides == ('left', 'right') and len(targets) == 12
        assert remaining > 11.0, 'Diagnosis consumed the retreat budget'
        clock[0] += 0.1
        if len(plans) == 1:
            raise MotionError(4, 'No path for this retreat', 99999)
        return 'validated trajectory'
    commander.plan_targets = plan
    assert commander.plan_with_fallback(None, {'mode': 'left'}, ('left',)) == (
        'validated trajectory', start, ('left', 'right'))
    assert len(checks) == 2 and len(plans) == 2


@pytest.mark.parametrize('fail_on,label', [(1, 'Current state'), (2, 'Target candidate 1')])
def test_validity_timeout_identifies_exact_check(commander, fail_on, label):
    fallback_setup(commander)
    calls = []
    def validity(*args):
        calls.append(args)
        if len(calls) == fail_on:
            raise MotionError(7, 'Response timed out')
        return NS(valid=True)
    commander.state_validity = validity
    commander.plan_targets = lambda *a: pytest.fail('No confirmed blocker')
    with pytest.raises(MotionError, match=label + ': /check_state_validity timed out'):
        commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))


def test_clearance_filter_keeps_one_simultaneous_plan(commander):
    start, _ = fallback_setup(commander)
    replies = iter([NS(valid=True), cross_reply()])
    commander.state_validity = lambda *a: next(replies)
    endpoints = iter([False, True])
    commander.retreat_has_clearance = lambda *a: next(endpoints)
    calls = []
    commander.plan_targets = lambda *a: calls.append(a) or 'combined'
    assert commander.plan_with_fallback(None, {'mode': 'left'}, ('left',)) == (
        'combined', start, ('left', 'right'))
    assert len(calls) == 1 and calls[0][1] == ('left', 'right')
    assert len(calls[0][3]) == 12


def test_path_clearance_rejects_first_combined_plan(commander):
    fallback_setup(commander)
    replies = iter([NS(valid=True), cross_reply()])
    commander.state_validity = lambda *a: next(replies)
    paths = iter([False, True])
    commander.path_has_clearance = lambda *a: next(paths)
    calls = []
    commander.plan_targets = lambda *a: calls.append(a) or 'combined'
    commander.plan_with_fallback(None, {'mode': 'left'}, ('left',))
    assert len(calls) == 2 and all(c[1] == ('left', 'right') for c in calls)


def test_real_combined_path_rejects_intermediate_close_pass(commander):
    from dual_arm_commander.config import joints
    start, _ = fallback_setup(commander)
    commander.path_has_clearance = commander.__class__.path_has_clearance.__get__(commander)
    commander.p.update(avoidance_path_joint_step=0.01, avoidance_path_max_samples=5000, start_tolerance=0.02)
    names = joints('left') + joints('right')
    def point(value, second):
        return NS(positions=[value]*12, velocities=[], accelerations=[],
                  time_from_start=NS(sec=second, nanosec=0))
    path = NS(joint_trajectory=NS(joint_names=names, points=[point(0, 0), point(0.2, 1)]))
    checked = []
    def gap(start, targets):
        value = targets[names[0]]
        checked.append(value)
        return not 0.08 < value < 0.12
    commander.retreat_has_clearance = gap
    import time
    assert not commander.path_has_clearance(start, path, ('left', 'right'), time.monotonic()+5)
    assert any(0.08 < value < 0.12 for value in checked)


def test_combined_path_budget_and_cancel_stop_validation(commander):
    fallback_setup(commander)
    commander.path_has_clearance = commander.__class__.path_has_clearance.__get__(commander)
    commander.p.update(avoidance_path_joint_step=0.01, avoidance_path_max_samples=5000, start_tolerance=0.02)
    from dual_arm_commander.config import joints
    names = joints('left') + joints('right')
    path = NS(joint_trajectory=NS(joint_names=names, points=[NS(positions=[0.0]*12,
              velocities=[], accelerations=[], time_from_start=NS(sec=0, nanosec=0))]))
    with pytest.raises(MotionError, match='budget exhausted'):
        commander.path_has_clearance(commander.measured(), path, ('left', 'right'), 0.0)
    commander.cancel_event.set()
    with pytest.raises(Canceled):
        commander.path_has_clearance(commander.measured(), path, ('left', 'right'), 0.0)
