"""Action-only dual-arm commander. All actual movement goes through MoveIt."""
import copy
import math
import random
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, RobotTrajectory, PlanningSceneComponents
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from dual_arm_interfaces.action import MoveArms
from .config import SIDES, joints, choice, load_presets, validate
from .lifecycle import MotionError, Canceled, run_action, wait_future


class Commander(Node):
    def __init__(self):
        super().__init__('dual_arm_commander')
        defaults = {
            'presets_file': '', 'robot_description': '',
            'action_name': '/arms/move', 'joint_states_topic': '/joint_states',
            'move_group_action': '/move_action', 'execute_action': '/execute_trajectory',
            'ik_service': '/compute_ik', 'scene_service': '/get_planning_scene',
            'left_group': 'left_ur_manipulator', 'right_group': 'right_ur_manipulator',
            'dual_group': 'both_ur_manipulator',
            'left_tcp': 'left_tcp', 'right_tcp': 'right_tcp',
            'velocity_scaling': 0.1, 'acceleration_scaling': 0.1,
            'planning_time': 5.0, 'planning_attempts': 5, 'ik_attempts': 8,
            'ik_timeout': 0.5, 'planning_budget': 30.0,
            'execution_timeout': 180.0, 'response_timeout': 10.0,
            'cancel_timeout': 10.0, 'state_max_age': 1.0,
            'joint_tolerance': 0.001, 'start_tolerance': 0.02,
            'planner_id': 'RRTConnectkConfigDefault', 'pipeline_id': 'ompl',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        for name in ('planning_time', 'ik_timeout', 'planning_budget', 'execution_timeout',
                     'response_timeout', 'cancel_timeout', 'state_max_age',
                     'joint_tolerance', 'start_tolerance'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('planning_attempts', 'ik_attempts'):
            if not 1 <= self.p[name] <= 64:
                raise ValueError(f'{name} must be in 1..64')
        for name in ('velocity_scaling', 'acceleration_scaling'):
            if not math.isfinite(self.p[name]) or not 0 < self.p[name] <= 1:
                raise ValueError(f'{name} must be in (0,1]')
        self.presets = load_presets(self.p['presets_file'], self.p)
        model = ET.fromstring(self.p['robot_description'])
        self.bounds = {}
        for joint in model.findall('joint'):
            if joint.get('type') in ('revolute', 'prismatic'):
                limit = joint.find('limit')
                low, high = float(limit.get('lower')), float(limit.get('upper'))
                soft = joint.find('safety_controller')
                if soft is not None:
                    low = max(low, float(soft.get('soft_lower_limit', low)))
                    high = min(high, float(soft.get('soft_upper_limit', high)))
                self.bounds[joint.get('name')] = (low, high)
        required = set(joints('left') + joints('right'))
        if not required <= self.bounds.keys():
            raise ValueError('robot_description must contain the 12 prefixed arm joint limits')
        for preset in self.presets.values():
            self.check_bounds(preset)
        self.lock = threading.RLock()
        self.busy = False
        self.finishing = False
        self.fault_message = ''
        self.cancel_event = threading.Event()
        self.state = {}  # joint -> (position, monotonic receive time, ROS stamp)
        self.group = ReentrantCallbackGroup()
        self.create_subscription(JointState, self.p['joint_states_topic'], self.on_state,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.planner = ActionClient(self, MoveGroup, self.p['move_group_action'], callback_group=self.group)
        self.executor_client = ActionClient(self, ExecuteTrajectory, self.p['execute_action'], callback_group=self.group)
        self.ik = self.create_client(GetPositionIK, self.p['ik_service'], callback_group=self.group)
        self.scene = self.create_client(GetPlanningScene, self.p['scene_service'], callback_group=self.group)
        self.plan_pub = self.create_publisher(RobotTrajectory, '/arms/planned_trajectory', 10)
        self.server = ActionServer(self, MoveArms, self.p['action_name'],
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=self.group)
        self.get_logger().info(f'Action ready: {self.p["action_name"]}; presets: {", ".join(self.presets)}')

    def on_state(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if math.isfinite(position):
                    self.state[name] = (position, now, stamp)

    def measured(self):
        with self.lock:
            now = time.monotonic()
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            for name in joints('left') + joints('right'):
                entry = self.state.get(name)
                if entry is None or now - entry[1] > self.p['state_max_age'] or (
                    entry[2] != 0 and not -0.1 <= ros_now - entry[2] <= self.p['state_max_age']):
                    raise MotionError(2, f'Missing/stale joint state: {name}')
            state = RobotState()
            state.is_diff = True
            state.joint_state.name = [n for n, v in self.state.items() if now-v[1] <= self.p['state_max_age']]
            state.joint_state.position = [float(self.state[n][0]) for n in state.joint_state.name]
            return state

    def on_goal(self, request):
        with self.lock:
            if self.busy or self.fault_message:
                self.get_logger().warning('Goal rejected: ' + (self.fault_message or 'commander busy'))
                return GoalResponse.REJECT
            self.busy = True
            self.finishing = False
            self.cancel_event.clear()
        # Validate in execute callback so invalid requests return a useful result.
        return GoalResponse.ACCEPT

    def on_cancel(self, handle):
        with self.lock:
            if self.finishing or not handle.is_active:
                return CancelResponse.REJECT
            self.cancel_event.set()
        return CancelResponse.ACCEPT

    def canceled(self):
        return self.cancel_event.is_set() or not rclpy.ok()

    def check_cancel(self):
        if self.canceled():
            raise Canceled()

    def fault(self, message):
        with self.lock:
            self.fault_message = message
        self.get_logger().error(message)

    def feedback(self, handle, stage, sides, detail=''):
        msg = MoveArms.Feedback()
        msg.stage, msg.active_arms, msg.detail = stage, ','.join(sides), detail
        handle.publish_feedback(msg)

    def resolve(self, goal):
        goal_type = choice(goal.goal_type, ('preset', 'explicit'), 'goal_type')
        if goal_type == 'preset':
            if goal.preset not in self.presets:
                raise ValueError(f'Unknown preset {goal.preset!r}; available: {", ".join(self.presets)}')
            # Reject ambiguous overrides, rather than silently ignoring them.
            if goal.mode or goal.target_type or goal.velocity_scaling or goal.acceleration_scaling:
                raise ValueError('Preset goals cannot override mode, target_type or scaling')
            data = copy.deepcopy(self.presets[goal.preset])
        else:
            if goal.preset:
                raise ValueError('Explicit goals must leave preset empty')
            data = {key: getattr(goal, key) for key in ('mode', 'target_type',
                     'sequential_order', 'velocity_scaling', 'acceleration_scaling')}
            for side in SIDES:
                pose = getattr(goal, f'{side}_target')
                if pose.header.stamp.sec or pose.header.stamp.nanosec:
                    raise ValueError('Use zero target timestamps; poses resolve when the action starts')
                p, q = pose.pose.position, pose.pose.orientation
                data[f'{side}_target'] = {'frame_id': pose.header.frame_id,
                    'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w]}
                js = getattr(goal, f'{side}_joints')
                if len(js.name) != len(js.position) or len(set(js.name)) != len(js.name):
                    raise ValueError(f'{side}_joints: mismatched lengths or duplicate names')
                if js.velocity or js.effort:
                    raise ValueError('Joint targets use name and position only')
                data[f'{side}_joints'] = dict(zip(js.name, js.position))
            data = validate(data, self.p)
        if goal.plan_only and data['mode'] == 'sequential':
            raise ValueError('Sequential plan_only is unsupported; test each step or use fake hardware')
        self.check_bounds(data)
        return data

    def check_bounds(self, data):
        if data['target_type'] == 'joints':
            sides = SIDES if data['mode'] in ('both', 'sequential') else (data['mode'],)
            for side in sides:
                for name, position in data[f'{side}_joints'].items():
                    low, high = self.bounds[name]
                    if not low <= position <= high:
                        raise ValueError(f'{name}={position} outside URDF limits [{low}, {high}]')

    def service(self, client, request):
        self.check_cancel()
        if not client.wait_for_service(timeout_sec=0.5):
            raise MotionError(2, f'Service unavailable: {client.srv_name}')
        return wait_future(client.call_async(request), self.p['response_timeout'], self.canceled)

    def check_scene(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.service(self.scene, req).scene.allowed_collision_matrix
        for name, allowed in zip(acm.default_entry_names, acm.default_entry_values):
            if allowed and name.startswith(('left_', 'right_')):
                raise MotionError(2, f'Collision checking disabled by ACM default for {name}')
        for i, first in enumerate(acm.entry_names):
            if not first.startswith('left_'):
                continue
            for j, second in enumerate(acm.entry_names):
                if second.startswith('right_') and acm.entry_values[i].enabled[j]:
                    raise MotionError(2, f'Cross-arm collision checking disabled: {first} / {second}')

    def remote(self, client, goal, timeout):
        self.check_cancel()
        if not client.wait_for_server(timeout_sec=0.5):
            raise MotionError(2, 'MoveIt action server unavailable')
        return run_action(client, goal, timeout, self.p['cancel_timeout'], self.canceled, self.fault)

    def pose(self, target):
        msg = PoseStamped()
        msg.header.frame_id = target['frame_id']
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target['position']
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = target['orientation']
        return msg

    def solve_targets(self, data, sides, start, attempt):
        if data['target_type'] == 'joints':
            return {name: value for side in sides for name, value in data[f'{side}_joints'].items()}
        seed = copy.deepcopy(start)
        if attempt:
            values = dict(zip(seed.joint_state.name, seed.joint_state.position))
            for side in sides:
                for name in joints(side):
                    values[name] = random.uniform(*self.bounds[name])
            seed.joint_state.position = [values[n] for n in seed.joint_state.name]
        targets = {}
        for side in sides:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.p[f'{side}_group']
            req.ik_request.ik_link_name = self.p[f'{side}_tcp']
            req.ik_request.pose_stamped = self.pose(data[f'{side}_target'])
            req.ik_request.robot_state = seed
            # Candidate only: full combined trajectory is collision checked by MoveIt.
            req.ik_request.avoid_collisions = False
            sec = self.p['ik_timeout']
            req.ik_request.timeout.sec = int(sec)
            req.ik_request.timeout.nanosec = int((sec-int(sec))*1e9)
            reply = self.service(self.ik, req)
            if reply.error_code.val != 1:
                raise MotionError(3, f'{side} IK failed', reply.error_code.val)
            solved = dict(zip(reply.solution.joint_state.name, reply.solution.joint_state.position))
            for name in joints(side):
                value = solved.get(name)
                if value is None or not math.isfinite(value):
                    raise MotionError(3, f'IK response missing finite joint {name}')
                targets[name] = value
            seed = reply.solution
        return targets

    def plan(self, handle, data, sides):
        start = self.measured()
        deadline = time.monotonic() + self.p['planning_budget']
        attempts = self.p['ik_attempts'] if data['target_type'] == 'pose' else 1
        last_error = MotionError(4, 'No valid motion plan')
        for attempt in range(attempts):
            self.check_cancel()
            if time.monotonic() >= deadline:
                break
            self.feedback(handle, 'planning', sides, f'Candidate {attempt+1}/{attempts}')
            try:
                targets = self.solve_targets(data, sides, start, attempt)
            except MotionError as exc:
                if exc.code != 3:
                    raise
                last_error = exc
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = self.p['dual_group'] if len(sides) == 2 else self.p[f'{sides[0]}_group']
            request.pipeline_id = self.p['pipeline_id']
            request.planner_id = self.p['planner_id']
            request.start_state = start  # Never use randomized IK seed as motion start.
            request.num_planning_attempts = self.p['planning_attempts']
            request.allowed_planning_time = min(self.p['planning_time'], remaining)
            request.max_velocity_scaling_factor = data['velocity_scaling']
            request.max_acceleration_scaling_factor = data['acceleration_scaling']
            constraints = Constraints()
            for name, value in targets.items():
                joint = JointConstraint()
                joint.joint_name, joint.position = name, float(value)
                joint.tolerance_above = joint.tolerance_below = self.p['joint_tolerance']
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            request.goal_constraints = [constraints]
            goal.planning_options.plan_only = True
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            response = self.remote(self.planner, goal, remaining)
            code = response.result.error_code.val
            if response.status == GoalStatus.STATUS_SUCCEEDED and code == 1:
                trajectory = response.result.planned_trajectory
                if not trajectory.joint_trajectory.points:
                    raise MotionError(4, 'MoveIt returned an empty trajectory')
                return trajectory, start
            last_error = MotionError(4, 'MoveIt planning failed', code)
        raise last_error

    def run_step(self, handle, data, sides, result, plan_only):
        self.check_cancel()
        self.check_scene()
        trajectory, start = self.plan(handle, data, sides)
        self.plan_pub.publish(trajectory)
        self.check_cancel()
        if plan_only:
            return
        # Also check the stationary arm: its movement could invalidate the plan.
        current = dict(zip((state := self.measured()).joint_state.name, state.joint_state.position))
        original = dict(zip(start.joint_state.name, start.joint_state.position))
        for name in joints('left') + joints('right'):
            if abs(current[name] - original[name]) > self.p['start_tolerance']:
                raise MotionError(2, f'Robot moved during planning: {name}; submit a fresh goal')
        self.check_scene()
        self.feedback(handle, 'executing', sides)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        # Let MoveIt choose configured controllers, including fake-hardware names.
        response = self.remote(self.executor_client, goal, self.p['execution_timeout'])
        code = response.result.error_code.val
        if response.status != GoalStatus.STATUS_SUCCEEDED or code != 1:
            raise MotionError(5, 'MoveIt execution failed', code)
        result.executed = True
        for side in sides:
            setattr(result, f'{side}_completed', True)

    def execute(self, handle):
        result = MoveArms.Result()
        try:
            self.feedback(handle, 'validating', ())
            data = self.resolve(handle.request)
            self.check_cancel()
            mode = data['mode']
            if mode == 'sequential':
                order = SIDES if data['sequential_order'] == 'left_first' else tuple(reversed(SIDES))
                for side in order:
                    self.run_step(handle, data, (side,), result, False)
            else:
                sides = SIDES if mode == 'both' else (mode,)
                self.run_step(handle, data, sides, result, handle.request.plan_only)
            result.message = 'Plan ready; no motion executed' if handle.request.plan_only else 'Requested motion completed'
        except ValueError as exc:
            result.error_code, result.message = 1, str(exc)
        except MotionError as exc:
            result.error_code, result.message, result.moveit_error_code = exc.code, str(exc), exc.moveit_code
        except Exception as exc:
            result.error_code, result.message = 8, f'{type(exc).__name__}: {exc}'
            self.get_logger().error(result.message)
        finally:
            # Serialize completion against cancellation and new goal acceptance.
            with self.lock:
                self.finishing = True
                cancel = self.cancel_event.is_set()
            if cancel and not self.fault_message:
                # rclpy transitions to CANCELING after cancel_callback returns.
                limit = time.monotonic() + 1.0
                while not handle.is_cancel_requested and time.monotonic() < limit and rclpy.ok():
                    time.sleep(0.01)
                if handle.is_cancel_requested:
                    result.error_code, result.message = 6, 'Canceled; downstream operation terminated'
                    handle.canceled()
                else:
                    result.error_code, result.message = 8, 'Cancellation state transition unavailable'
                    handle.abort()
            elif result.error_code:
                handle.abort()
            else:
                handle.succeed()
            with self.lock:
                self.busy = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = Commander()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cancel_event.set()
        executor.shutdown(timeout_sec=15.0)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
