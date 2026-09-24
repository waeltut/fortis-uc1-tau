#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
    RobotTrajectory,
)

from shape_msgs.msg import SolidPrimitive


class MoveItPoseCommander(Node):

    def __init__(self):
        super().__init__('dual_arm_moveit_commander')

        # These MUST match the planning group names in your SRDF.
        self.declare_parameter('left_group', 'left_arm')
        self.declare_parameter('right_group', 'right_arm')

        self.declare_parameter('left_end_effector_link', 'left_tcp')
        self.declare_parameter('right_end_effector_link', 'right_tcp')

        # Used only if incoming PoseStamped.frame_id is empty.
        self.declare_parameter('default_reference_frame', 'chest')

        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)

        # Conservative defaults for initial physical testing.
        self.declare_parameter('velocity_scaling', 0.10)
        self.declare_parameter('acceleration_scaling', 0.10)

        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('orientation_tolerance', 0.03)

        # Start in plan-only mode.
        self.declare_parameter('execute', False)

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.left_sub = self.create_subscription(
            PoseStamped,
            '/left_arm/goal_pose',
            lambda msg: self.pose_callback('left', msg),
            10
        )

        self.right_sub = self.create_subscription(
            PoseStamped,
            '/right_arm/goal_pose',
            lambda msg: self.pose_callback('right', msg),
            10
        )

        # Useful for SSH/headless operation.
        self.left_plan_pub = self.create_publisher(
            RobotTrajectory,
            '/left_arm/planned_trajectory',
            10
        )

        self.right_plan_pub = self.create_publisher(
            RobotTrajectory,
            '/right_arm/planned_trajectory',
            10
        )

        # For now only allow one arm command at a time.
        self.busy = False

        self.get_logger().info(
            'Dual-arm MoveIt pose commander started'
        )

        self.get_logger().info(
            f'Execute = {self.get_parameter("execute").value}'
        )

    def pose_callback(self, side, target):
        if self.busy:
            self.get_logger().warning(
                f'Ignoring {side} goal: another MoveIt request is active'
            )
            return

        if not target.header.frame_id:
            target.header.frame_id = self.get_parameter(
                'default_reference_frame'
            ).value

        group = self.get_parameter(f'{side}_group').value

        end_effector_link = self.get_parameter(
            f'{side}_end_effector_link'
        ).value

        execute = self.get_parameter('execute').value

        self.get_logger().info(
            f'Received {side} arm target:\n'
            f'  frame: {target.header.frame_id}\n'
            f'  position: '
            f'({target.pose.position.x:.4f}, '
            f'{target.pose.position.y:.4f}, '
            f'{target.pose.position.z:.4f})\n'
            f'  group: {group}\n'
            f'  link: {end_effector_link}\n'
            f'  execute: {execute}'
        )

        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '/move_action is not available. Is move_group running?'
            )
            return

        goal = self.build_move_group_goal(
            group,
            end_effector_link,
            target,
            execute
        )

        self.busy = True

        future = self.move_group_client.send_goal_async(goal)

        future.add_done_callback(
            lambda f: self.goal_response_callback(side, f)
        )

    def build_move_group_goal(
        self,
        group,
        end_effector_link,
        target,
        execute
    ):
        goal = MoveGroup.Goal()

        request = goal.request

        request.group_name = group

        request.num_planning_attempts = int(
            self.get_parameter('planning_attempts').value
        )

        request.allowed_planning_time = float(
            self.get_parameter('planning_time').value
        )

        request.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scaling').value
        )

        request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value
        )

        # Empty diff means "use current robot state".
        request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = f'{group}_pose_goal'

        #
        # Position constraint
        #
        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = target.header.frame_id
        position_constraint.link_name = end_effector_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE

        position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )

        # Sphere radius in metres.
        sphere.dimensions = [position_tolerance]

        region_pose = Pose()
        region_pose.position = target.pose.position
        region_pose.orientation.w = 1.0

        position_constraint.constraint_region.primitives.append(
            sphere
        )

        position_constraint.constraint_region.primitive_poses.append(
            region_pose
        )

        constraints.position_constraints.append(
            position_constraint
        )

        #
        # Orientation constraint
        #
        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = target.header.frame_id
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = target.pose.orientation

        orientation_tolerance = float(
            self.get_parameter('orientation_tolerance').value
        )

        orientation_constraint.absolute_x_axis_tolerance = (
            orientation_tolerance
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            orientation_tolerance
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            orientation_tolerance
        )

        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        request.goal_constraints.append(constraints)

        #
        # MoveGroup action behaviour
        #
        goal.planning_options.plan_only = not execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def goal_response_callback(self, side, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(
                f'Failed sending {side} MoveIt goal: {ex}'
            )
            self.busy = False
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'{side} MoveIt goal was rejected'
            )
            self.busy = False
            return

        self.get_logger().info(
            f'{side} MoveIt goal accepted'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: self.result_callback(side, f)
        )

    def result_callback(self, side, future):
        try:
            wrapped_result = future.result()
            result = wrapped_result.result

        except Exception as ex:
            self.get_logger().error(
                f'{side} MoveIt result error: {ex}'
            )
            self.busy = False
            return

        if side == 'left':
            self.left_plan_pub.publish(result.planned_trajectory)
        else:
            self.right_plan_pub.publish(result.planned_trajectory)

        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            if self.get_parameter('execute').value:
                self.get_logger().info(
                    f'{side} arm motion completed successfully'
                )
            else:
                self.get_logger().info(
                    f'{side} arm planning successful '
                    '(execution disabled)'
                )
        else:
            self.get_logger().error(
                f'{side} MoveIt failed with error code '
                f'{error_code}'
            )

        self.busy = False


def main(args=None):
    rclpy.init(args=args)

    node = MoveItPoseCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()