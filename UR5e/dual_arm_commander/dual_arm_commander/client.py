"""Small preset client; also demonstrates feedback, result and cancellation."""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from dual_arm_interfaces.action import MoveArms


def main(args=None):
    parser = argparse.ArgumentParser(description='Run one named arm preset')
    parser.add_argument('preset')
    parser.add_argument('--action', default='/arms/move')
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--cancel-after', type=float, help='Request cancellation after this many seconds')
    options = parser.parse_args(args)
    if options.cancel_after is not None and options.cancel_after <= 0:
        parser.error('--cancel-after must be positive')
    rclpy.init()
    node = Node('move_preset_client')
    client = ActionClient(node, MoveArms, options.action)
    timer = None
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Arm action unavailable')
        goal = MoveArms.Goal()
        goal.goal_type, goal.preset, goal.plan_only = 'preset', options.preset, options.plan_only
        sent = client.send_goal_async(goal, feedback_callback=lambda msg:
            node.get_logger().info(f'{msg.feedback.stage}: {msg.feedback.active_arms} {msg.feedback.detail}'))
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected (busy or faulted); see commander log')
        if options.cancel_after is not None:
            def cancel():
                timer.cancel()
                handle.cancel_goal_async()
            timer = node.create_timer(options.cancel_after, cancel)
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future)
        outcome = future.result()
        result = outcome.result
        print(f'status={outcome.status}, error={result.error_code}, MoveIt={result.moveit_error_code}: {result.message}')
        print(f'executed={result.executed}, left_completed={result.left_completed}, right_completed={result.right_completed}')
        return 0 if outcome.status == GoalStatus.STATUS_SUCCEEDED else 1
    finally:
        if timer:
            timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
