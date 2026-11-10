# Action examples

Source `~/ros2_ws/install/setup.bash` in each terminal. Start the new bringup first.
Coordinates below reproduce the existing home values or illustrate the API;
review the target and use fake hardware/plan-only before physical execution.
All numeric YAML values use decimal points.

## Named preset

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: preset, preset: home}' --feedback
```

Short client (same action, not a separate command path):

```bash
ros2 run dual_arm_commander move_preset home
ros2 run dual_arm_commander move_preset home_joints --plan-only
```

`home` uses the original Cartesian home positions. `home_joints` is the different
joint posture from the original SRDF. Neither controls fingers.

## Left TCP pose

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: explicit, mode: left, target_type: pose,
    left_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: 0.15, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    velocity_scaling: 0.1, acceleration_scaling: 0.1}' --feedback
```

## Right TCP pose

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: explicit, mode: right, target_type: pose,
    right_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: -0.15, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    velocity_scaling: 0.1, acceleration_scaling: 0.1}' --feedback
```

## Both simultaneously

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: explicit, mode: both, target_type: pose,
    left_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: -0.15, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    right_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: -0.45, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    velocity_scaling: 0.1, acceleration_scaling: 0.1}' --feedback
```

This generates a single combined joint-space plan, not two independent concurrent
plans. It does not enforce a fixed relative transform between the two TCPs along
the path. It is not a rigid-object cooperative carrying controller.

## Both sequentially, right first

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: explicit, mode: sequential, sequential_order: right_first,
    target_type: pose,
    left_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: 0.15, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    right_target: {header: {frame_id: mur}, pose: {
      position: {x: 0.6, y: -0.15, z: 1.2},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
    velocity_scaling: 0.1, acceleration_scaling: 0.1}' --feedback
```

The second arm is planned only after the first execution succeeds. If the second
fails, there is no rollback of the first arm.

## Explicit joint target

The named-joint order can vary; the name/position pairing must match. This sample
uses your existing SRDF left `ready` posture, in radians:

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: explicit, mode: left, target_type: joints,
    left_joints: {
      name: [left_shoulder_pan_joint, left_shoulder_lift_joint, left_elbow_joint,
             left_wrist_1_joint, left_wrist_2_joint, left_wrist_3_joint],
      position: [0.0, -1.5707, 1.5707, 0.0, 1.5707, -0.7854]},
    velocity_scaling: 0.1, acceleration_scaling: 0.1,
    plan_only: true}' --feedback
```

For both arms, use `mode: both` and include an analogous `right_joints` field with
all six right joint names and positions. Presets are more convenient for repeated
12-joint targets.

## Plan without execution

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: preset, preset: home, plan_only: true}' --feedback
```

Success means planning succeeded. Result `executed`, `left_completed` and
`right_completed` remain false. Sequential plan-only requests are rejected with
`INVALID_GOAL`: use fake hardware to test the whole sequence.

Monitor planned trajectories before sending a goal (topic is not latched):

```bash
ros2 topic echo /arms/planned_trajectory
```

## Cancellation

The included client retains its goal handle and can request cancellation:

```bash
ros2 run dual_arm_commander move_preset home --cancel-after 2.0
```

It sends the request, waits two seconds from acceptance and requests cancellation,
then waits for a final result. If the action already finished, there is nothing to
cancel. Test cancellation using fake hardware first. Closing a CLI terminal is not
a cancellation protocol and must not be treated as stopping the robot.

## Read state

```bash
ros2 topic echo /left_arm/pose --once
ros2 topic echo /right_arm/pose --once
ros2 topic echo /joint_states --once
ros2 action info /arms/move
ros2 interface show dual_arm_interfaces/action/MoveArms
```

## Python coordinator

The runnable preset client is in
`dual_arm_commander/dual_arm_commander/client.py`. This is the core pattern inside
a node running under an executor (use a reentrant callback group for a coordinator
that awaits actions from its callbacks):

```python
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from dual_arm_interfaces.action import MoveArms

# In your node's __init__:
self.arm_group = ReentrantCallbackGroup()
self.arm_client = ActionClient(
    self, MoveArms, '/arms/move', callback_group=self.arm_group)

# As an async method called by your running ROS executor:
async def move_home(self):
    goal = MoveArms.Goal()
    goal.goal_type = 'preset'
    goal.preset = 'home'
    handle = await self.arm_client.send_goal_async(goal)
    if not handle.accepted:
        raise RuntimeError('Commander busy or faulted')
    self.current_arm_goal = handle  # Save for an external cancellation request.
    outcome = await handle.get_result_async()
    self.current_arm_goal = None
    if outcome.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(outcome.result.message)
    return outcome.result
```

To cancel from another callback, call
`self.current_arm_goal.cancel_goal_async()` and still wait for the original final
result. A cancellation acknowledgement alone does not complete your task step.
For dynamically selected poses, set `goal_type='explicit'`, `mode='left'` or
`'right'`, `target_type='pose'` and the appropriate `PoseStamped` target. Keep its
stamp zero. Wait for navigation success before sending it. Do not send the next
arm goal until the current one terminates.

## Existing hands (separate from arm actions)

These are retained unchanged for compatibility. They publish desired finger
positions; publishing does not confirm physical completion:

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.9, 0.0, 1.0, 1.0, 1.0, 1.0]}'
ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.9, 0.0, 1.0, 1.0, 1.0, 1.0]}'
```
