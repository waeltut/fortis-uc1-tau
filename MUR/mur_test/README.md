# mur_test

ROS 2 Humble coordinator for MiR navigation followed by the selected UR5e arm.
Requires Nav2, TF, mur_base_selector, MoveIt and dual_arm_commander running.

## Sequence

1. Receive `/base_selection/target` and `/base_selection/selected_pose` (PoseStamped).
2. Ask for navigation confirmation in the terminal.
3. Match the selected base against JSON on `/base_selection/result` or
   `/base_selection/candidate_diagnostics`, and lock its `left` or `right` arm.
   `both` is rejected: this test supplies one TCP target.
4. Anchor the TCP target in `map` before driving; execute `/navigate_to_pose`.
5. After Nav2 succeeds, log base arrival error and transform the anchored TCP
   target using the actual latest TF into `mur`. Arrival error does not block IK.
6. Automatically send `dual_arm_interfaces/action/MoveArms` to `/arms/move`:
   `goal_type: explicit`, `mode: left|right`, `target_type: pose`, and the selected
   arm's target with a zero timestamp, as required by the commander.
7. Print commander feedback and wait for its terminal result. Execution succeeds
   only when action status, error code, executed flag and selected-arm completion
   agree. IK, planning, collision checking and trajectory execution belong to
   dual_arm_commander.

No arm confirmation prompt or opposite-arm fallback is used. With `plan_only:
true`, navigation still executes but the commander only plans the arm movement.

## Build and run

Replace `~/ros2_ws/src/MUR/mur_test` with this package. Build the new interface
package first if it is not already installed:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dual_arm_commander mur_test
source install/setup.bash
ros2 action list -t
```

The action list should include `/arms/move [dual_arm_interfaces/action/MoveArms]`
and `/navigate_to_pose [nav2_msgs/action/NavigateToPose]`.
Start this node in a separate interactive terminal **before publishing a target**:

```bash
ros2 run mur_test mur_test --ros-args \
  --params-file ~/ros2_ws/src/MUR/mur_test/config/mur_test.yaml
```

Use the usual selector target publisher. For example, with coordinates adjusted
for your scene:

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}}'
```

The launch file accepts `config:=/absolute/path.yaml`. ROS launch does not reliably
forward interactive stdin; use `ros2 run` for confirmation. To run without console
input, explicitly set `confirm_navigation: false` in that config. This allows
both navigation and arm dispatch automatically when a selection arrives.

## Configuration

All settings are in `config/mur_test.yaml`.

| Parameter | Default | Purpose |
|---|---|---|
| target_topic | /base_selection/target | Requested TCP PoseStamped |
| selected_pose_topic | /base_selection/selected_pose | Selected base PoseStamped |
| selection_diagnostics_topic | /base_selection/candidate_diagnostics | Selector JSON String; blank disables |
| selection_result_topic | /base_selection/result | Selector JSON String; blank disables |
| navigation_action | /navigate_to_pose | Nav2 action |
| commander_action | /arms/move | MoveArms action |
| base_goal_frame | map | Anchor frame and navigation frame |
| base_frame | base_link | Frame used to measure arrival error |
| planning_frame | mur | Post-navigation TCP target frame |
| confirm_navigation | true | Ask before moving the base |
| plan_only | false | Plan arm without executing; base still moves |
| velocity_scaling | 0.1 | Arm velocity scaling; 0 uses commander default |
| acceleration_scaling | 0.1 | Arm acceleration scaling; 0 uses commander default |
| navigation_timeout | 180.0 | Navigation result timeout, seconds |
| arm_timeout | 180.0 | Entire commander planning/execution timeout, seconds |
| cancel_timeout | 20.0 | Wait for terminal result after cancel, seconds |
| interface_timeout | 15.0 | Nav2 discovery and goal acceptance timeout, seconds |
| commander_wait_timeout | 5.0 | Commander discovery timeout, seconds |
| pose_wait_timeout | 30.0 | Incoming target/selection polling window, seconds |
| arm_selection_timeout | 4.0 | Wait for matching selector metadata, seconds |
| tf_timeout | 5.0 | Latest-TF lookup timeout, seconds |

Set arm_timeout to cover the commander's planning and execution budgets.
Other MoveIt parameters belong in dual_arm_commander's configuration, not here.
No direct IK, MoveGroup or ExecuteTrajectory client is used by mur_test.

## Cancellation and matching

A new target during motion requests cancellation and blocks the stale arm step.
Timeouts and Ctrl+C request cancellation and wait for a terminal result. Unknown
acceptance registers cancellation for a late response. Unconfirmed cancellation
or an unavailable result latches a local fault and ends the test loop; inspect
motion before restarting. Normal confirmed failures allow another target.
These checks do not command an emergency stop.

Selector JSON must contain `arm` and `pose: [x, y, yaw]`, optionally `frame_id`.
The match uses 3 cm / 5 degree tolerance and arrival timestamps within 5 seconds.
The topic protocol has no request ID, so use one selection request at a time;
identical targets are ignored while a request is pending. Publish a fresh target
after each completed or failed cycle. TF lookup uses the latest available sample,
without a separate age limit.

## Lichtblick height relay

The independent relay preserves frame, timestamp, XY and orientation while setting Z:

```bash
ros2 run mur_test pose_height_relay --ros-args -p height_m:=1.3
ros2 param set /pose_height_relay height_m 1.1
```

Defaults: input `/goal_pose_2d`, output `/base_selection/target`.
Override with `-p input_topic:=...` / `-p output_topic:=...` at startup.

## Validation

Offline tests cover action lifecycle and generated arm requests. Real ROS action,
TF and physical robot execution must be checked on the platform.
