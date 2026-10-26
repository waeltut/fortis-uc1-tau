# mur_test — manually confirmed Nav2 → fresh IK → MoveIt 2

ROS 2 **Humble**, Python `ament_python` package. Designed to be run in its **own interactive terminal**, alongside the existing MUR/MiR drivers, TF, Nav2, MoveIt 2 and base selector. Neither navigation nor arm movement happens without typing `y` in the corresponding terminal prompt. By default, `Enter` answers **no**.

## Sequence

1. Listen for the desired arm TCP target on `/base_selection/target` (`geometry_msgs/msg/PoseStamped`).
2. Listen for the selector's **chosen MiR base pose** (`geometry_msgs/msg/PoseStamped`; default `/base_selection/selected_pose`, **placeholder—verify actual selector output**).
3. Print both poses and prompt `Navigate MiR to this selected base pose? [y/N]`.
4. If yes, transform the selected base pose to `map` and send a Nav2 `/navigate_to_pose` action goal; wait for **SUCCEEDED**. Do **not** plan arm motion if Nav2 fails.
5. After arrival, transform the **original TCP target** into MoveIt's planning frame using **current TF**, and call `/compute_ik` for a fresh collision-aware arm joint solution using **current** `/joint_states`.
6. Submit the resulting arm joint goal to MoveIt 2 `/move_action` with `plan_only=true`. Do **not** execute yet. Report IK joint values and trajectory length/time.
7. Prompt `Execute this planned ARM trajectory? [y/N]`. If yes, verify the robot still matches the planned start state and send the exact planned trajectory to MoveIt 2 `/execute_trajectory`.
8. On failure or declined action, abort this cycle. Publish a new TCP target for another test.

This deliberately recomputes IK **after** navigation; it does not rely on cached candidate IK from the base selector. The arm goal is expressed in the MoveIt `world` frame (configurable). Provided the TF tree connects `map → ... → world` / `chest`, transforming the original TCP pose after navigation supplies the correct post-navigation goal coordinates.

## Important integration requirement

**Check the actual `mur_base_selector` output before using this package.** This code assumes the selected base pose is a `geometry_msgs/msg/PoseStamped`. If the selector publishes only a `MarkerArray`, `PoseArray`, diagnostics, or another message type, `mur_test` cannot recover the selected base pose from it. Add a dedicated `PoseStamped` output publisher to the selector (recommended) or write an adapter. Do NOT subscribe to `/base_selection/target` as the selected base pose: that is an **arm TCP target**, not a driving goal.

The package also requires an active **standard MoveIt 2 move_group server** exposing `/compute_ik`, `/move_action`, and `/execute_trajectory`. A custom `/dual_arm_moveit_commander` node alone may not expose those standard interfaces. If your installation has only the custom commander, adapt the MoveIt portions to its service/action API, or bring up `move_group` with your dual-arm config. **The package has not been hardware-tested on your MUR platform.**

## Install/build

Extract into your ROS 2 workspace, so the layout is:

```text
~/ros2_ws/src/MUR/mur_test/
  package.xml
  setup.py
  setup.cfg
  config/mur_test.yaml
  launch/mur_test.launch.py
  mur_test/test_node.py
```

Build and source:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select mur_test
source install/setup.bash
```

Start robot drivers/TF, Nav2, MoveIt and your selector **first**. Inspect their interfaces:

```bash
ros2 topic list -t | grep -E '(base_selection|joint_states)'
ros2 action list -t | grep -E '(navigate_to_pose|move_action|execute_trajectory)'
ros2 service list -t | grep compute_ik
ros2 topic info /base_selection/selected_pose -v
```

Edit `config/mur_test.yaml` (and **rebuild/source** after changing installed config when not symlinked) so `selected_pose_topic` matches the actual selected-pose publisher, then run in **a real interactive terminal**:

```bash
ros2 launch mur_test mur_test.launch.py
```

Or run with an override without modifying YAML:

```bash
ros2 run mur_test mur_test --ros-args \
  --params-file ~/ros2_ws/src/MUR/mur_test/config/mur_test.yaml \
  -p selected_pose_topic:=/my_actual_selected_pose
```

If `ros2 launch` runs under a terminal or remote launcher where stdin is not forwarded, use the `ros2 run` form above in a dedicated terminal (not as a background process). The console confirmations need stdin.

## Example target

**This is an illustrative target only, not a verified safe goal.** Publish using your usual selector input:

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}}'
```

After the selector publishes the *chosen base PoseStamped*, the first console prompt appears. If the selector uses transient-local durability, `mur_test` uses a normal reliable subscriber; run this node **before** publishing a new TCP target to avoid mismatched old selections. It clears any cached selection every time a new TCP target arrives.

For a conservative test, choose **no** at both prompts initially, then test Nav2 with no arm execution and finally test the full pipeline in a controlled, cleared workspace with emergency stop available.

## Parameters

Set in `config/mur_test.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `target_topic` | `/base_selection/target` | TCP target PoseStamped |
| `selected_pose_topic` | `/base_selection/selected_pose` | **Verify and change:** actual chosen base PoseStamped |
| `joint_states_topic` | `/joint_states` | Current arm positions |
| `navigation_action` | `/navigate_to_pose` | Nav2 action |
| `base_goal_frame` | `map` | Nav2 goal frame |
| `navigation_timeout` | `180.0` | Maximum travel waiting time, seconds |
| `ik_service` | `/compute_ik` | MoveIt IK service |
| `move_group_action` | `/move_action` | MoveIt joint path planning action |
| `execute_action` | `/execute_trajectory` | MoveIt trajectory execution action |
| `arm_group` | `left_ur_manipulator` | MoveIt SRDF planning group |
| `tcp_link` | `left_tcp` | MoveIt end-effector link |
| `planning_frame` | `world` | Target frame passed to IK |
| `planning_pipeline_id` | `''` | Use MoveIt configured default |
| `planner_id` | `''` | Use MoveIt configured default |
| `allowed_planning_time` | `10.0` | MoveIt planning time budget, seconds |
| `velocity_scaling` | `0.1` | Arm motion speed scaling |
| `acceleration_scaling` | `0.1` | Arm acceleration scaling |
| `ik_timeout` | `3.0` | IK search timeout, seconds |
| `ik_attempts` | `5` | Used if installed request definition supports it |
| `ik_avoid_collisions` | `true` | Reject collision IK solutions |
| `joint_goal_tolerance` | `0.01` | Joint goal tolerance, radians |
| `max_joint_state_age` | `3.0` | Reject stale state messages, seconds |
| `max_planned_start_deviation` | `0.10` | Reject execution when joints drift from planned start, radians |
| `interface_timeout` | `15.0` | Wait for interfaces, seconds |
| `planning_timeout` | `45.0` | Total wait for planning result, seconds |
| `execution_timeout` | `120.0` | Arm execution wait, seconds |
| `pose_wait_timeout` | `30.0` | Wait for incoming goal + selected pose, seconds; retries |
| `tf_timeout` | `5.0` | Wait for frame conversion, seconds |

For the right arm change at least `arm_group: right_ur_manipulator` and `tcp_link: right_tcp`.

## Safety / limits

- This is a **test orchestrator**, not a certified safety controller. Always supervise the physical robot, check the selected destination and use the hardware safety system.
- Plan validity is checked by MoveIt at planning time; the physical environment may change before execution. The code checks trajectory start joint deviation but is **not** a substitute for current collision checking or safe operating procedures.
- Nav2 executes a navigation goal and may **re-plan**; it does not execute the exact path used during candidate ranking.
- Moving `map`/`odom` frames and TF timestamp behaviour depend on your robot; this test uses **latest available TF** after navigation. Verify transformation consistency on your system.
- This implementation expects **one target at a time** and a newly selected pose after it. New targets received during execution abort subsequent stages but cannot undo movement already started. Action timeouts request cancellation, and you must still verify the robot has stopped.
- The IK service returns a candidate joint configuration; MoveIt then independently plans a collision-checked path to it. An IK solution alone does not guarantee a valid motion plan.
- No automatic return-to-home or automatic arm movement is enabled.
