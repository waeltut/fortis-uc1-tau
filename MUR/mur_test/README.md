# mur_test

Interactive ROS 2 Humble coordinator for a multi-point manipulation workspace.
Requires Nav2, TF, mur_base_selector, MoveIt and dual_arm_commander.

## Build and run

Replace the packages `mur_base_selector`, `mur_base_selector_interfaces` and
`mur_test` in `~/ros2_ws/src/MUR`. Keep your generated dexterous cache and robot
configuration. The selector and test must use the same installed SelectBase interface. Stop the
selector while replacing/rebuilding its package, then source and restart it.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to mur_base_selector mur_test
source install/setup.bash
```

Start your usual robot, Nav2, MoveIt/commander and selector launches. Run the test
in its own interactive terminal (ROS launch does not reliably forward stdin):

```bash
ros2 run mur_test mur_test --ros-args \
  --params-file ~/ros2_ws/src/MUR/mur_test/config/mur_test.yaml
```

In another sourced terminal, start the height relay:

```bash
ros2 run mur_test pose_height_relay --ros-args -p height_m:=1.3
```

In Lichtblick, publish PoseStamped clicks to `/goal_pose_2d`. Each click becomes a
point on `/mur_test/point`, with its Z replaced by `height_m`. Frame, XY and full
orientation are preserved: the relay does not change tool direction. Use a unit
quaternion. Height can be changed between clicks:

```bash
ros2 param set /pose_height_relay height_m 1.1
```

## Terminal workflow

1. Place points. The test acknowledges each as P1, P2, … and asks for its arm.
   Enter accepts `any`; other choices are `left`, `right`, `both`. Incoming clicks
   are queued while you answer. Duplicate clicks count as separate points.
2. Type `done` after assigning all points. `clear` resets the collected set;
   `quit` exits. Points are anchored in the map before navigation.
3. The test calls `/base_selection/select` once with every point and requirement.
   It prints the shared base pose and resolved per-point arms. A failure sends
   no motion command and returns to collection.
4. Confirm navigation with `y`. An expired selection is recalculated and asks
   again with the fresh destination. No navigation occurs without confirmation.
5. After Nav2 succeeds, the test reports actual base arrival error and offers
   `P1`, `P2`, …, `home`, `new`, or `quit`. Selecting a point commands that arm;
   the menu repeats after each completed command or confirmed motion failure.
6. Every point command transforms the original map target using the latest actual
   base pose into `planning_frame`, then calls `/arms/move` with an explicit pose.
   Arrival error does not prevent attempting MoveIt planning.
7. `home` calls the same MoveArms action with `goal_type: preset`, `preset: home`.
   The commander owns the preset's joint values and motion mode. The test expects
   this preset to home both arms and checks completion of both.

| Requirement | Selection | At the point menu |
|---|---|---|
| any / empty | At least one arm must reach; selector chooses per point | Use returned arm |
| left | Left must reach | Move left |
| right | Right must reach | Move right |
| both | Each arm must individually reach from the same base | Ask left or right for this visit |

`both` never sends both TCPs to the same pose simultaneously. You can revisit the
point with either arm. MoveIt still checks the actual current robot state and
collision scene for every commanded motion. Selection uses sampled reachability,
not exact online IK or a guarantee that every arm trajectory will succeed.

Once `done` freezes the workspace, additional point clicks are ignored with a
warning. Choose `new` to collect another workspace from the current location.
This prevents an incidental click from changing an active navigation/arm request.
`quit` does not automatically home or drive the base back.

## Point input and selector service

The relay sends clicks to `/mur_test/point`. They are collected locally until
`done`; the selector is contacted only by the service request. The selector has
no legacy `/base_selection/target` input or selected_pose output. To select a
single point directly, call `/base_selection/select` with its `target` and `arm`
fields, or a one-element `targets` array. This does not command navigation.

To add a 3D point directly to the test without the relay:

```bash
ros2 topic pub --once /mur_test/point geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 1.3}, orientation: {y: 0.7071068, w: 0.7071068}}}'
```

## Configuration

Edit `config/mur_test.yaml`, or override parameters with `--ros-args -p name:=value`.

| Parameter | Default | Purpose |
|---|---|---|
| target_topic | /mur_test/point | Workspace point input |
| selection_service | /base_selection/select | Multi-point selector |
| selection_timeout | 60.0 | Response timeout in seconds; increase with selector budget |
| navigation_action | /navigate_to_pose | Nav2 action |
| commander_action | /arms/move | MoveArms action |
| base_goal_frame | map | Point anchor/navigation frame |
| base_frame | base_link | Arrival-error measurement frame |
| planning_frame | mur | Post-navigation arm target frame |
| plan_only | false | Plan arm/home commands only; base navigation still executes |
| velocity_scaling | 0.1 | Explicit arm-goal scaling; 0 uses commander default |
| acceleration_scaling | 0.1 | Explicit arm-goal scaling; 0 uses commander default |
| navigation_timeout | 180.0 | Navigation result timeout, seconds |
| arm_timeout | 180.0 | Commander planning/execution timeout, seconds |
| cancel_timeout | 20.0 | Terminal-result wait after cancel, seconds |
| interface_timeout | 15.0 | Discovery/goal acceptance timeout, seconds |
| commander_wait_timeout | 5.0 | Commander discovery timeout, seconds |
| tf_timeout | 5.0 | Latest-TF lookup timeout, seconds |

The relay has `input_topic` (default `/goal_pose_2d`), `output_topic` (default
`/mur_test/point`) and dynamic `height_m` (default 1.3). Topic changes need restart.
Home uses the commander's preset settings, not the explicit-goal scaling fields.
For sequential home presets, the commander may reject `plan_only`.

## Failure and cancellation

Motion timeout or Ctrl+C requests action cancellation and waits for a terminal
result. Unknown acceptance registers cancellation for a late response. Unconfirmed
cancellation/result latches a local fault and stops the loop. Inspect the robot
before restarting. These are action-protocol checks, not emergency-stop commands.
A selection timeout sends no motion but does not cancel server work. Wait for it
or explicitly call `/base_selection/cancel` before retrying. TF uses the latest
available sample without a separate age limit.

## Validation

```bash
python3 -m unittest discover -s src/MUR/mur_test/test -v
```

Offline tests exercise request arrays, arm assignment validation, collection,
workspace freezing, home/pose action payloads and action lifecycle handling.
ROS generated interfaces, DDS, live TF, Nav2 and physical execution need checking
on the platform; no live robot validation was performed in the editing environment.
