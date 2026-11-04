# Configuring motions

## Two files

- `dual_arm_commander/config/presets.yaml`: ordinary YAML, top-level `presets`.
- `dual_arm_commander/config/commander.yaml`: ROS parameter YAML, node key
  `dual_arm_commander`, then `ros__parameters`.

Defaults are installed by colcon. Prefer an external file via `presets_file:=...`
for ongoing tuning. Files are validated once at startup, not reloaded while moving.
Duplicate YAML keys, malformed presets and out-of-URDF-bounds joint presets fail
startup with an error. A valid configuration is not a guarantee of reachability or
collision-free motion; planning happens for every request.

## Included presets

| Preset | Target type | Source |
| --- | --- | --- |
| `home` | Pose, both arms | Previous commander home: left `(0.6,0.15,1.2)`, right `(0.6,-0.15,1.2)` in `mur`, identity orientations |
| `home_joints` | Joints, both arms | Existing left/right SRDF `home` states |
| `ready_joints` | Joints, both arms | Existing left/right SRDF `ready` states |

The SRDF joint home is a different posture from the old Cartesian home. Do not
assume these two presets are equivalent. Joint presets choose a posture without
IK ambiguity. Changing a YAML preset does not change an SRDF named state.

## Add a pose preset

Add the following under `presets:`. These are example coordinates to review for
your setup, not a hardware-validated target:

```yaml
presets:
  inspect_left:
    mode: left
    target_type: pose
    velocity_scaling: 0.10
    acceleration_scaling: 0.10
    left_target:
      frame_id: mur
      position: [0.6, 0.15, 1.2]
      orientation: [0.0, 0.0, 0.0, 1.0]
```

Positions are metres. Orientations are unit quaternions `[x,y,z,w]`, not Euler
angles. `mur` is relative to the robot; `map` is relative to the warehouse and
requires a valid TF chain into MoveIt's planning frame. Poses are resolved using
latest TF during IK at execution of the task step; keep the base stationary during
arm planning/execution. Explicit goal stamps must be zero. Sequential step two
resolves its target after step one finishes.

For a two-arm preset, add `right_target` and use `mode: both` or `mode: sequential`.
For sequential movement add `sequential_order: left_first` or `right_first`.

## Add a joint preset

Use a mapping of all six named joints for each selected arm. Copy a complete
measured posture from `/joint_states`, not assumed array indices. Values are radians.
For example, this left posture is copied from the existing SRDF `ready` state:

```yaml
presets:
  left_ready:
    mode: left
    target_type: joints
    velocity_scaling: 0.10
    acceleration_scaling: 0.10
    left_joints:
      left_shoulder_pan_joint: 0.0
      left_shoulder_lift_joint: -1.5707
      left_elbow_joint: 1.5707
      left_wrist_1_joint: 0.0
      left_wrist_2_joint: 1.5707
      left_wrist_3_joint: -0.7854
```

To teach your own home: move the robot to the desired posture using your existing
approved method, read `/joint_states --once`, copy each side's six name/position
pairs into the preset, and review a fresh plan. No trajectory is recorded or reused.

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /left_arm/pose --once
ros2 topic echo /right_arm/pose --once
```

To make the default `home` joint-based, replace its entry with your named-joint
mapping (or the reviewed `home_joints` entry). Its action name stays `home`.

The YAML examples each show a complete top-level document for clarity. In one
file, have only **one** `presets:` key and put all entries beneath it.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `velocity_scaling` | `0.10` | Default fraction of configured joint velocity limits |
| `acceleration_scaling` | `0.10` | Default fraction of configured joint acceleration limits |
| `planning_time` | `5.0` s | MoveIt planning time per candidate |
| `planning_attempts` | `5` | MoveIt planner attempts per candidate |
| `ik_attempts` | `8` | Candidate IK seeds; measured seed first, then random bounded seeds |
| `ik_timeout` | `0.5` s | Requested per-arm IK timeout |
| `planning_budget` | `30.0` s | Budget for candidate search per step; a pending IK service call can extend this by `response_timeout` |
| `execution_timeout` | `180.0` s | Maximum wait for a downstream execution before cancellation |
| `response_timeout` | `10.0` s | Service response timeout |
| `cancel_timeout` | `10.0` s | Action acceptance timeout and cancellation-terminal-result grace period |
| `state_max_age` | `1.0` s | Maximum joint-state receive/stamp age |
| `joint_tolerance` | `0.001` rad | Joint planning goal tolerance |
| `start_tolerance` | `0.02` rad | Maximum change during planning before requiring a new request |
| `planner_id` | `RRTConnectkConfigDefault` | Existing OMPL planner |
| `pipeline_id` | `ompl` | MoveIt pipeline |

Scaling must be in `(0,1]`. Omitting a value or using zero selects the node default
for explicit requests and preset definitions. It never means "zero speed".
Per-goal values on a preset request are rejected; edit the preset instead.

Controller, payload, joint-limit and calibration configuration remain in their
original files. Scaling multiplies configured limits; it does not define those
limits. The existing model uses its supplied calibration configuration.

Group names are launch arguments. TCP links and ROS endpoint/topic names can be
changed in `commander.yaml`. The robot's joint prefixes remain `left_` and `right_`;
this package is not a generic arbitrary-joint robot adapter.

## Failures and cancellation

Planning/IK errors, invalid requests and terminal execution failures abort that
goal. They do not permanently disable subsequent requests. A caller should inspect
the error before retrying; there are no automatic execution retries.

If downstream acceptance or termination cannot be established, the commander
latches a fault and rejects new goals. Inspect/stop any still-running execution
and restart bringup after resolving the communication problem. This is deliberately
different from an ordinary, confirmed terminal MoveIt failure.

Cancellation is a software request through MoveIt, not a safety-rated emergency
stop. The server waits for the downstream terminal result, not merely a cancel
acknowledgement. Robot safety functions remain independent.
