# MoveArms action contract

Endpoint: `/arms/move`. Type: `dual_arm_interfaces/action/MoveArms`.
The authoritative definition is `dual_arm_interfaces/action/MoveArms.action`.

## Goal

| Field | Values / behaviour |
| --- | --- |
| `goal_type` | `preset` (default) or `explicit` |
| `preset` | Exact case-sensitive key in YAML, used for preset goals |
| `mode` | `left`, `right`, `both`, `sequential`; required for explicit goals |
| `target_type` | `pose`, `joints`; required for explicit goals |
| `sequential_order` | `left_first` (default), `right_first` |
| `left_target`, `right_target` | `geometry_msgs/PoseStamped`; metres, unit quaternion, nonempty TF frame, zero timestamp |
| `left_joints`, `right_joints` | `sensor_msgs/JointState`; exactly six named positions per active arm in radians, no velocity/effort targets |
| `velocity_scaling`, `acceleration_scaling` | Explicit goals: `(0,1]`; zero selects node defaults |
| `plan_only` | Default false; true plans but never executes |

Modes/type/order are trimmed and lowercased. Only required active-arm targets are
used. One target type applies to both arms. The goal contains fixed ROS fields;
there are no arbitrary dictionaries in the wire interface. The YAML joint preset
mapping is translated into the same internal representation as `JointState`.

Preset requests should contain only `goal_type`, `preset`, and optionally
`plan_only`. Mode/type/scaling overrides are rejected; the preset owns its
sequential order and targets. Explicit requests cannot specify a preset name.

## Lifecycle

1. Busy or faulted server rejects immediately; see server log for the reason
   (ROS goal rejection itself has no custom result payload).
2. An idle server accepts the goal, validates/resolves it and returns a detailed
   aborted result if invalid. Acceptance does not mean validation or motion success.
3. For poses, the commander obtains individual arm IK candidates, first from
   measured state then alternative bounded random seeds. Candidate IK itself does
   not check collisions; the MoveIt planner validates the complete joint-space
   motion. No candidate is sent directly to a controller.
4. Mode `both` plans the combined 12-joint group. Single/sequential steps use the
   respective 6-joint group and the current state of the other arm.
5. The commander publishes the plan, checks state freshness/drift and the live
   collision matrix, then requests execution through MoveIt's action interface.
6. It waits for execution's final status before returning success or proceeding
   to the next sequential step. No hidden queue or automatic execution retry exists.

The initial state for planning is always measured state, never the random IK seed
or the desired goal posture. MoveIt remains responsible for planning-scene and
controller execution validation. All 12 arm joints must have fresh state, even
for a single-arm goal, because the other arm is a potential obstacle.

## Feedback and result

Feedback fields: `stage` (`validating`, `planning`, `executing`), `active_arms`
(`left`, `right`, `left,right`, or empty), `detail` (human-readable).
No invented completion percentage or ETA is emitted.

| Result field | Meaning |
| --- | --- |
| `error_code` | Commander error classification below |
| `moveit_error_code` | Underlying MoveIt error when available, otherwise zero |
| `message` | Human-readable outcome |
| `left_completed`, `right_completed` | True only for a successfully completed execution involving that arm |
| `executed` | At least one requested execution completed successfully |

`executed=false` is **not** proof that no motion occurred: an aborted/canceled
trajectory can have moved partway. Read actual state after failures. Plan-only
success leaves all three booleans false. In simultaneous mode, a failed combined
execution cannot establish independent success for either arm, so both completion
flags remain false. Sequential partial success preserves the first arm's flag.

| Code | Name |
| --- | --- |
| 0 | `OK` |
| 1 | `INVALID_GOAL` |
| 2 | `NOT_READY` |
| 3 | `IK_FAILED` |
| 4 | `PLANNING_FAILED` |
| 5 | `EXECUTION_FAILED` |
| 6 | `CANCELED` |
| 7 | `TIMEOUT` |
| 8 | `INTERNAL_ERROR` |

Always check the ROS action terminal status as well. Normal success is
`STATUS_SUCCEEDED`; errors are `STATUS_ABORTED`; confirmed cancellation is
`STATUS_CANCELED`. A cancellation race can return canceled with completion flags
true if execution finished before it could be stopped.

## Cancellation, timeouts and ownership

Cancellation during IK waits does not start motion; it ends the parent goal and
ignores the pending service response. Cancellation during planning/execution is
forwarded to the downstream action. The commander does not release its busy slot
until that action terminates, or it latches a communication fault and rejects all
new requests. A late downstream acceptance is also sent a cancellation request.

If termination cannot be confirmed, the result is aborted with a timeout/error,
not a false claim of cancellation. Inspect the hardware/MoveIt state before
restarting the commander. Ordinary confirmed failures do not require restarting.

Only this server's requests are serialized. Other MoveIt clients, controller
clients or pendant motions can bypass this ownership. Do not run another motion
producer concurrently. Keep the mobile base stationary for arm execution. This
software interface does not replace the robot's protective-stop or emergency-stop
functions.

## Architecture references

- ROS actions: https://design.ros2.org/articles/actions.html
- MoveIt Humble action integration: https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

This implementation uses the public MoveIt action/service interfaces from Python,
so it does not need a separate MoveGroupInterface robot model inside a C++ worker.
The model supplied at launch is used for joint-bound checks and IK seed bounds.
