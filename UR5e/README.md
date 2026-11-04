# Dual UR5e — action-based motion

ROS 2 Humble / MoveIt 2 packages for the MiR-mounted dual UR5e robot.
One application motion endpoint: **`/arms/move`**, type
**`dual_arm_interfaces/action/MoveArms`**. It plans and executes one request,
provides progress, supports cancellation, and returns a final result. It has no
motion queue. The calling coordinator owns sequences and retries.

## Start here

1. Follow [installation and migration](docs/INSTALL.md).
2. Review [preset and parameter configuration](docs/CONFIGURATION.md).
3. Use the copyable [command examples](examples.md).
4. Integrate using the [action contract](docs/API.md).
5. Run the [validation procedure](docs/VALIDATION.md).

## Packages

| Package | Responsibility |
| --- | --- |
| `dual_arm_interfaces` | Generated `MoveArms.action`; clients only need this interface package |
| `dual_arm_commander` | Action server, preset parser, MoveIt coordination, preset CLI client and tests |
| `dual_arm_driver` | Existing public launch, TCP pose publisher, payload initializer |
| `duo_ur` | Robot geometry, meshes, SRDF, MoveIt and ros2_control configuration |
| `hand_control` | Original finger interface, unchanged |
| `Universal_Robots_ROS2_Driver` | Supplied external driver tree, byte-for-byte unchanged |

## Launch

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py
```

This starts hardware, MoveIt, the action commander, state publishing, payload setup
and the existing hand nodes. Startup sends **no arm motion**.

For fake hardware (no physical arm or hand connection):

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py use_fake_hardware:=true
```

Fake hardware uses wall time and ordinary joint trajectory controllers. Real
hardware uses the scaled trajectory controllers. No RViz is required.

```bash
ros2 action send_goal /arms/move dual_arm_interfaces/action/MoveArms \
  '{goal_type: preset, preset: home}' --feedback
```

`home` preserves the previous Cartesian home positions. It no longer moves fingers.
`home_joints` and `ready_joints` are separate presets copied from existing SRDF
states; review them before use. Details and configuration are below the links above.

## Modes

| Mode | Meaning |
| --- | --- |
| `left` | Left arm only |
| `right` | Right arm only |
| `both` | One combined 12-joint plan, executed through both controllers |
| `sequential` | First arm completes, then second is planned from updated state |

Mode, goal type, target type and sequential order are case-insensitive.
Preset names are case-sensitive. Both arms share the same target type per goal.
Simultaneous execution is coordinated by MoveIt; this is not a guarantee of exact
physical synchronisation if the two UR controllers apply different speed scaling.

## State topics retained

- `/left_arm/pose`, `/right_arm/pose`: TCP `PoseStamped`, default frame `mur`, 10 Hz.
- `/joint_states`, `/tf`, `/tf_static`: existing robot state.
- Existing TCP broadcaster/controller telemetry remains available.
- `/arms/planned_trajectory`: most recent generated trajectory for each step.

Removed: arm `goal_pose` subscriptions, automatic pairing, per-arm queues,
`/dual_arm/go_home`, home status topic and finger side effects of home.
There are no application arm-motion services. MoveIt/controller internal services
and actions remain necessary; the action API does not disable them or prevent
other clients from bypassing it. Keep one motion owner.

Hand-control commands remain their existing topics; this action controls arms
only. Use `start_hands:=false` if another system manages the hands.

## Changes to collision checking

Nine left/right collision exclusions in the supplied SRDF were removed. The
commander checks the live MoveIt allowed-collision matrix before planning and
again before execution; cross-arm exclusions cause a clear refusal. Other SRDF
exclusions, geometry, TCP transforms, calibration settings, controller settings,
payload setup and driver files were not changed. A resulting start-state collision
must be investigated in the model or physical posture; do not hide it by disabling
cross-arm collision checking.

## Validation status

Python/configuration and downstream lifecycle tests are included and were run in
the delivery environment. ROS 2/MoveIt are not installed there, so a Humble colcon
build, fake-hardware integration and physical motion have **not** been run. Follow
`docs/VALIDATION.md` before deploying on hardware.
