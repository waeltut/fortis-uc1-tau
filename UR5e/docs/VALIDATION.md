# Validation and commissioning

## Checks performed for this delivery

- 36 pytest cases passed: preset validation, downstream lifecycle and action
  orchestration with stubbed ROS imports. These do not replace ROS integration tests.
- Python source/launch syntax checked.
- XML/YAML parsed; package layout and example command field names checked.
- External UR driver and hand-control files compared against the uploaded archive.
- No ROS 2 installation was available in the delivery environment. A ROS interface
  generation/build, fake-hardware run and real-robot test remain unverified.

## Build and unit tests on Humble

After following INSTALL.md:

```bash
cd ~/ros2_ws
colcon test --packages-select dual_arm_commander
colcon test-result --verbose
ros2 interface show dual_arm_interfaces/action/MoveArms
```

The pure-Python tests also run without ROS:

```bash
python3 -m pytest -q src/UR5e/dual_arm_commander/test
```

## Fake-hardware integration

Use a separate ROS domain from the real robot during testing; set the same domain
in every test terminal. Example:

```bash
export ROS_DOMAIN_ID=68
source ~/ros2_ws/install/setup.bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py use_fake_hardware:=true
```

Wait for controller startup and state topics. In a second terminal with the same
setup and domain, perform these checks:

1. `ros2 action info /arms/move` shows one server.
2. `ros2 topic echo /left_arm/pose --once` and right equivalent return current poses.
3. Run `move_preset home --plan-only`; result should show no executed/completed flags.
4. Run left and right explicit examples. Only the requested arm should move.
5. Run `home` or a reviewed reachable joint preset with mode `both`. Observe a
   single 12-joint trajectory and success after both controllers complete.
6. Run the sequential example with both orders. Feedback must show first execution
   ending before second planning begins.
7. While a sufficiently long request runs, submit another. It must be rejected,
   not queued or substituted for the first.
8. Run `move_preset home --cancel-after 0.2`. If still active, it should terminate
   canceled and stop further steps. A completed request can legitimately win the
   cancellation race; use a longer motion to exercise execution cancellation.
9. Send a nonexistent preset and an invalid quaternion; expect aborted result
   `INVALID_GOAL`, with no motion, and verify a later valid request still works.
10. Use an unreachable target; expect IK/planning failure without execution. Verify
    a later valid goal is accepted.
11. For sequential partial failure, give the first arm a reachable target and the
    second an unreachable one. Result should preserve first-arm completion only.
12. Stop the test MoveIt server during an active action. Verify timeout/cancellation
    handling and that an unconfirmed downstream state locks out further goals.
    Restart the complete fake bringup after this fault-injection check.

If home fails due to a collision, inspect the reported contact pair and model.
The cross-arm exclusions removed from the supplied SRDF can reveal a previously
hidden collision. Do not count a planning failure as a passed movement test.

Fake hardware checks the ROS orchestration and planning setup; it does not validate
physical clearance, calibration, timing under UR speed scaling, or grasp behaviour.

## Real hardware

Restore your normal ROS domain, stop all test/old command nodes, and verify arm
identity, tool/payload configuration, scene geometry and current pose. Begin with
plan-only requests and low scaling. Check that both External Control programs and
required controllers are ready. Keep the base stationary while commanding arms.

Verify cancellation and result timing under your actual speed scaling, and choose
`execution_timeout` accordingly. Confirm the removal of hand side effects from
`home` is reflected in your task coordinator.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Action type not found | Build/source `dual_arm_interfaces`; use a fresh terminal |
| Old topic goals do nothing | They were removed; migrate caller to `/arms/move` |
| Goal rejected | Another goal is active, or downstream uncertainty latched a fault; see log |
| Missing/stale joint | Joint-state broadcaster, controller state, time synchronisation, domain |
| Service/action unavailable | MoveIt startup, namespace and configured endpoint names |
| Cross-arm collision checking disabled | Live ACM or a stale SRDF installation; rebuild/source/restart |
| Start state in collision | Inspect actual contacts and robot geometry/posture |
| Robot moved during planning | Another command source/base movement/state change; submit fresh goal |
| Execution failure | Controller activation, pendant External Control, protective stop, trajectory tolerance |
| Joint target rejected | All six names per arm, radians, URDF/soft limits |
| TF/IK failure | Target frame, latest TF chain, tool link, target reachability/orientation |
| Preset edits ignored | Correct `presets_file` path and restart; no live reload |

A failed motion's completion flags are not a measurement of how far it moved.
Read current joint/TCP state before deciding what to do next.
