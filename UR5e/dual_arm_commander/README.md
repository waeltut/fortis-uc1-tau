# Dual-arm action commander

Single-arm requests first use normal IK and MoveIt planning. Successful normal
plans do not run the avoidance search or its native distance checker.

## Simultaneous blocking-arm assistance

After a failed single-arm plan, `/check_state_validity` must confirm a valid
current state and a target whose reported contacts are exclusively cross-arm.
Diagnosis stops at the first verified target. IK-only failures, ambiguous contacts,
missing services and explicit `both`/`sequential` requests do not trigger assistance.

The fallback searches small joint adjustments of the other arm, smallest first.
It retains the requested arm's joint/IK target. Candidates that fail endpoint
clearance are discarded before planning. MoveIt then plans a combined trajectory
for `both_ur_manipulator`, with all twelve joint goals specified. The native
checker validates sampled positions along that combined trajectory before it is
accepted. The first acceptable plan wins.

The commander sends **one combined trajectory in one execution call**. Both arms
can move simultaneously; there is no retreat-first phase or post-retreat replan.
The avoiding arm stays at its new endpoint. `plan_only` publishes the complete
combined trajectory without execution. Result completion flags refer to the
explicitly requested goals; feedback identifies both moving arms.

## Clearance and computation

The default additional clearance is 0.05 m between left/right URDF collision
shapes, including modelled hands. Geometry is cached on first fallback use.
Distance queries run locally, with no per-candidate contact/cost service calls.
The checker does not include attached objects or world obstacles; MoveIt keeps
its normal collision checks for these.

The combined path is sampled using linear, cubic or quintic joint interpolation
according to the waypoint positions/velocities/accelerations. A derivative bound
sets sample density and includes spline overshoot. The check includes the measured
start, intermediate samples and the actual planned endpoint. If the initial arm
separation is below the configured margin, the fallback can fail without moving.
No margin is silently relaxed. This is sampled validation of the nominal combined
trajectory, not continuous collision detection or proof of physical tracking.

Separate UR scaled controllers can progress differently. The nominal combined
path check does not guarantee clearance under arbitrary relative delays or
independent speed scaling. A physical collision's cause cannot be determined from
these source changes alone. Model accuracy, controller/joint mappings and runtime
logs must be checked when physical behaviour disagrees with the plan. Modelled
hands are fixed shapes and do not follow actual finger articulation.

The existing 15-second fallback budget includes diagnosis, candidate search and
path checks. The first valid plan is used; there is no clearance optimisation loop.
Candidate and sample caps bound work. Exhaustion fails rather than skipping checks.
Cancellation/termination can extend wall time beyond the planning allowance.
Normal successful plans retain their original fast path and have no extra checks.

## Configuration

Edit `config/commander.yaml`, rebuild and restart bringup.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `auto_move_blocking_arm` | `true` | Enable assistance for single-arm requests |
| `avoidance_clearance` | `0.05` | Cross-arm margin at endpoint and sampled combined-path states, metres; `0` disables extra distance checks |
| `avoidance_budget` | `15.0` | Fallback planning/checking allowance, seconds |
| `avoidance_max_joint_offset` | `0.7` | Maximum final displacement per avoiding-arm joint, radians |
| `avoidance_max_candidates` | `96` | Candidate cap; current generator produces at most 72 |
| `avoidance_path_joint_step` | `0.01` | Maximum per-joint interpolation variation per sample interval, radians |
| `avoidance_path_max_samples` | `5000` | Maximum samples per combined trajectory; exceeding it rejects the plan |
| `state_validity_service` | `/check_state_validity` | MoveIt whole-robot contact diagnosis |

Joint displacement limits constrain candidate endpoints, not every path point.
The normal `planning_budget` is separate from the fallback allowance.
`/arms/planned_trajectory` publishes both arms when fallback succeeds.

## Build and validation

```bash
cd ~/ros2_ws
rosdep install --from-paths src/UR5e/dual_arm_commander --ignore-src -r -y
colcon build --packages-select dual_arm_commander dual_arm_driver --symlink-install --cmake-force-configure
source install/setup.bash
colcon test --packages-select dual_arm_commander
colcon test-result --verbose
```

The native extension uses MoveIt Core and pybind11. The CMake Python-discovery,
link-signature and executable-permission fixes are included. The action interface
and presets are unchanged. The native geometry test requires a compiled extension;
Python tests cover simultaneous fallback orchestration, cancellation/failure,
intermediate close passes, spline interpolation and computation limits.
