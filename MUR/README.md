# MUR volume selector 0.5.1 — ignore only the opposite moving arm

Contains mur_base_selector 0.5.1 and mur_reachability 0.7.1.
This corrects the overly broad collision exclusion in 0.5.0.

## Offline collision policy

For each arm, check IK accuracy, joint limits, arm self-collision and collisions
with the rest of the robot URDF. Ignore only the opposite moving arm branch,
including its descendant hand, fingers and tools. Fixed mounts, torso and base
remain collision obstacles. Existing SRDF disabled-collision pairs are respected.
The branch starts at the first active joint's child link; it does not use name
prefixes. Overlapping branches and misplaced TCPs fail initialization.

No live arm posture or live planning-scene service is used. Each worker builds
its own collision scene from URDF/SRDF. Moving the opposite arm cannot change
its partner's cache results. Two workers, one per arm, remain the default.
The native metadata records the ignored link list for each arm.

The evaluated arm's URDF hand/tool geometry is included. Non-IK joints (such as
fingers) use model-default positions, not live positions; this is not a guarantee
for every possible finger shape. Runtime attached objects and world obstacles
are not part of this offline volume. Geometry/limits/SRDF changes require fresh
cache generation. Actual arm trajectories still require MoveIt planning later.

The configurable 250-degree FOV, TCP direction constraint, cache path, online
zero-IK lookup and Nav2 scoring remain. Nav2 uses the actual current footprint;
there is no required starting arm posture. Movement during selection invalidates
that query to avoid using an outdated footprint.

## Update and rebuild BOTH packages

Stop the generator and selector. Back up source/configuration outside ros2_ws/src.
Replace mur_base_selector and mur_reachability under ~/ros2_ws/src/MUR with these
directories. Preserve robot-specific frame/topic/model/scoring configuration.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select mur_reachability mur_base_selector --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Generate a fresh cache

Neither the old posture-dependent cache nor the 0.5.0 collision-free cache is
compatible. Back up any existing dexterous_volume.npz under a different name
before generating. Do not use --resume on a previous release's checkpoint.
With MoveIt model parameters and TF available:

```bash
ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml \
  --workers 2
```

Default output:
`~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz`.
For interruptions of this version's generation, repeat with --resume. No arm
posture needs restoring. Sampling/model/mount settings must still match.

The selector rejects mismatched collision policies. Missing or incomplete cache
prints instructions and exits cleanly. Restart the selector after generation.
The driver launch entry stays `("mur_base_selector", "base_selector.launch.py", {})`.

## Verification

67 Python tests passed; syntax, XML and YAML validated. Six native MoveIt tests
are included for body, mount, self and own-tool collisions, opposite-arm posture
independence, and both arm directions. ROS/MoveIt are unavailable here, so native
compilation and those native tests have NOT been run. Run them in your workspace:

```bash
colcon test --packages-select mur_reachability
colcon test-result --verbose
```
