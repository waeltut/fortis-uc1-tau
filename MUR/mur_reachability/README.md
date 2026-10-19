# mur_reachability

Provides the native `mur_reachability_engine` Python module for offline workspace
generation. It loads the MoveIt robot model and IK solvers and evaluates batches
of TCP poses. It has no standalone executable, online reachability service or
motion-execution interface. The online selector reads the resulting NPZ directly.

## Configuration

Edit `config/volume_engine.yaml` and pass its source path to the generator:

```bash
ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml \
  --engine-config ~/ros2_ws/src/MUR/mur_reachability/config/volume_engine.yaml \
  --workers 2
```

The YAML uses `/**: ros__parameters:` so both worker node names receive the same
model configuration. Source parameter services must respond when engines start.

| Parameter | Default | Meaning |
| --- | --- | --- |
| source_node | /move_group | Source of robot_description, semantic, kinematics and planning-limit parameters |
| reference_frame | mur | URDF link rigidly upstream of both arm chains |
| base_frame | base_link | Base frame in which cache positions are sampled; generator obtains mount TF |
| left_group | left_ur_manipulator | Left arm IK joint group |
| right_group | right_ur_manipulator | Right arm IK joint group |
| left_tip | left_tcp | Left target TCP link |
| right_tip | right_tcp | Right target TCP link |
| joint_limit_margin | 0.02 | Clearance from bounded joint-position limits; radians for revolute joints |
| verification_position_tolerance | 0.001 | Maximum FK endpoint translation error, metres |
| verification_orientation_tolerance | 0.01 | Maximum FK endpoint rotation error, radians |

Each arm group must have a working IK solver. The reference frame must be rigidly
upstream. Tips must belong to their moving branches; branches must be disjoint.

## Evaluation and collision policy

Each worker owns its model, solver and URDF/SRDF collision scene. Seeds start from
model defaults, then use successful warm starts and bounded random retries. No
live arm position or live planning-scene snapshot is taken.

For left-arm evaluation, all pairs involving the right moving branch are ignored;
right-arm evaluation does the reverse. Branches include descendants of the first
active arm joint, including hand/fingers/tool links. Fixed mounts remain obstacles.
All other SRDF collision rules are preserved. Whole-model self-collision checks
include the selected hand/tool even if it is outside the IK group.

Non-IK joints use model-default positions. Runtime attachments and world obstacles
are not imported. The cache is sampled endpoint evidence for the configured robot,
not a collision-free trajectory or a certificate for arbitrary finger/payload shapes.

## Python interface

- `Engine(config, sim_time, node_name='mur_kinematics')`: load model and initialise worker.
- `model_info()`: YAML metadata with frames, model identity, groups, tips, arm joints,
  collision policy and ignored collision links.
- `evaluate_offline(arm, poses, mount, seconds, attempts)`: bool result per pose;
  arm 0=left, 1=right; transforms use `[x,y,z,qx,qy,qz,qw]`. mount is base-from-reference.
- `close()`: stop worker resources; close the second engine before the first context owner.

Native batch evaluation releases the Python GIL, allowing two independent workers.
The generator owns checkpointing and output; this module does not save cache files.
Model parameters, groups, tips, tolerances and collision policy contribute to identity.

## Tests

The native test suite checks moving-branch membership, body/mount/self/tool
collisions and opposite-arm exclusion in both directions:

```bash
cd ~/ros2_ws
colcon test --packages-select mur_reachability
colcon test-result --verbose
```

See the sibling mur_base_selector documentation for cache generation and usage.
