# Cache generation

`ros2 run mur_base_selector build_volume` checks a grid of TCP positions and full
orientations separately for each arm. It stores per-orientation success bits and
per-arm completed-voxel flags in a compressed NPZ file. The selector derives
coverage profiles from these bits, including for voxels that fail the default
coverage threshold. Joint solutions and trajectories are not stored.

## Run

Start MoveIt for robot-description/IK parameters and publish the base-to-reference
TF. Live arm joint positions and a planning-scene service are not required.

```bash
ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml \
  --engine-config ~/ros2_ws/src/MUR/mur_reachability/config/volume_engine.yaml \
  --workers 2
```

Output defaults to
`~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz`.
The pathname is excluded from generation identity, so a checkpoint can be moved.
Set the same file in the selector and viewer. The file is read directly from its
configured path and is not installed/copied automatically by colcon.

Two workers own independent native models, solvers and collision scenes; one
writer commits complete arm/voxels and saves checkpoints. No motion is commanded.
A failed bounded IK solve is unavailable evidence, not proof of unreachability.

## Interrupt and resume

Press Ctrl+C once and wait for worker cleanup/checkpoint writing and the shell
prompt. Committed voxels are retained; queued/uncommitted work may be repeated.
Repeat the command with `--resume`. Model, mount and generation settings must match;
physical arm positions need not be restored. Existing output is not overwritten
without resume; use a new filename for a different generation configuration.

A ROS shutdown traceback can occur after Ctrl+C if the context was already shut
down; the worker saves before final ROS cleanup. If uncertain, inspect the file:

```bash
python3 - <<'PYTHON'
from pathlib import Path
import numpy as np
p = Path('~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz').expanduser()
with np.load(p, allow_pickle=False) as f:
    done = f['completed']
    print(f'{int(done.sum())}/{done.size} arm/voxels saved')
PYTHON
```

## Command-line options

| Option | Default | Meaning |
| --- | --- | --- |
| `--config` | Installed volume_generation.yaml | Offline sampling configuration; use a source path to avoid rebuilding after edits |
| `--engine-config` | Installed volume_engine.yaml | Native model/group/TCP/tolerance configuration |
| `--output` | volume_file from generation YAML | Override cache output path |
| `--workers` | 2 | 1 or 2; independent arm workers |
| `--resume` | Off | Load and continue a compatible checkpoint |
| `--use-sim-time` | Off | Use simulation ROS time |
| `--estimate-only` | Off | Print grid/orientation/check counts without starting ROS/IK |

## Sampling parameters: volume_generation.yaml

Coordinates are relative to the cached base frame, normally base_link.

| Parameter | Default | Meaning |
| --- | --- | --- |
| volume_file | Source config/dexterous_volume.npz path | Cache output |
| bounds_min | [-1.0, -1.1, 0.3] | Lower XYZ workspace bounds, metres |
| bounds_max | [1.1, 1.1, 1.6] | Upper XYZ workspace bounds, metres |
| voxel_size | 0.10 | Grid spacing, metres; samples at voxel centres |
| workspace_fov_deg | 250.0 | Position bearing span centred on base front |
| workspace_front_offset_deg | 0.0 | Front offset, degrees |
| orientation_step_deg | 45.0 | Full orientation lattice including roll; must divide 180, supported builder range 15–90 degrees |
| tool_axis | [1.0, 0.0, 0.0] | Unit TCP-frame vector used as pointing direction |
| min_inward_angle_deg | 70.0 | Minimum angle from inward TCP-to-base direction |
| inward_reference | horizontal | horizontal removes Z from the inward ray; spatial uses the full 3-D ray |
| ik_timeout | 0.01 | Timeout per native IK attempt, seconds |
| attempts | 3 | Bounded attempts per pose, 1–100; warm/default seed then random retries |
| save_every_voxels | 10 | Save after this many committed arm/voxels across both workers |

Smaller voxels and orientation steps increase offline cost substantially. All
sampling settings participate in checkpoint compatibility; keep them unchanged
when resuming. Completion means both arm/voxel counters finish.

## Collision policy

Each arm is checked against itself and the remaining URDF geometry, including
body, fixed mounts and its own hand/tool descendants. Only the opposite moving
arm branch and its downstream hand/tool are ignored. Existing SRDF disabled
collision pairs remain allowed. The branch begins at the first active arm joint's
child link; fixed parent mounts remain obstacles. Overlapping branches are rejected.

Non-IK joints such as fingers use model-default positions. Runtime attached
objects and world obstacles are not imported. Consequently, the cache is not a
certificate for every finger shape, runtime payload, environment or trajectory.
The other arm's live posture never affects generation or resume.

## Derived online profiles

The original 70-degree cache contains the sampled orientations needed for 90- and
120-degree profiles. For each voxel/profile, coverage is successful allowed samples
divided by all allowed samples in that profile. Excluded orientations are not
counted as failures. Profiles are calculated once in memory and do not rewrite NPZ.

For every candidate, the requested direction and nearest sampled orientation must
fit an eligible profile, its cached bit must have succeeded and coverage must meet
the selector threshold. Acceptance is the union of profiles; the broadest passing
profile is reported. Both-arm mode may use a different eligible profile per arm.
A percentage threshold means qualifying volumes need not be strictly nested.

With the horizontal inward ray, a downward TCP direction is at 90 degrees, so it
can use 70/90 but not 120. Position FOV is independent of TCP direction. Coverage
is a fraction of sampled orientations, not a uniform solid-angle probability or
a guarantee for every continuous pose between samples.
