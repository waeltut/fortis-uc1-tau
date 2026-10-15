# MUR sampled workspace volume selector — 0.5.1

The default `base_selector` now uses precomputed workspace orientation coverage.
There is **zero online IK**: the online node does not load the native engine.
Only the volume workflow is included. Old sample-cache selectors, servers, and
custom reachability messages/services have been removed. The coordinator is not included.

## What is precomputed

The offline builder checks a lattice of workspace positions and full TCP
orientations, including roll, separately for each arm. It keeps an orientation
bitmask at each position. The online node requires both broad coverage (default
90% of allowed sampled orientations) and a successful bit for the requested
orientation's nearest bin. A failed timed IK solve is treated as unavailable,
not as proof of physical unreachability.

The workspace is in `base_link`. Position FOV is centred on base +X and defaults
to 250 degrees (±125 degrees). Positions outside it are removed BEFORE offline
IK. Tool direction is a separate constraint: by default TCP +X must be at least
70 degrees from the horizontal ray pointing from the TCP back toward the base.
Outward, sideways, and downward tools are therefore eligible. Set
`inward_reference: spatial` for the full 3D inward ray, or change `tool_axis`
if your TCP convention differs. This does not constrain elbow motion.

## Missing or incomplete cache

The selector prints the resolved cache path and a generation/resume command,
then exits normally (exit code 0) if the cache is missing or incomplete. It does
not wait in the background; restart it after generation finishes. Corrupt or
unreadable files produce one error message and exit code 1, without a traceback.
The rest of a normal launch can continue. No cache is silently overwritten.

The default uses `~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz`.
For a different workspace layout, set `volume_file` in BOTH YAML files. `~`
is expanded for the account running ROS. The selector reads the configured source
path directly; the cache is not copied to `install` during builds. Moving an
existing checkpoint does not change its generation identity.

## Install and build

Back up your source packages/configs outside `ros2_ws/src` first. Replace the two
package directories with the clean directories in this archive at `~/ros2_ws/src/MUR/`, preserving any
local robot-specific configuration overrides as described in the root migration guide. Do not keep duplicate packages
under `src`. The new launch defaults to `volume_selector.yaml`; transfer your
Nav2 topic/frame/footprint customisations there if you changed them previously.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select mur_reachability mur_base_selector --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Both packages must be rebuilt: the offline engine has a new API. Installed
versions are `mur_base_selector` 0.5.1 and `mur_reachability` 0.7.1.
No commander or coordinator changes are required.

## Generate or resume the cache

Your old `reachability.bin` is not a volume cache and cannot substitute for it.
The new format is `dexterous_volume.npz`. The old file is neither read nor modified
by this pipeline. No `/generate_reachability_cache` call is used here.

Run MoveIt (for model/solver parameters) and robot TF. No live arm joint states or
planning-scene service are required for generation. Each arm uses a model-default
IK seed followed by warm/random retries, independent of physical arm positions.
The generator sends no motion commands. Two workers have separate native models, collision scenes
and solvers, with one checkpoint writer.
Use `--workers 1` for serial generation. Worker count does not change cache identity. Temporarily omit the selector
from your master launch until the first cache exists.

Edit `mur_base_selector/config/volume_generation.yaml`. Pass its source path
explicitly so you do not have to rebuild after each generator-setting edit:

```bash
ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml \
  --estimate-only

ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml
```

The default output is `~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz`. Override it
with `--output /absolute/path/name.npz`, then use the same `volume_file` in the
online YAML. `--engine-config /path/to/volume_engine.yaml` selects the offline
MoveIt model/group/TCP/joint-limit settings. There is no legacy `cache_file` parameter in this clean engine configuration. Add `--use-sim-time` if using simulation time.

Generation does substantially more work than the previous random-FK cache:
the default grid contains 4238 FOV-filtered positions and 208 orientations before
the inward-direction filter. It can take hours, depending on IK behaviour.
Progress/checkpoints are saved every 10 committed arm/voxels across both workers.
Ctrl+C waits for active batches, saves committed work, and exits. Queued results
not yet committed may be recomputed when resuming.
To resume with the same settings, model and rigid base-to-reference mount:

```bash
ros2 run mur_base_selector build_volume \
  --config ~/ros2_ws/src/MUR/mur_base_selector/config/volume_generation.yaml \
  --resume
```

Existing output is never overwritten by a new run without `--resume`. Choose a
new filename when changing the generation settings. An incomplete cache cannot
be used by the online selector.

## Configurable FOV and resolution

`volume_generation.yaml` (regenerate for these changes):

```yaml
workspace_fov_deg: 250.0
workspace_front_offset_deg: 0.0
voxel_size: 0.10
orientation_step_deg: 45.0
min_inward_angle_deg: 70.0
tool_axis: [1.0, 0.0, 0.0]
inward_reference: horizontal
```

`volume_selector.yaml` (restart selector for changes):

```yaml
front_fov_deg: 250.0
front_yaw_offset_deg: 0.0
minimum_orientation_coverage: 0.90
max_height_error: 0.05
max_orientation_error_deg: 30.0
```

You may narrow the online FOV without regenerating. To expand beyond the cached
FOV, regenerate with the wider angle. Generation and selection front offsets
must match. Full 360-degree generation is supported.

The defaults are a coarse foundation. `voxel_size: 0.05` and
`orientation_step_deg: 30.0` improve sampling at substantially greater offline
cost. The orientation step must divide 180 and be at least 15 degrees in this
implementation. Coverage is the fraction of sampled allowed orientations,
not a uniform solid-angle probability or a mathematical continuum guarantee.

## Run and use

Keep your original driver extra-launch entry:

```python
("mur_base_selector", "base_selector.launch.py", {}),
```

The same launch now starts the volume selector. If your entry explicitly passes
an old `selector_config`, change it to `volume_selector.yaml`. No separate
reachability server/cache server is needed online.

Or launch separately (only one selector at a time):

```bash
ros2 launch mur_base_selector base_selector.launch.py \
  selector_config:=$HOME/ros2_ws/src/MUR/mur_base_selector/config/volume_selector.yaml \
  arm_mode:=either
```

The input and display topics remain the same:

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
'{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.7071067811865476, z: 0.0, w: 0.7071067811865476}}}'

ros2 topic echo /base_selection/result --qos-durability transient_local
```

`/base_selection/target_markers` persists the target.
`/base_selection/candidate_markers`, `selected_pose`, `selected_path`, `status`,
`candidate_diagnostics`, and `selected_valid` retain their roles.
Selection does not drive the robot or move an arm.

## Online sequence and scoring

1. Check current TF, joint states, URDF/SRDF identity, scan and Nav2 state.
2. Take the nearest cached height slice. For every base heading (24 by default),
   check cached coverage/orientation, then invert each TCP offset to obtain the
   base X/Y. No grid rounding is added to these projected X/Y positions.
3. Compute one Dijkstra travel-cost field from the current base position in the
   global costmap. Lethal and disallowed unknown cells are blocked; inflation
   costs increase travel cost. Diagonal corner cutting is disallowed. This is a
   point-robot estimate for ranking, not final footprint validation.
4. Rank the projected base poses. For `either`, choose the better arm score at
   each identical base pose before asking Nav2 for a path.
5. Check destination footprints, request Nav2 paths for a bounded shortlist,
   and check swept footprints along those paths. Default: at most 80 destination
   checks and 8 path requests, stopping after 3 feasible paths.
6. Compare feasible results, refresh costmaps and recheck the chosen path.

The score is a weighted sum of navigation, destination cost, preferred working
distance and arm-specific facing. A wall is a rejection, not just a high score.
Near-wall inflation is a soft cost. Navigation initially uses the shared travel
field; final scoring uses actual path length and sampled path cost.

```yaml
weight_navigation: 1.0
weight_destination_cost: 2.0
weight_working_distance: 1.0
weight_facing: 0.5
travel_inflation_weight: 3.0
length_scale: 5.0
distance_scale: 0.25
left_preferred_distance: 0.65
right_preferred_distance: 0.65
both_preferred_distance: 0.65
left_facing_min_deg: 0.0
left_facing_max_deg: 60.0
right_facing_min_deg: -60.0
right_facing_max_deg: 0.0
both_facing_min_deg: -10.0
both_facing_max_deg: 10.0
```

Working-distance cost is `((distance - preferred_distance) / distance_scale)^2`,
using horizontal base-to-target distance. Facing cost is zero within the selected
arm's sector and increases quadratically outside it. Positive bearings mean left.
Default preferred distance is an initial tuning value, not robot-calibrated.

Total selection budget is 12 s, including a 2 s final-validation reserve. Travel
field expansion has a 2 s cap. Unsettled field cells are excluded if that cap is
hit; results report `travel_field_truncated`. Increase `travel_field_budget` and
total `selection_budget` if large maps need more expansion. The result is bounded
best-of-checked, not a global optimum. Python grid expansion is a replaceable
foundation component if profiling identifies it as the next bottleneck.

## Meaning and limits of the output

The result explicitly reports `online_ik_checks: 0`,
`reachability_evidence: sampled_volume_lookup`, orientation coverage, height and
orientation-bin error, and `exact_target_ik_verified: false`. `joint_solutions`
is empty: no executable arm state is produced. `selected_valid` means the
sampled-volume conditions and navigation checks passed within the validity period.

This sampled cache cannot prove every pose between samples reachable. The height
and orientation tolerances bound approximation; they are not final arm tracking
tolerances. Tighten them or increase offline resolution for stronger evidence.
A future arm executor must still plan the actual arm trajectory.

Generation checks IK endpoint accuracy, joint limits, self-collision and collisions
against the remaining URDF geometry. Only the opposite moving arm branch and
its descendant hand/tool links are excluded. Fixed mounts, torso and base remain
checked, subject to the existing SRDF disabled-collision pairs. The arm-side
crossing filter is removed; configurable FOV and tool direction constraints remain.
No generation, resume or lookup requires restoring a live arm posture.

Each worker has an independent model/solver/collision scene built from URDF/SRDF.
Non-IK joints such as fingers use model-default positions. This is sampled endpoint
evidence for that geometry, not a guarantee for all hand shapes or later attached
objects. World obstacles and actual arm trajectories are handled later by MoveIt.
Changing geometry, limits, SRDF, TCPs, sampling or the rigid mount requires a new cache.

Navigation uses the actual published footprint. Arms may start in any posture,
but must remain stationary during selection to keep footprint checks valid.
`both` means independent endpoint evidence; collisions between arms are excluded.

Policy `self_body_ignore_other_arm_v1` rejects previous collision-free AND
posture-dependent caches/checkpoints, including with --resume. Generate a fresh
cache. New-policy checkpoints can resume after physical arms move.

## Files and verification

- `volume.py`: orientation masks, inverse projection, distance/facing costs,
  shared travel field; no ROS dependencies.
- `build_volume.py`: resumable offline generation.
- `volume_workers.py`: two arm workers and single-writer checkpoint coordination.
- `volume_node.py`: online selection; no native engine or IK calls.
- `navigation.py`: TF, sensors, Nav2 transport and footprint validation.
- `mur_reachability/src/offline_engine.cpp`: offline MoveIt batch checks.
- `mur_reachability/src/offline_model.hpp`: model loading, joint-limit and arm-specific collision checks.

67 retained Python tests passed, including volume lookup geometry, full orientation/roll,
FOV, coverage holes, unknown space, grid rotation, travel-field truncation,
zero-IK workflow, arm scoring, path comparison and final invalidation. Tests for the removed algorithms were removed; shared transport and geometry
checks remain included. Python syntax, package XML and YAML were checked.
ROS/MoveIt are not installed in the development environment: native compilation,
real cache generation, end-to-end ROS behaviour and robot timing remain untested.

```bash
PYTHONPATH=./mur_base_selector python3 -m unittest discover -s mur_base_selector/test
```
