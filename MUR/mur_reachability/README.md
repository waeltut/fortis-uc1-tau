# MUR reachability cache and base candidates

ROS 2 Humble / MoveIt. Separate from the earlier `mur_workspace` package;
this is an orientation-aware reachable-pose cache, **not** an all-orientation
dexterous volume. It never sends a motion command.

## Quick start

Extract so there is exactly one package folder:

    ~/ros2_ws/src/MUR/mur_reachability/package.xml

Inside the ROS container:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src/MUR/mur_reachability --ignore-src -r -y
colcon build --symlink-install --packages-select mur_reachability --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Start your normal MiR + arms + MoveIt launch. Then:

```bash
ros2 launch mur_reachability reachability.launch.py
```

This launches two nodes:

- `reachability_cache`: service to generate and save the cache; visualises its spatial projection.
- `base_candidates`: reads the cache, accepts a goal pose, and returns/visualises candidate base poses.

Generate once (the service returns after saving):

```bash
ros2 service call /generate_reachability_cache std_srvs/srv/Trigger '{}'
```

When generation finishes, load the new cache into the query node:

```bash
ros2 service call /reload_reachability_cache std_srvs/srv/Trigger '{}'
```

Later launches load the saved cache automatically; do not generate again just
because MiR has moved. On later launches you can omit the builder:

```bash
ros2 launch mur_reachability reachability.launch.py start_builder:=false
```

Both nodes require the running MoveIt parameter source at startup. The query
node also needs the scene service and TF at query time for exact verification.
This is not a standalone viewer for use without the robot description.

## Supply a target pose

The service returns both arm results in one response. Replace this example
with your actual TCP goal and orientation; these numbers are not a promise
of a reachable task:

```bash
ros2 service call /find_base_candidates mur_reachability/srv/FindBaseCandidates \
"{target: {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, time_budget: 3.0, max_verified_per_arm: 40}"
```

For less terminal output, publish the same pose to the goal topic. The node
logs a summary and publishes all visualisations:

```bash
ros2 topic pub --once /reachability/goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Goals in any TF-connected frame are supported. A zero timestamp uses the
latest TF; a nonzero timestamp requests the transform at that time. The
transformed target is held fixed for the query. Only one query is processed
at a time; overlapping requests can queue. Do not publish goals continuously
while benchmarking response time.

## Lichtblick

Open a 3D panel using `odom` as the display frame (or change `fixed_frame` in
configuration to your map frame).

| Topic | Meaning |
|---|---|
| `/reachability/cache_markers` | Blue/orange 3D cubes showing spatial occupancy of left/right cached TCP samples, attached to `mur` |
| `/reachability/candidate_markers` | Pale 5 cm floor tiles = approximate candidate area; bright tiles = at least one verified heading; yellow arrow = target TCP pose |
| `/reachability/left_candidates` | PoseArray of approximate left base poses, including heading |
| `/reachability/right_candidates` | PoseArray of approximate right base poses, including heading |
| `/reachability/left_verified` | Exact endpoint-verified left base poses; display as arrows |
| `/reachability/right_verified` | Exact endpoint-verified right base poses; display as arrows |

Toggle left/right marker namespaces separately where colours overlap. The
floor tiles are only a 2D projection: a bright square does NOT mean every
heading in it is valid. Use the verified PoseArrays for actual headings.
Each pose is the `base_link` origin at that cell centre; its z coordinate is
copied from current base TF. `floor_z` only controls where tiles are drawn.

The cache volume is a visual spatial projection of samples reachable with
some orientation. It is not a dexterous volume or certification of all points
inside a cube. Candidate markers are cleared at the start of every new query,
on reload, and remain empty after an error rather than showing stale success.
Cache markers are available while the builder node runs, including when it
loads an existing cache.

## How the cache is generated

Robot dimensions, mounts, TCP offsets, joint limits, collision shapes, SRDF
and IK configuration are copied from the running `/move_group`. No UR5e link
lengths or hardcoded robot geometry are embedded in the package. Group names,
TCP names and frame names are configuration defaults for your existing setup.
`reference_frame: mur` uses the common ancestor that worked in your model.

For each arm independently:

1. Capture the planning-scene joint state, attached objects and allowed-collision matrix.
2. Sample scalar arm joints inside the model bounds, with a 0.02 rad default
   joint-limit margin (for the UR revolute joints).
3. Compute TCP pose by forward kinematics, relative to `mur`.
4. Apply left y >= -0.20 m / right y <= +0.20 m TCP limits.
5. Reject robot self-collisions under the captured other-arm/hand/attachment state.
6. Store the full TCP pose and joint configuration for every accepted sample.

Defaults: 300000 attempted configurations per arm, configurable random seed.
This offline loop has no IK solve and no per-sample ROS request. Generation
can still take time because it checks collisions. It is an approximation to
reachable pose space, not an exhaustive coverage guarantee. More samples can
improve coverage. A zero-sample arm is reported as an error; no new cache
replaces the old one in that case.

The atomic, checksummed binary cache is at:

    ~/.ros/mur_reachability/reachability.bin

It includes model/parameter provenance, frame, group/joint ordering, sample
poses and seeds, generation settings, captured joint states, and the exact
scene snapshot as ROS CDR hex in YAML metadata. It uses explicit little-endian
IEEE-754 double records, not raw C++ struct memory. A previous file is retained
as `.previous`. Keep `cache_file` in a persistent bind mount if your container
is recreated.

The cancellation service is:

```bash
ros2 service call /cancel_reachability_cache std_srvs/srv/Trigger '{}'
```

Cancellation does not replace the previous finished cache.

## How an online query works

The cache is loaded in memory once. The lookup scans cached samples using
24 planar base headings (15 degree spacing). It filters by target height and
orientation, calculates candidate base translations, and bins them at 5 cm.
Several joint seeds can be retained per `(x cell, y cell, heading)`.

For yaw R and stored TCP offset p in the base frame:

    base_xy = target_xy - (R * p)_xy

The offset includes the live `base_link -> mur` transform, obtained from TF.
No assumption that the two frame origins or axes coincide is made. The base
must be level and the fixed frame must have an upward z axis. The base origin
height is held at its currently observed value; slopes and height-changing
platforms are not handled.

Search tolerances (default 4 cm in height, 20 degrees in orientation) only
select approximate seeds. They are NOT used to certify final poses. Snapping
to cell centres also requires refinement. The node uses local C++ MoveIt IK
and the cached seeds to solve the exact target at each selected planar base
pose. It explicitly verifies the resulting FK, joint bounds/margin, crossover
limit and whole-robot self-collision using a new scene snapshot. The target
is transformed into the hypothetical robot frame; it does not incorrectly
use the actual base pose for the hypothetical IK.

The default verification tolerances are 1 mm position and 0.01 rad orientation.
Left and right candidates are verified in turn, up to 40 successful poses per
arm or until the processing budget is exhausted. Candidates are ordered by
approximate cache-match error only; this is NOT navigation optimisation.

The default budget is 3 seconds, including the scene request, lookup and
verification. This is a cooperative deadline, not a hard realtime guarantee:
IK plugins must honour their timeout, and CPU scheduling, message publication,
response serialisation and request queuing can add latency. Lookup checks the
deadline periodically and reports truncation. Full Jetson timings must be
measured on your system. Do not present all unverified candidates as certified.

## Response semantics and reliability

- `success`: the query ran successfully, even if it found no verified placements.
- `left_candidates` / `right_candidates`: approximate cached suggestions.
- `left_verified` / `right_verified`: exact goal IK and self-collision checked
  at the returned base pose, for the captured current other-arm state.
- `left_solutions` / `right_solutions`: arm joint states paired in order with
  the corresponding verified base poses.
- `budget_exhausted`: the processing deadline was reached.
- `search_truncated`: lookup time/candidate/memory limits omitted suggestions.
- `elapsed_seconds`: measured server processing duration, excluding time queued
  before the callback and transport after it.
- `cache_id`: which recorded dataset was used.

No result means “no solution found within this sampled search and budget,”
not that the task is impossible. A verified endpoint is also not proof of a
collision-free arm path. Different goals or headings can require different
IK branches. Position projection alone loses heading information.

The two arm sets apply independently to the same desired TCP pose. Neither
their visual overlap nor their intersection proves simultaneous bimanual
feasibility. A future dual-goal task must specify and verify both arm states
together.

This version intentionally does NOT check navigation costmaps, world obstacles,
base footprints, floor collisions, or arm/base motion paths. Those belong in
the next navigation/manipulation feasibility stage. Do not execute these base
poses directly as certified safe goals. Self-collision follows MoveIt's ACM;
TF-only MiR geometry is not included unless present in the MoveIt collision
model. World objects are excluded from both cache and endpoint checks.

A changed parked arm/hand can change coverage. Online verification uses the
new state, so accepted endpoints are rechecked, but regions omitted from the
old cache cannot be recovered merely by lookup. Rebuild for a materially
changed parked configuration or tool/attachment. Cache loading rejects
model/parameter or cache-filter mismatches. Files referenced by URDF mesh URLs
are not content-hashed: regenerate if external mesh contents change without
a URDF change. Live model changes require restarting the nodes as well.

## Configuration and troubleshooting

Edit `config/reachability.yaml`, restart the nodes, and rebuild/reload the
cache if its frame, model or cache-filter settings changed. Custom file:

```bash
ros2 launch mur_reachability reachability.launch.py config:=/absolute/path/reachability.yaml
```

- **No cache loaded:** generate, then call `/reload_reachability_cache`.
- **TF error:** check `fixed_frame`, `base_frame`, `reference_frame` and the goal
  frame. The query fails rather than guessing a transform.
- **No verified results:** inspect candidate count, goal orientation, solver
  settings and current self-collisions. Increase samples, seed search tolerances,
  IK timeout or query budget as appropriate; no option silently disables checks.
- **Sparse area:** this is a sampled map and limited shortlist, not every possible
  base cell. Increasing `samples_per_arm` and regenerating improves sampling.
- **Cache mismatch:** both nodes must use the same model, reference frame,
  groups/TCPs, crossover and joint-margin settings.
- **Clock skew on extraction:** archive timestamps are fixed to 2020 to avoid
  the future-timestamp issue seen with the earlier package.

## Validation included

The independent C++ core test compiles without ROS and exercises known base
translations/headings, tilted mounting frames, quaternion sign equivalence,
height rejection, deadline truncation, cache round-trip and corruption checks.
The synthetic benchmark used 300000 samples and 24 headings and took about
7 ms in the authoring environment for lookup only. This is NOT a ROS/IK/Jetson
benchmark or a latency guarantee.

Full ROS/MoveIt build and integration execution were not possible in the
authoring environment because ROS is absent. The first full build and robot
integration test must run in your Humble container. Relevant API references:

- https://moveit.picknik.ai/humble/api/html/classmoveit_1_1core_1_1RobotState.html
- https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html
- https://moveit.picknik.ai/humble/api/html/classrobot__model__loader_1_1RobotModelLoader.html
