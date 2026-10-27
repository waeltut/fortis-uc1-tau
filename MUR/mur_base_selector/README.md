# mur_base_selector

Finds a base pose `(x, y, yaw)` for a requested TCP pose. It uses an offline sampled
workspace cache, scores eligible base poses, and requests Nav2 paths for a bounded
shortlist. It publishes the selected pose/path without executing motion.

## Executables

| Executable | ROS node | Purpose |
| --- | --- | --- |
| `base_selector` | `/mur_base_selector` | Online selection with zero IK calls |
| `build_volume` | `/mur_volume_builder` plus native model nodes | Offline cache generation, two arm workers by default |
| `volume_visualizer` | `/mur_volume_visualizer` | Read-only workspace and selection diagnostics |

## Configuration

Files are under `~/ros2_ws/src/MUR/mur_base_selector/config/`:

- `volume_selector.yaml`: online scoring, budgets, interfaces and cache path.
- `volume_generation.yaml`: offline sampling, orientation constraints and output path.

Native model settings are in
`~/ros2_ws/src/MUR/mur_reachability/config/volume_engine.yaml`.

Pass source configuration paths explicitly to use edits without rebuilding.
Launch defaults otherwise use installed package-share YAML copies. Restart the
selector after changing its parameters; it rejects live configuration changes.
Viewer display parameters can change live. All YAML numbers use decimal points;
angles explicitly ending in `_deg` are degrees, other angular tolerances are radians.

[Full selector parameter reference](docs/selector_parameters.md)

## Launch selector

Prerequisites: complete cache, connected TF, fresh configured arm joint states,
current published footprint, scan stream, active configured Nav2 lifecycle nodes,
Nav2 costmap services and planner action, and the cache's MoveIt parameter source
for model identity checks. Run one selector instance.

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mur_base_selector base_selector.launch.py \
  selector_config:=$HOME/ros2_ws/src/MUR/mur_base_selector/config/volume_selector.yaml \
  arm_mode:=either
```

Launch arguments: `selector_config` (YAML path), `arm_mode` (`left`, `right`,
`either`, `both`; default `either`), `use_sim_time` (default `false`). The launch
arm_mode argument overrides the YAML value. With the MUR driver's extra-launch
list, the entry is `("mur_base_selector", "base_selector.launch.py", {})`.

## Target input

`/base_selection/target` accepts `geometry_msgs/msg/PoseStamped`. Set a valid TF
frame, finite XYZ and a unit quaternion. Zero timestamp uses the latest TF. The
target is transformed and anchored into `fixed_frame` for that request. Targets
received while busy are rejected, not queued.

Example at map `(2.5, 0, 0.8)` with TCP +X pointing downward:

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
'{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.7071067811865476, z: 0.0, w: 0.7071067811865476}}}'
```

```bash
ros2 topic pub /base_selection/target geometry_msgs/msg/PoseStamped \
'{header: {frame_id: map}, pose: {position: {x: 1.4, y: 0.0, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```


The selected arm's configured TCP link is the target. `both` asks for independent
reachability evidence for each arm at the same target/base pose; it is not two
independent target inputs or a simultaneous collision-free dual-arm plan.

## Selection workflow

1. Check current state, cache model/mount identity and Nav2 availability.
2. Use the nearest cached height layer within tolerance. Enumerate base yaw bins
   and inverse-project eligible TCP offsets into candidate base X/Y positions.
3. Check requested direction, nearest successful orientation bit and coverage in
   at least one configured profile (default 70/90/120 degrees).
4. Build a shared global-costmap travel field for preliminary navigation scoring.
5. Rank candidates by navigation, destination cost, working distance and facing;
   deduplicate identical base poses and diversify the bounded shortlist.
6. Check the current footprint at destinations. Request one Nav2 path per checked
   candidate pose; check swept footprints along returned paths.
7. Compare feasible candidates and revalidate against fresh costmaps. Publish a
   selected result, or a failure explaining which limit/check prevented selection.

A timeout or failed IK sample is not proof of physical unreachability. The search
is best-of-checked under configured limits, not a global optimum. The cache is
sampled endpoint evidence, not an executable arm trajectory. Later MoveIt motion
planning must check the actual arm motion and environment.

## Scoring and footprint checks

Lower score is better. Navigation cost is normalised by `length_scale`, working
distance uses squared deviation from the per-arm preferred distance divided by
`distance_scale`, and facing uses squared angular deviation outside the per-arm
zero-penalty sector. Increase `weight_facing` and narrow that sector to favour
forward-facing base poses. Bearing zero means the target is ahead of the base
(with `front_yaw_offset_deg=0`); positive bearings are left. It is independent of
TCP roll/pitch/yaw.

Footprint checks use the actual published polygon, transformed into the candidate
pose, with additional `footprint_padding`. Lethal cells (254), disallowed unknown
cells (255) and an uncovered global footprint reject a pose. Lower costs, including
inflation value 253, contribute soft cost rather than footprint rejection. Nav2
can apply its own planner constraints. Local-map overlap is checked where available;
leaving the local map alone does not reject a global candidate.

Arms can start anywhere, but arm/base motion during selection invalidates the query
so footprint evidence is not silently reused after motion. No footprint reduction
or arm retraction is requested.

## Outputs

Topics below use the `/base_selection/` prefix.

| Suffix | ROS type | Contents |
| --- | --- | --- |
| `selected_pose` | `geometry_msgs/PoseStamped` | Selected base goal in fixed_frame; volatile, emitted on success |
| `selected_path` | `nav_msgs/Path` | Validated candidate path; emptied on invalidation |
| `selected_valid` | `std_msgs/Bool` | Current selection validity flag |
| `result` | `std_msgs/String` | JSON: request ID, validity, selected arm/pose, score, path costs/length, profiles, cache ID, timings and counts |
| `status` | `std_msgs/String` | JSON state/message and supplementary diagnostics |
| `candidate_diagnostics` | `std_msgs/String` | JSON candidate IDs, arm, pose, state, score and detail |
| `candidate_markers` | `visualization_msgs/MarkerArray` | Status-coloured shortlist arrows |
| `target_markers` | `visualization_msgs/MarkerArray` | Requested TCP target and direction |
| `visualization_data` | `std_msgs/String` | Bounded stage samples with exact counts for the viewer |

All listed outputs except selected_pose are reliable/transient-local. A latched
message may describe a past request: inspect request_id, valid and selected_valid.
Validity expires after selection_ttl and is monitored for base/arm motion; it is
not continuous path revalidation against every new obstacle.

Result fields include `orientation_profiles` per arm (minimum inward angle,
coverage, allowed sample count), `active_profile_angles_deg`, `volume_candidates`,
`ranking_pool`, `destination_checks`, `path_checks`, `feasible_paths`,
`travel_field_truncated`, `timings`, `elapsed_seconds` and `valid_for_seconds`.
`online_ik_checks` is zero and `joint_solutions` is empty. `pose` is `[x,y,yaw]`.
An invalidated result may contain only request_id and valid=false.

```bash
ros2 topic echo /base_selection/result --qos-durability transient_local
ros2 topic echo /base_selection/candidate_diagnostics --qos-durability transient_local --once
```

A red candidate arrow means rejection, not necessarily destination collision.
Inspect state: global_/local_ lethal_obstacle or unknown_space, path_ rejection,
or planner failure. Grey=unexamined, orange=destination clear, green=path checked,
cyan=selected. Arrows show base +X, not TCP direction.

## Services and external interfaces

```bash
ros2 service call /base_selection/reselect std_srvs/srv/Trigger '{}'
ros2 service call /base_selection/cancel std_srvs/srv/Trigger '{}'
```

Reselect repeats the last anchored target. Cancel requests cancellation and clears
validity; an outstanding planner action must terminate before another request.
The selector calls the configured ComputePathToPose action, GetCostmap services,
GetState lifecycle services and cached source node's GetParameters service. It
subscribes to the configured PolygonStamped footprint, JointState and LaserScan
streams, and TF. Scan is used for freshness checks; obstacles come from costmaps.

## Cache and viewer

- [Generate/resume the cache; sampling, profiles and collision policy](docs/cache_generation.md)
- [Display voxels, height slices and candidate stages](docs/visualization.md)

Missing/incomplete cache produces a concise message with a generation/resume
command and exits. Incompatible or unreadable caches produce an error. No file is
silently overwritten or regenerated.
