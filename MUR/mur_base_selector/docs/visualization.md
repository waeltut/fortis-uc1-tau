# Workspace and candidate visualisation

The viewer reads the completed cache without IK. It displays cubes around the
current base frame and subscribes to selector diagnostics for target-specific
candidate headings. It does not select poses or command movement.

```bash
ros2 launch mur_base_selector volume_visualizer.launch.py
```

Launch arguments: `volume_file` (default source config/dexterous_volume.npz path),
`minimum_coverage` (0.9), `profile_angle_deg` (-1.0, meaning cached generation angle),
`arm` (both), `use_sim_time` (false). Example:

```bash
ros2 launch mur_base_selector volume_visualizer.launch.py \
  profile_angle_deg:=120.0 arm:=left
```

The viewer can run without Nav2 or MoveIt. To overlay cubes on the robot in map/odom,
TF must connect that display frame to the cached base frame. For cache-only inspection,
use base_link (or the cache's configured base frame) as the 3D display frame.

## Lichtblick/RViz topics

All outputs are reliable/transient-local visualization_msgs/MarkerArray. Enable
these in a 3D panel; late subscribers can receive the latest display.

| Topic | Contents |
| --- | --- |
| /base_selection/workspace/left | Blue left-arm workspace cubes |
| /base_selection/workspace/right | Orange right-arm workspace cubes |
| /base_selection/workspace/intersection | Purple overlap when arm=intersection |
| /base_selection/workspace/labels | Active profile, coverage threshold, displayed/total voxel counts |
| /base_selection/cache_candidates | Cache-valid base headings before navigation filtering/ranking |
| /base_selection/travel_candidates | Candidates with finite travel-field estimates |
| /base_selection/shortlist_candidates | Ranked shortlist before destination/path checks |
| /base_selection/diagnostic_labels | Request/stage counts, forward-facing counts and profile counts |

The selector separately publishes `/base_selection/target_markers` and
`/base_selection/candidate_markers` (status-coloured shortlist arrows).

Workspace brightness indicates coverage. Blue/orange cubes may overlap; toggle
one arm off to inspect the other. Workspace cubes follow the current base frame.
Candidate snapshots are anchored in the selection's fixed frame.

Candidate-stage headings are green within +/-30 degrees of forward, yellow from
30–60 degrees, and magenta beyond 60 degrees. Colours describe target bearing
relative to base heading, not navigation validity or TCP orientation. Show one
stage at a time to reduce overlapping drawings.

## Viewer parameters

Use `ros2 param set /mur_volume_visualizer <name> <value>`. Display changes appear
within two seconds and never change selector parameters or cache contents.

| Parameter | Default | Meaning |
| --- | --- | --- |
| volume_file | ~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz | Loaded once at startup; restart viewer to change/reload file |
| minimum_coverage | 0.9 | Minimum profile coverage, 0–1; 0 shows voxels with at least one successful allowed sample |
| profile_angle_deg | -1.0 | -1 uses cached generation angle; otherwise >= cached angle and <180 |
| arm | both | left, right, both (separate layers), or intersection |
| z_min | -10.0 | Lowest displayed height in cached base frame, metres |
| z_max | 10.0 | Highest displayed height, metres; must be >= z_min |
| alpha | 0.45 | Cube opacity, >0 and <=1 |
| voxel_scale | 0.85 | Cube edge length as fraction of voxel spacing, >0 and <=1 |
| max_voxels_per_arm | 20000 | Maximum displayed cubes per layer; labels retain total count |
| show_voxels | true | Toggle workspace cubes |
| use_sim_time | false | ROS simulation clock setting |

Compare profiles one command at a time:

```bash
ros2 param set /mur_volume_visualizer profile_angle_deg 70.0
ros2 param set /mur_volume_visualizer profile_angle_deg 90.0
ros2 param set /mur_volume_visualizer profile_angle_deg 120.0
```

Inspect a height slice and then restore all heights:

```bash
ros2 param set /mur_volume_visualizer z_min 0.75
ros2 param set /mur_volume_visualizer z_max 0.85
# Restore upper bound first so the interval stays valid.
ros2 param set /mur_volume_visualizer z_max 10.0
ros2 param set /mur_volume_visualizer z_min -10.0
```

Show any sampled reachability or the overlap:

```bash
ros2 param set /mur_volume_visualizer minimum_coverage 0.0
ros2 param set /mur_volume_visualizer arm intersection
```

The workspace view shows coverage-qualified points for its chosen profile, not
eligibility for the current target's exact orientation. The selector still checks
that direction and orientation bit. Cubes represent samples, not certified solid
reachable blocks. Non-IK finger geometry uses cached model defaults.

## Diagnose missing candidates

Start the selector and viewer, then send a target. Inspect the stage labels:

1. No forward candidates at cache stage: inspect height, orientation/profile,
   coverage and FOV conditions. Raising a facing weight cannot restore these.
2. Forward candidates disappear at travel stage: inspect global costmap and
   travel_field_truncated. This stage is a point-based travel estimate, not a
   destination footprint check.
3. Forward candidates disappear only at shortlist: inspect ranking_pool_size,
   working-distance and facing scores.
4. Shortlisted candidates turn red: read candidate_diagnostics for destination,
   planner or path rejection. The arrow's centre can be clear while its full
   current-arm footprint touches an obstacle.

Selector `visualization_max_candidates` bounds display samples per stage (6000 by
 default), spread across arm/bearing sectors. Counts are exact; sampled/displayed
fields explain thinning. Samples are not extra Nav2 requests. `publish_visualization_data`
can disable this diagnostic work for timing comparisons. Both settings require
selector restart. Viewer profile/threshold settings do not change the selector.

The viewer subscribes to `/base_selection/visualization_data` (std_msgs/String JSON):

```bash
ros2 topic echo /base_selection/visualization_data --qos-durability transient_local --once
```

New targets clear the preceding stage snapshots. Snapshots persist for inspection
after a request and are not live navigation clearance. If checking fails before
cache lookup, the labels remain at checking. Profile counts count per-arm evidence;
in both-arm mode their sum can exceed the number of candidate base poses.
