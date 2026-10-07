# MUR base selector

ROS 2 Humble / Ubuntu 22.04 `ament_python` package. Selects a manipulation base
pose by calling `/find_base_candidates`, filtering the **verified** service
response, and asking Nav2 to compute paths. It publishes a selected pose and
diagnostics. **It does not drive the robot or execute an arm trajectory.**

Defaults match the requested reachability settings: **20.0 seconds and 150
verified poses per arm**. These are explicit service request fields; changing
the reachability node's YAML defaults will not change this client's request.

## Install

Extract the archive so the package is at:

```text
~/ros2_ws/src/MUR/mur_base_selector/package.xml
```

Inside the ROS Humble container / ROS environment:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/MUR/mur_base_selector --ignore-src -r -y
colcon build --packages-up-to mur_base_selector --symlink-install
source install/setup.bash
```

`mur_reachability` must already be in the workspace. The package imports its
generated `FindBaseCandidates` Python interface; it does not create a duplicate
interface or modify the existing reachability package.

The original reachability CMake file uses `rosidl_generate_interfaces`, which
generates the Python service bindings as part of a normal build. If the import
fails, rebuild `mur_reachability` and source the same workspace overlay.

## Start and use

Start the existing MUR, MoveIt, reachability and Nav2 launches first. Keep the
base and arms stationary during selection. Then:

```bash
ros2 launch mur_base_selector base_selector.launch.py arm_mode:=either
```

Publish the desired **TCP target**, not a base navigation destination:

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.8}, orientation: {w: 1.0}}}'
```

The example is a target at `(1.0, 0.0, 0.8)` metres with identity orientation in
`map`; choose the actual TCP orientation needed for your task. A zero timestamp
means use latest TF. A nonzero input timestamp is respected when transforming
the target. The target is then anchored in `fixed_frame` for this request and
future reselections, even if the original input frame was attached to the robot.

For Lichtblick, publish `geometry_msgs/msg/PoseStamped` to
`/base_selection/target`. This is a separate topic from both Nav2 `/goal_pose`
and the reachability node's `/reachability/goal_pose`; it avoids triggering a
second reachability query in the existing topic callback.

Observe progress/results:

```bash
ros2 topic echo /base_selection/status --qos-durability transient_local
ros2 topic echo /base_selection/result --qos-durability transient_local
```

Repeat the same fixed target after moving an obstacle or changing parameters
and restarting the selector:

```bash
ros2 service call /base_selection/reselect std_srvs/srv/Trigger '{}'
```

Stop a selection:

```bash
ros2 service call /base_selection/cancel std_srvs/srv/Trigger '{}'
```

The `Trigger` response acknowledges that work started or was cancelled. The
asynchronous outcome is on `status` and `result`. New targets received while
busy are **rejected and not queued**; this is logged. Cancelling does not cancel
the existing reachability server computation because that service has no
cancellation interface. The worker drains its response, ignores the result,
and allows a new query. After a timeout, new work remains blocked until the
outstanding request finishes. A timed-out/cancelled planner action is cancelled
and tracked until terminal; late acceptance is handled too.

## Arm modes

| Mode | Selection rule |
|---|---|
| `left` | Left verified poses only |
| `right` | Right verified poses only |
| `either` | Union of both; choose an arm with the selected pose |
| `both` | Virtually identical XY and yaw in both verified response arrays |

The current reachability service tests **the same TCP target separately for
each arm**. `both` therefore means either arm can independently reach that same
target from the selected base pose. It is not a two-target bimanual planner.
The selector does not validate the two joint solutions simultaneously against
each other. Both solutions are included for inspection, not execution.

In `either`, duplicate base poses are checked once, retaining the solution with
less joint displacement. Approximate `left_candidates` / `right_candidates`
are never promoted to verified results.

## What the selector checks

1. Available TF, current arm joints, a fresh published footprint, fresh scan
   messages, active Nav2 planner/controller lifecycle nodes, and costmap services.
2. A service request with `target`, `time_budget: 20.0`, and
   `max_verified_per_arm: 150`.
3. Pose/solution correspondence, finite values and valid orientations in the
   response. Poses are transformed into `map` using TF.
4. Full convex footprint coverage at each candidate against the global costmap
   and the portion covered by the local costmap. The global map must cover the
   whole footprint. Being outside the local rolling window is not a rejection.
5. Preliminary ranking using straight-line distance, destination cost, heading
   change and joint displacement. Spatial/heading diversity moves nearby
   alternatives later in the order without deleting them.
6. Sequential `ComputePathToPose` requests with explicit start and goal. Check
   terminal action status, nonempty path, consistent frames and endpoint pose.
7. Full footprint sweeps along the path, including the start connector,
   initial/final rotations and interpolated segments. Both footprint interior
   and boundary are tested. Sweep hulls include a conservative angular chord
   padding so corners cannot slip through between yaw samples.
8. Rank paths, refresh both costmaps, and recheck candidates in score order
   until one survives. Update the selected path's cost/score on that fresh
   snapshot. This is the best earlier-ranked candidate that survives final
   checking; it is not a globally optimal search of every base pose.
9. Recheck base/arm posture and target-frame drift before publishing.

The node uses `nav2_msgs/srv/GetCostmap`, not the visualised
`nav_msgs/OccupancyGrid`, so costs are raw **0–255**. Cost **254** is lethal;
**255** is unknown and is rejected by default; **1–253** are soft costs when
checking the full footprint. Rejecting inflated cells under every footprint
point would effectively inflate the robot twice. Cell-boundary contact is
conservatively considered overlap. Concave footprints are converted to their
convex hull, which can reject some geometrically passable configurations.

The supplied footprint is `/local_costmap/published_footprint` (PolygonStamped).
It is transformed **back into `base_link` at its own timestamp** before being
placed at hypothetical candidate poses. This prevents applying the current
world pose twice. Its current arm posture is assumed to be the travel posture.
`footprint_padding` adds 0.02 m beyond whatever padding is already in that
published footprint.

## Scoring

Lower scores are preferred:

```text
score = 1.0 * (path_length_metres / 5.0)
      + 2.0 * mean_path_footprint_cost
      + 0.2 * (abs(shortest_final_heading_change) / pi)
      + 0.2 * mean_abs_joint_displacement_radians / pi
```

At each path sweep, the higher of the global/local mean footprint costs is
used. Those values are averaged across the sampled sweeps. Cost is a clearance
preference through the configured inflation layers; it is not an exact distance
to the nearest obstacle. Joint displacement uses actual joint coordinates,
without wrapping bounded joints through their limits. In `both`, use the larger
of the two arms' displacement scores.

**No manipulability or joint-limit-clearance score is fabricated.** Those are
not supplied by the existing service. Nor is a truncated response treated as a
complete workspace from which a true reachability margin could be inferred.
These can be added when the service exposes appropriate metrics.

The first shortlist contains up to 15 candidates. If none pass, the search
continues up to 60 path requests or the 30 s planning/checking budget. A planner
timeout aborts this selection attempt and cancels its action; it is not reported
as proof of physical unreachability. Some candidates will remain unexamined.

## Outputs and Lichtblick

| Topic | Type | Meaning |
|---|---|---|
| `/base_selection/selected_pose` | `geometry_msgs/msg/PoseStamped` | Chosen base pose, published once on success |
| `/base_selection/selected_path` | `nav_msgs/msg/Path` | Checked navigation path |
| `/base_selection/selected_valid` | `std_msgs/msg/Bool` | Snapshot result is within its TTL and not invalidated |
| `/base_selection/result` | `std_msgs/msg/String` | JSON result with request ID, arm, score, pose, joint solutions and limitations |
| `/base_selection/status` | `std_msgs/msg/String` | JSON phase/progress/failure |
| `/base_selection/candidate_markers` | `visualization_msgs/msg/MarkerArray` | Candidate arrows |
| `/base_selection/candidate_diagnostics` | `std_msgs/msg/String` | JSON per-candidate state and score |

Add the path and marker topics to the Lichtblick 3D panel in `map`:

- Cyan: selected pose.
- Green: path checked.
- Orange: destination clear; path not yet checked.
- Red: rejected or failed check.
- Grey: unexamined.

Diagnostics distinguish `global_lethal_obstacle`, `local_unknown_space`,
`endpoint_position_mismatch`, `endpoint_heading_mismatch`, planner terminal
statuses, `planning_timeout_or_error`, and fresh-map revalidation failures.

Results/status/diagnostics/path/validity use reliable transient-local QoS.
Selected pose is **volatile**, so a late subscriber does not receive an old
actionable pose. A consumer should correlate the `request_id` in `result`,
check its `valid` flag and respect `selected_valid`; there is no atomic ordering
guarantee across separate ROS topics. Result validity expires after 15 s and is
cleared by a new accepted request, failure, cancellation, or detected base/arm
motion. Obstacles are not continuously monitored after selection. A downstream
navigator must replan/check current conditions before and during execution.

## Parameters and tuning

Edit `config/base_selector.yaml`, rebuild if not symlinked, and restart the
selector. Runtime changes are rejected so an in-progress query cannot mix
configurations. The launch argument `arm_mode` overrides that YAML entry.
`selector_config` deliberately has its own launch argument name to avoid
colliding with the reachability launch's `config` argument.

| Parameter | Default | Meaning |
|---|---:|---|
| `time_budget` | 20.0 s | Reachability service calculation budget |
| `max_verified_per_arm` | 150 | Returned pose limit per arm |
| `reachability_timeout` | 30.0 s | Client wait limit; larger than server budget |
| `shortlist_size` | 15 | Initial path checks |
| `max_path_checks` | 60 | Expanded-search limit |
| `planning_budget` | 30.0 s | Path request/checking stage budget |
| `path_timeout` | 3.0 s | Per-action send/result wait limit |
| `footprint_padding` | 0.02 m | Additional travel footprint margin |
| `endpoint_xy_tolerance` | 0.02 m | Maximum planner endpoint displacement |
| `endpoint_yaw_tolerance` | 0.03 rad | Maximum planner endpoint yaw error |
| `max_data_age` | 3.0 s | Data/stream freshness bound |
| `selection_ttl` | 15.0 s | Snapshot result validity interval |
| `target_drift_tolerance` | 0.005 m | Allowed map/odom target translation drift |
| `target_rotation_drift_tolerance` | 0.01 rad | Allowed target rotation drift |

All angles in configuration are radians; positions/distances are metres.
These budgets are cooperative, not a hard real-time deadline for the entire
selection. Readiness, cancellation and final revalidation take additional time.

**Service truncation matters:** the inspected reachability implementation
publishes the full refined areas to its topics but truncates each service
verified array to `max_verified_per_arm`. The selector intentionally uses only
the service response. `search_truncated` is the cache-search flag, not a reliable
flag for this final response truncation. A response of 150 poses may omit valid
alternatives, including all common left/right poses. Raising the selector's
`max_verified_per_arm` towards the server's supported maximum of 1000 increases
coverage. It cannot return all 2000 refinement poses without changing that
server limit. An empty `both` intersection therefore does not prove that no
common placement exists.

If the planner ID differs, inspect:

```bash
ros2 param get /planner_server planner_plugins
ros2 interface show mur_reachability/srv/FindBaseCandidates
ros2 service list -t
ros2 topic echo /local_costmap/published_footprint --once
```

The configured arm joint names include the `_joint` suffix; change `arm_joints`
if the installed robot model uses different names. No invented circular
footprint is used as a fallback when the real footprint is missing.

## Include from mur_driver later

The package is standalone; the existing MUR launch is not changed. To include
it in `mur_driver`, import `IncludeLaunchDescription`, `PythonLaunchDescriptionSource`
and `get_package_share_directory` if they are not already imported, then add:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('mur_base_selector'),
        'launch', 'base_selector.launch.py')),
    launch_arguments={'arm_mode': 'either'}.items(),
)
```

The selector can start early and remain idle. Required services and live data
are checked when a target arrives. Add an `exec_depend` on `mur_base_selector`
to `mur_driver/package.xml` if that launch includes this package. There is no
reason to add the selector executable to `mur_driver/setup.py`; it belongs to
its own package.

## Validation and limits

Run the included portable tests:

```bash
cd ~/ros2_ws/src/MUR/mur_base_selector
PYTHONPATH=. python3 -m unittest discover -s test -v
```

The delivered version was tested with geometry tests and workflow tests using
ROS message/transport doubles. Tests cover full-footprint interior and corner
collisions, unknown/inflation policy, rotated map origins, partial local-map
coverage, translational and rotational sweeps, request settings, response
array mismatch, service failure, cancellation, final obstacle changes,
both-arm intersection and Humble action status/endpoint handling.

**ROS 2 is not installed in the generation environment.** Actual `colcon`
build, DDS/QoS, TF2 timing and live Nav2/MUR integration must be exercised in
the robot container. Python syntax compilation and portable tests are not a
substitute for that integration check.

This is a stationary, snapshot-based selector. It does not implement driving
while reachability runs, MoveIt environmental collision checking, arm-motion
planning, combined bimanual collision checking, or continuous goal switching.
It cannot guarantee physical navigation success from a successful global path.
Footprint sweeps validate piecewise linear XY/shortest-yaw interpolation of the
returned path, not a differential-drive controller's exact future trajectory.
Nav2 still owns runtime collision avoidance and execution.

Humble `GetCostmap` fills its response timestamps at request time. Scan
freshness, lifecycle state and fresh footprint publication are additional
guards; they do not prove every upstream obstacle layer is current. Verify
that both costmaps have the intended observation sources and robot footprint.

Before manipulation, validate the actually reached base pose and the arm plan
in the current MoveIt planning scene. The previously used 0.4 m navigation goal
tolerance can be far too loose for the verified arm workspace. This package
does not silently change Nav2 tolerances.

## Interface sources

- Existing `mur_reachability.zip`, version 3: `srv/FindBaseCandidates.srv` and
  `src/nodes.cpp` were inspected for response semantics and truncation.
- [Nav2 Humble ComputePathToPose](https://api.nav2.org/actions/humble/computepathtopose.html)
- [Nav2 Humble GetCostmap](https://api.nav2.org/srvs/humble/getcostmap.html)
- [Nav2 Humble costmap publisher implementation](https://api.nav2.org/nav2-humble/html/costmap__2d__publisher_8cpp_source.html)

Humble action results are handled via their terminal status; the code does
not depend on newer Nav2 `error_code` result fields or newer occupancy services.
