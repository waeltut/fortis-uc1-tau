# On-demand OccupancyGrid score maps

`score_visualizer` is a separate diagnostic process. It is not started by the normal selector launch and adds no subscriptions, calculations or publications to the normal selector. Launch it for an inspection session, trigger calculations explicitly, and stop it afterward.

It never publishes navigation goals or selection results. Existing selection topics and `/base_selection/select` are unchanged. All maps are diagnostic cost displays, not occupancy probabilities or Nav2 costmap inputs.

## Build and launch

After updating the package, rebuild it (the interfaces package is unchanged):

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select mur_base_selector --cmake-clean-cache
source install/setup.bash
ros2 launch mur_base_selector score_visualizer.launch.py
```

MoveIt model parameter services, TF, joint states, scan and the configured Nav2 lifecycle/costmap services must be available. Fast mode does not require a footprint or make Nav2 path requests. Detailed mode also needs the current published footprint and planner action. Keep the robot stationary during a calculation. Run detailed mode while ordinary selection/navigation is idle: its requests use the same Nav2 planner server and consume computation there.

The launch reads the same `volume_selector.yaml` as normal selection so score weights, arm preferences, cache and Nav2 endpoints match. For custom tuning:

```bash
ros2 launch mur_base_selector score_visualizer.launch.py \
  selector_config:=/absolute/path/volume_selector.yaml \
  diagnostic_config:=/absolute/path/score_visualizer.yaml \
  arm_mode:=either mode:=fast
```

Use `use_sim_time:=true` only if the robot stack also uses simulation time. The launch reads the `mur_base_selector`, `/mur_base_selector` and/or `/**` parameter entries from the selector YAML and applies them to the diagnostic node. It does not fetch parameters from an already-running selector. Match any additional overrides you used for that selector.

## Target and trigger

After starting the visualizer, publish a target as usual on `/base_selection/target`; the visualizer stores it but does not calculate. The normal selector will still react to that usual topic.

To inspect without triggering normal selection, use the dedicated target input:

```bash
ros2 topic pub --once /base_selection/score_target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.2, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

These coordinates are an example. Use your intended full TCP pose. The most recently received valid message from either target topic becomes the diagnostic target. Targets published before the diagnostic node started are not recovered. A new target during a calculation is stored for the next trigger; it does not alter the current calculation.

Calculate once:

```bash
ros2 service call /base_selection/score_map std_srvs/srv/Trigger '{}'
ros2 topic echo /base_selection/score_map/status
```

The Trigger response acknowledges that work started; wait for status `complete`. Overlapping triggers are rejected. The frozen target, captured configuration and result summary are in `/base_selection/score_map/metadata` (String JSON).

Cancel a calculation:

```bash
ros2 service call /base_selection/score_map/cancel std_srvs/srv/Trigger '{}'
```

Start/failure/cancellation clears old map and arrow outputs. Completed maps remain transient-local published while the node stays alive. No periodic recalculation or timestamp refresh occurs. To preserve a session beyond node shutdown, record these topics with your normal recording tool.

## View in Lichtblick

In the 3D panel use the same fixed frame as the selector (normally `map`) and enable:

| Topic under `/base_selection/score_map/` | Type / meaning |
|---|---|
| `fast/total` | OccupancyGrid: total approximate score |
| `fast/travel` | OccupancyGrid: weighted travel contribution |
| `fast/destination` | OccupancyGrid: weighted destination centre-cell cost |
| `fast/distance` | OccupancyGrid: weighted preferred-distance penalty |
| `fast/facing` | OccupancyGrid: weighted facing-sector penalty |
| `fast/state` | OccupancyGrid: evaluation status, not a score |
| `fast/winners` | MarkerArray: heading arrows, namespace and colour identify arm |
| `detailed/total`, `detailed/travel`, etc. | Same layers using successful detailed checks |
| `detailed/state`, `detailed/winners` | Detailed evaluation status and winning arrows |
| `metadata` | String JSON: counts, scale, raw score ranges, clipping, configuration and snapshot time |
| `status` | String JSON: idle/calculating/complete/failed/cancelled |

Enable one score layer at a time to avoid overlapping maps hiding each other. Arrow namespaces use your configured arm names. Arrows are drawn at the real sampled XY position with its real candidate heading; they are not snapped to cell centres. Arrow display is limited by `max_markers`; the grid still contains every scored cell.

The timestamp is the diagnostic snapshot acquisition time, not the current display time. Use `fixed_frame` to display old snapshots without requiring old moving-frame transforms. A map is historical even when it remains visible.

## How to interpret cells

A candidate is an XY position **plus heading and arm**. The grid groups candidates into XY cells and chooses the one with the lowest total score in each cell. Every component map shows the contribution of that same winning candidate. It does not independently choose a different arm/heading for each component.

Score layers use a fixed linear scale:

- 0: zero cost, best possible score under these non-negative terms.
- 100: cost at or above `diagnostic.score_max` (clipped).
- -1: no scored candidate. This can mean no cache candidate, no travel estimate, or no successful detailed check. It is not proof of unreachability.

The best observed candidate need not have value 0. No interpolation fills gaps, and a scored cell does not certify every position inside it. Different base heights and continuous headings/orientations outside the sampled lattice are not searched.

**State layers have a different, categorical meaning:**

| Value | Meaning |
|---|---|
| -1 | No sampled candidate in this cell after filters |
| 0 | At least one candidate has a score (estimate in fast mode; passed checks in detailed mode) |
| 50 | No scored candidate, but at least one candidate has no estimate or remains unchecked |
| 100 | Every sampled candidate in this cell was rejected by detailed destination/path/planner checks |

A fast state of 0 is not a footprint/path clearance certificate. A detailed state of 0 can coexist with unchecked headings/arms in that cell; it only establishes that at least one sampled candidate passed diagnostic checks. Metadata reports counts per candidate, not merely per cell.

## Fast versus detailed

**Fast (default):** query all cache candidates under current coverage/orientation/FOV/height/arm/heading filters, compute one bounded travel field from the current base, and evaluate each candidate with a settled finite travel estimate. Cost is identical to the normal selector's preliminary scoring formula: weighted travel, global centre-cell cost, preferred distance and facing. It bypasses the normal shortlist and path-count limits. This mode deliberately does not perform footprint or path collision checks.

Travel-field truncation is recorded in metadata. Unsettled/out-of-map/blocked/disconnected locations do not receive travel scores and remain unknown; these causes are not conflated with confirmed infeasibility. Increase `diagnostic.travel_budget` and, if necessary, `diagnostic.calculation_budget` to inspect more of a large map.

**Detailed:** publishes the fast map, then checks candidates in ascending approximate-score order, followed by candidates lacking a travel estimate. It uses the captured footprint and costmaps for destination/swept-path checks, and requests a Nav2 path for each destination that passes. Successful candidates use the normal selector's final score formula (validated path length and path cost instead of the travel-field estimate). No exact target IK is performed.

```bash
# Stop the previous visualizer before starting another with the same outputs.
ros2 launch mur_base_selector score_visualizer.launch.py mode:=detailed
```

Limits are `diagnostic.max_detailed_candidates` and `diagnostic.calculation_budget`. Unchecked candidates retain unknown status. Planner timeouts/transport uncertainty stop further checking and leave the unresolved candidate unchecked; a failed planner attempt is not a mathematical proof that no path exists. Metadata reports `detailed_partial`, destination checks and path requests. It is possible for all cells with successful checks still to have unchecked alternatives.

Nav2 computes paths against its live maps, whereas diagnostic footprint/path validation uses captured maps. There is no final fresh-map selection check. Thus detailed results are still **historical diagnostics**, never execution-ready selections. Do not run detailed diagnostics concurrently with navigation or another selection using the same planner.

## Configuration

Edit `config/score_visualizer.yaml`, then restart only the diagnostic node. If launching with defaults, installed YAML is used; pass `diagnostic_config` to a source/custom file when tuning without rebuilding.

| Parameter prefix `diagnostic.` | Default | Meaning |
|---|---|---|
| `mode` | fast | `fast` or `detailed`; launch `mode` argument overrides YAML |
| `grid_resolution` | 0.10 m | XY aggregation cell size; does not change cache sampling |
| `score_max` | 10.0 | Fixed upper scale shared by total and weighted components |
| `max_cells` | 1 000 000 | Allocation cap; increase cell size if exceeded |
| `max_markers` | 1 500 | Maximum winning arrows per map |
| `calculation_budget` | 60 s | Cooperative calculation budget; service/TF/cancellation and publication work can add overhead |
| `travel_budget` | 15 s | Travel field expansion budget, within total budget |
| `max_detailed_candidates` | 200 | Maximum detailed destination checks, including blocked destinations |
| `heading_filter` | false | Restrict the sampled base yaw bins |
| `heading_deg` | 0° | Desired base yaw in `fixed_frame` |
| `heading_tolerance_deg` | 7.5° | Maximum wrapped angular difference from desired yaw |

For one arm, launch with `arm_mode:=left` (or your configured name). For one sampled heading, enable the heading filter and use a tight tolerance around a yaw bin. With 24 yaw bins, bin spacing is 15°. The filter selects existing bins rather than generating a new continuous heading.

To compare score weights, use the same target, base pose, map, resolution and fixed `score_max` in each diagnostic run. Supply separately edited selector YAML files to the diagnostic launch; this does not change the running production selector. Metadata includes the complete effective configuration and unclipped score ranges. If many cells clip at 100, raise the scale for **all** runs being compared.
