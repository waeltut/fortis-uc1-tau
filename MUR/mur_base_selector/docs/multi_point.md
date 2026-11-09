# Multi-point base selection

The service `/base_selection/select` (`mur_base_selector_interfaces/srv/SelectBase`)
accepts multiple full TCP poses and finds one shared base pose, including heading.
The selection itself never starts navigation or arm movement.

## Request

- `targets`: PoseStamped array; order defines P1, P2, … for clients.
- `arms`: empty array for all `any`, or exactly one string per target. Empty
  entries also mean `any`. `either` is accepted as an alias for `any`.
- A nonempty `targets` array supersedes the legacy `target` and `arm` fields.
- With `targets` empty, `target` and `arm` retain the single-point service form.
  Empty `arm` means `any`. Nonempty `arms` without `targets` is rejected.

| Arm requirement | Evidence required at the shared XY/yaw pose |
|---|---|
| any / empty | At least one configured arm can reach; it may differ between points |
| left | Left arm can reach this point |
| right | Right arm can reach this point |
| both | Every configured arm can individually reach this point |

Other configured arm names remain supported for other robots. All poses require a
frame, finite positions and valid nonzero quaternion (normalised by the selector).
Each pose is transformed independently into `fixed_frame` before the lookup.
Zero stamps use latest TF; nonzero stamps request their stated transform time.

Example (replace these coordinates and orientations for your scene):

```bash
ros2 service call /base_selection/select mur_base_selector_interfaces/srv/SelectBase \
  '{targets: [
    {header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0, z: 1.3}, orientation: {y: 0.7071068, w: 0.7071068}}},
    {header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.2, z: 1.3}, orientation: {y: 0.7071068, w: 0.7071068}}},
    {header: {frame_id: map}, pose: {position: {x: 2.4, y: 0.1, z: 1.2}, orientation: {y: 0.7071068, w: 0.7071068}}}
  ], arms: [left, right, any]}'
```

In Python:

```python
request = SelectBase.Request()
request.targets = [p1, p2, p3]  # geometry_msgs.msg.PoseStamped
request.arms = ['left', 'right', 'both']
future = client.call_async(request)
# On success: response.navigation_goal; response.point_arms[i] for each Pi.
```

## Intersection and score

P1 supplies sampled candidate base XY/yaw poses. Every later point is evaluated
at those exact base poses. Independent translated target grids are not intersected
by exact floating-point equality, and XY projections at different headings are
never treated as a common solution.

For later points the relative TCP XY is mapped to the nearest cache voxel centre
on the selected height layer (at most half a voxel per horizontal axis). An absent
voxel is rejected; a successful neighbouring voxel is not substituted for a failed
nearest voxel. Height and orientation use the existing configured tolerances.
Actual target bearing, front FOV and tool-direction constraints are checked too.
Coverage, requested orientation bit, completion and per-arm profile checks still
apply. This is sampled-volume evidence: exact requested-pose IK is not verified.
P1 determines the finite candidate grid, so point order can affect sampled results;
no continuous-space/global optimality guarantee is made.

For each common pose, an `any` point chooses its lowest working-distance/facing
penalty arm. Those penalties are averaged across points. Navigation/path and
destination-cost terms are counted once. Existing Nav2 path, footprint, fresh-map,
robot-motion guards and global selection budget are retained. More targets may
need a larger `selection_budget`; increase the client's `selection_timeout` too.
An empty intersection or exhausted budget returns `success: false`; it is not a
proof that no continuous-space solution exists.

## Response and diagnostics

`point_arms` is one resolved arm (`left`, `right`, `both`, or configured name) per
input point, in order. `arm` is that common value if all assignments agree,
otherwise `mixed`. Multi-point consumers must use `point_arms`.
`planning_groups` and `tcp_frames` are the corresponding unique used arms in
first-use order; they are not indexed one-to-one by point.

The result JSON includes `point_arms`, normalised `point_requirements`, and
`point_targets` as `[x,y,z,qx,qy,qz,qw]` in `fixed_frame`. Existing top-level
coverage/orientation/profile fields and target marker describe P1; those fields
are not a per-point evidence array. Candidate visual snapshots show the common
feasible set with the P1 arm colouring. The separate on-demand score visualizer
continues to show single-target scores, not multi-point workspace scores.

`reselect` preserves all frozen points and requirements for an array request.
Single-target topic requests still use the configured `arm_mode` and publish the
usual outputs. Single-target reselect continues to use the configured arm mode.
Concurrent requests are rejected. The existing cancel service applies to the
whole workspace query. Client timeout alone does not cancel server work.

## Interactive workflow

See `mur_test/README.md`. Its height relay sends clicks to `/mur_test/point`.
The test collects points and arm requirements, submits the service on `done`,
asks before navigating, then repeatedly offers P1/P2/…, `home`, `new`, `quit`.
For `both`, choose one arm per visit; both TCPs are not sent to the identical pose
simultaneously. `home` uses `/arms/move` with the commander's `home` preset.

Rebuild `mur_base_selector_interfaces`, the selector and all service clients,
then restart them. ROS service type compatibility requires the same regenerated
interface on every machine. No cache regeneration is needed solely for this
interface/lookup change. Keep your full existing dexterous cache.
