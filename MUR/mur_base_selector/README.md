# Mobile manipulation base selector

Given a stamped TCP target pose, choose a mobile-base pose and arm using a sampled dexterous-space cache, working-distance/facing preferences, costmaps and Nav2 path checks. The selector does not move the base or arms and performs no online IK.

## Packages

- `mur_base_selector`: offline MoveIt cache builder, online selector, visualizer and configuration. Includes the native C++ engine and Python nodes.
- `mur_base_selector_interfaces`: the `SelectBase` service definition, so clients can depend on the interface without the MoveIt implementation.

The separate `mur_reachability` implementation package is no longer needed. Keeping interface generation separate avoids the ROS 2 Humble `rosidl_generate_interfaces` / `ament_python_install_package` build conflict.

## Install

Replace the old `mur_base_selector` source directory with this one. Move the old `mur_reachability` package outside `ros2_ws/src` and place `mur_base_selector_interfaces` beside the selector. Do not overlay new files on the old package: the build type is now `ament_cmake`.

In a fresh shell, source ROS Humble but do not source an old workspace overlay before rebuilding:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
# These are generated package-specific build/install directories, not source files.
rm -rf build/mur_base_selector install/mur_base_selector
rm -rf build/mur_reachability install/mur_reachability
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to mur_base_selector
source install/setup.bash
```

These cleanup commands assume the usual isolated install layout, not `--merge-install`. Existing launch calls using `mur_base_selector/base_selector.launch.py` keep working. Update any external package manifest depending on `mur_reachability` to depend on `mur_base_selector` instead.

The supplied cache is unchanged and remains readable. Its robot description, semantic model and mount must still match the running robot. It is not a cache for other robots.

## Start

Start MoveIt, TF, joint states, scan input and active Nav2 planner/controller/costmap services first. Then:

```bash
ros2 launch mur_base_selector base_selector.launch.py
```

`selector_config` selects the online YAML. `arm_mode` defaults to `either` and overrides the YAML value; it can name an individual configured arm or `both`. `use_sim_time` must match the robot stack.

## Request/response service

```bash
ros2 service call /base_selection/select mur_base_selector_interfaces/srv/SelectBase \
  "{target: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.2, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, arm: either}"
```

Target coordinates are an example, not a prescribed robot motion. The target is a **full TCP pose**: orientation affects reachability. An omitted/zero quaternion is rejected. A zero timestamp requests the latest transform; a nonzero timestamp is honoured when a transform is needed. The target is then frozen in `fixed_frame`.

Request `arm`:

- Empty string: use the configured `arm_mode`.
- `either`: select one configured arm according to score.
- A configured name, e.g. `left`: restrict this request to that arm without changing topic behaviour or configuration.
- `both`: require independent cache evidence for every configured arm at the same base pose. This is an intersection query, **not simultaneous multi-arm feasibility**.

Response:

| Field | Meaning |
|---|---|
| `success`, `message` | Success or an explicit failure/busy/cancellation explanation |
| `request_id` | Accepted request sequence; rejected requests use zero |
| `navigation_goal` | Selected `PoseStamped` in `fixed_frame`, ready for a separate Nav2 client |
| `arm` | Selected configured name, or `both` for an intersection query |
| `planning_groups`, `tcp_frames` | Corresponding MoveIt group(s) and TCP link(s) from cache metadata |
| `score` | Lower is better; selection is bounded and not globally optimal |
| `valid_for_seconds` | Validity horizon measured from selection, not receipt |
| `diagnostics_json` | Same successful result metadata as the result topic |

Only use goal/arm/score fields when `success` is true. The service waits for selection, rather than merely acknowledging submission. It uses the same validation and publications as topic requests. Concurrent requests are rejected, not queued. A service client timing out does not cancel server work: explicitly call `/base_selection/cancel` if needed. Budget, service and planner cancellation timeouts can make total response time exceed `selection_budget`; set client timeout with margin.

For Python clients, import `SelectBase` from `mur_base_selector_interfaces.srv`, create a client and use `call_async`. Do not block a mutually exclusive callback that is also needed to receive its response.

## Existing topic interface

```bash
ros2 topic pub --once /base_selection/target geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.2, z: 1.0}, orientation: {w: 1.0}}}'
ros2 service call /base_selection/reselect std_srvs/srv/Trigger '{}'
ros2 service call /base_selection/cancel std_srvs/srv/Trigger '{}'
```

`reselect` acknowledges asynchronous submission using the default arm mode and the last frozen target. It does not preserve a previous per-service arm override.

| Output | Type / purpose |
|---|---|
| `/base_selection/selected_pose` | PoseStamped; selected navigation pose |
| `/base_selection/result` | String JSON; arm, score and evidence |
| `/base_selection/selected_valid` | Bool; validity, expiry or invalidation |
| `/base_selection/selected_path` | nav_msgs/Path; checked planning path |
| `/base_selection/status` | String JSON; progress/failure reason |
| `/base_selection/candidate_diagnostics` | String JSON; candidate states |
| `/base_selection/candidate_markers` | MarkerArray; candidates |
| `/base_selection/target_markers` | MarkerArray; requested TCP target |
| `/base_selection/visualization_data` | String JSON; visualization snapshots |

Service requests also publish these outputs. Avoid running a topic consumer that automatically navigates if the service caller also sends navigation goals; otherwise two coordinators could react to one selection.

## Configure a robot

See [robot_configuration.md](docs/robot_configuration.md) for a single-arm example, automatic model inputs, assumptions and cache rebuild rules. Your MUR settings remain in `config/volume_engine.yaml`, `config/volume_generation.yaml` and `config/volume_selector.yaml`.

- [Cache generation](docs/cache_generation.md)
- [Online parameters and scoring](docs/selector_parameters.md)
- [Visualization](docs/visualization.md)
- [On-demand OccupancyGrid score maps](docs/score_visualizer.md)

## Scope of the result

The cache checks sampled endpoint IK, joint bounds and SRDF self/body collisions offline. Other configured moving arm branches are ignored during each arm's offline check. Environment arm collisions, exact requested-pose IK and arm trajectory feasibility are **not checked** by selection. `both` does not certify combined collision-free arm motion. After navigation, transform the original TCP target using the actual base pose and ask MoveIt to plan with the current complete collision scene.

The base is assumed planar and arms rigidly mounted relative to the cached reference frame. Mobile torso/lift configurations need separate caches or a more general model. Current base/arm motion and stale input cause rejection. Nav2 path validation uses the current padded footprint; maintain that posture during navigation or revalidate. Changes in the world can invalidate a returned pose before its TTL expires.

## Verification

```bash
colcon test --packages-select mur_base_selector
colcon test-result --verbose
# Pure Python algorithm and transport-double suite, no ROS installation required:
PYTHONPATH=src/MUR/mur_base_selector python3 -m unittest discover \
  -s src/MUR/mur_base_selector/test -v
```

The archive was checked with the Python tests and the supplied cache. This editing environment has no ROS/MoveIt installation, so C++ compilation, generated service imports, DDS/executor integration and a live Nav2 query still require verification in the ROS Humble workspace. The tests using transport doubles do not substitute for those checks.
