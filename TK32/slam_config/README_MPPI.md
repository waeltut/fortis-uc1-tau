# MUR Nav2 MPPI configuration (ROS 2 Humble)

## Files
- `config/nav2_params.yaml`: **MPPI enabled**, with polygon footprint collision checks.
- `config/nav2_params_dwb.yaml`: unmodified original DWB configuration for rollback/comparison.
- `slam_config/footprint_check.py`: read-only runtime check of dynamic-footprint wiring.
- `launch/nav2_launch.py`: original launch; no change needed for MPPI (it reads the YAML).
- `launch/navigation_launch.py`: alternate launch; behavior-server commands now go through the same velocity smoother as normal navigation.

The package was not tested on the actual MiR/Jetson hardware. The YAML is
syntactically valid and cross-checked against Humble MPPI parameters; perform
the live tests below before sending navigation goals near obstacles.

## Installation

From the ROS 2 Humble system / container that runs Nav2:

```bash
sudo apt update
sudo apt install ros-humble-nav2-mppi-controller
cd ~/ros2_ws
colcon build --symlink-install --packages-select slam_config
source install/setup.bash
```

Confirm the plugin is available:

```bash
ros2 pkg prefix nav2_mppi_controller
```

Launch via the same MUR launch setup as previously. If you run Nav2 separately:

```bash
ros2 launch slam_config nav2_launch.py
```

Note: confirm the deployed `slam_config` directory is replacing the original
source package (e.g. `~/ros2_ws/src/TK32/slam_config`). Restart Nav2 after
rebuilding since `controller_server` loads the plugin on activation.

## Key control settings

| Setting | Value | Reason |
| --- | --- | --- |
| `controller_frequency` / `model_dt` | 10 Hz / 0.1 s | Match MPPI prediction timestep to control cycle |
| `time_steps` / `batch_size` | 15 / 1000 | 1.5 s horizon; modest Jetson CPU starting point |
| `vx_max` / `vx_min` | 0.20 / -0.05 m/s | Cautious drive and slow reverse |
| `wz_max` | 0.40 rad/s | Matches the existing velocity smoother |
| `ax_max` / `ax_min` | 0.30 / -0.40 m/s² | Match smoother's physical limits |
| `az_max` | 0.60 rad/s² | Match smoother angular acceleration |
| `CostCritic.consider_footprint` | true | Use the local costmap's polygon for predicted collision checks |
| `CostCritic.collision_cost` | 1,000,000 | Heavily penalise predicted collisions; **not a safety-rated stop** |

The local and global costmap's existing fixed polygon is retained as a **fallback**.
This config alone cannot produce a dynamic footprint. The separate
`mur_footprint` package MUST publish the correct `geometry_msgs/msg/Polygon`
onto `/local_costmap/footprint` in the coordinates of the local costmap's
`robot_base_frame: base_footprint`.

**Do not publish a replacement footprint to
`/local_costmap/published_footprint`**: that is a `PolygonStamped` OUTPUT of
Nav2, transformed into the costmap's global frame (`odom`). The controller
uses the current costmap footprint internally; it is not subscribed directly
to the output topic.

Global and local costmaps are separate. If `mur_footprint` only publishes the
local input, the global planner will still use its static footprint; update
the separate publisher too if global footprint awareness is needed, using the
correct frame (`robot_base_frame: base_link` in the existing global config).

## Runtime verification BEFORE motion

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.CostCritic.consider_footprint
ros2 param get /controller_server controller_frequency
ros2 param get /local_costmap/local_costmap footprint
ros2 topic info /local_costmap/footprint -v
ros2 topic info /local_costmap/published_footprint -v
ros2 run slam_config check_dynamic_footprint
```

Expected controller plugin: `nav2_mppi_controller::MPPIController`.
Expected collision checking: `true`.
Expected input `/local_costmap/footprint` type: `geometry_msgs/msg/Polygon`
with at least one `mur_footprint` (or equivalent) publisher. Expected output
`/local_costmap/published_footprint` type:
`geometry_msgs/msg/PolygonStamped` from Nav2.

The costmap component parameter path may vary; if the footprint parameter
command says *node not found*, list the names with
`ros2 node list | grep costmap` and inspect that node's parameters. A topic
publisher existing is necessary but not sufficient: confirm that published
vertices correctly track each UR5e arm while the MiR is stopped. Allow enough
time for costmap TF updates. The footprint checker does not move anything.

### Check the entire path

```bash
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel
ros2 topic echo /local_costmap/published_footprint --once
ros2 topic info /global_costmap/footprint -v
```

Start testing in a wide, empty area with arms parked and protective systems
active. Compare observed response, controller CPU load, final pose accuracy,
clearance, and navigation success with the DWB baseline.

### Performance tuning

If controller cycles overrun or CPU utilisation is excessive, first reduce
`batch_size` to `500` (rebuild/restart). Do **not** disable footprint collision
checking as a quick fix for CPU load. `CostCritic.trajectory_point_step` is
not supported by the Humble implementation; setting it would not save CPU.

Your inflation radius remains **0.6 m** in both costmaps. This exceeds the
static rectangle's circumscribed radius (about 0.583 m), but extended arms
can make the dynamic footprint much larger. Humble MPPI recalculates the
circumscribed-footprint threshold as the robot's shape changes. If an extended
arm's radius is greater than the costmap inflation radius, optimised checks
may be less efficient. Tune using measured dynamic footprint size and
observed costmap behaviour, rather than blindly enlarging inflation.

## Rollback to DWB

The original YAML is preserved unchanged. Run the same launch with:

```bash
ros2 launch slam_config nav2_launch.py \
  params_file:="$(ros2 pkg prefix slam_config)/share/slam_config/config/nav2_params_dwb.yaml"
```

If your top-level `mur_launch` already passes `params_file`, set that launch
argument to the DWB YAML or copy the backup over `nav2_params.yaml` and rebuild.
This backup also restores the original smoother limits and 20 Hz controller.
