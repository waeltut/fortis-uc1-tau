# Selector parameters

Edit `config/volume_selector.yaml` under `mur_base_selector.ros__parameters`.
Pass it with `selector_config:=/absolute/path/volume_selector.yaml` and restart
the selector. Source YAML edits need no rebuild when passed explicitly. To inspect
what actually loaded: `ros2 param get /mur_base_selector selection_budget`.

Defaults below are the distributed YAML defaults. Tune your own values as needed.
ROS `use_sim_time` is supplied by the launch argument (default false).

| Parameter | Default | Meaning |
| --- | --- | --- |
| `fixed_frame` | `map` | Frame for anchored target, candidate poses and output paths. |
| `base_frame` | `base_link` | Mobile-base frame; must match cached base_frame. |
| `reachability_frame` | `odom` | Intermediate frame used only to explain TF age failures (usually odom). |
| `arm_mode` | `either` | left, right, either (choose one), or both (independent evidence for both). Launch argument overrides YAML. |
| `service_timeout` | `3.0` | Maximum service wait/call duration, seconds, also bounded by remaining query time. |
| `planner_action` | `/compute_path_to_pose` | Nav2 ComputePathToPose action name. |
| `planner_id` | `GridBased` | Planner plugin ID sent to Nav2. |
| `global_costmap_service` | `/global_costmap/get_costmap` | GetCostmap service for global navigation and full destination/path footprint coverage. |
| `local_costmap_service` | `/local_costmap/get_costmap` | GetCostmap service for additional checks inside the local map. |
| `footprint_topic` | `/local_costmap/published_footprint` | PolygonStamped current robot footprint, including current arm extent. |
| `joint_states_topic` | `/joint_states` | JointState stream used for freshness and movement checks. |
| `scan_topic` | `/scan` | LaserScan stream used for freshness checks. |
| `lifecycle_services` | `["/planner_server/get_state", "/controller_server/get_state"]` | GetState service names; all listed nodes must be active. |
| `arm_joints` | `["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint", "right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint", "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint"]` | Joint names required for current-state freshness and motion monitoring. |
| `max_data_age` | `3.0` | Maximum accepted sensor/state/costmap age, seconds. |
| `tf_timeout` | `0.5` | Maximum TF lookup wait, seconds. |
| `footprint_padding` | `0.02` | Extra polygon expansion, metres, applied to the received footprint. |
| `allow_unknown` | `false` | Allow costmap value 255 through custom footprint/travel checks; Nav2 may still reject unknown space. |
| `max_path_checks` | `8` | Maximum candidate poses sent to Nav2 per query. One path request per candidate. |
| `path_timeout` | `1.0` | Maximum wait for a Nav2 path request, seconds, bounded by remaining query budget. |
| `cancel_timeout` | `2.0` | Maximum wait for planner cancellation acknowledgement, seconds. |
| `diversity_distance` | `0.2` | Candidate spatial diversity scale, metres; affects shortlist ordering. |
| `diversity_angle` | `0.35` | Candidate heading diversity scale, radians; affects shortlist ordering. |
| `path_linear_step` | `0.05` | Maximum translation step between swept-footprint subdivisions, metres. |
| `path_angular_step` | `0.08` | Maximum yaw step between swept-footprint subdivisions, radians. |
| `start_xy_tolerance` | `0.05` | Allowed offset of returned path start from requested start, metres. |
| `endpoint_xy_tolerance` | `0.02` | Allowed path endpoint position error, metres. |
| `endpoint_yaw_tolerance` | `0.03` | Allowed path endpoint heading error, radians. |
| `base_motion_tolerance` | `0.03` | Maximum base displacement during selection/validity monitoring, metres. |
| `base_rotation_tolerance` | `0.03` | Maximum base yaw change during selection/validity monitoring, radians. |
| `joint_motion_tolerance` | `0.04` | Maximum monitored arm joint-position change, radians for revolute joints. |
| `length_scale` | `5.0` | Navigation score normalisation distance, metres. |
| `selection_ttl` | `15.0` | Published selection validity duration, seconds; movement can invalidate earlier. |
| `publish_visualization_data` | `true` | Publish stage snapshots for the viewer; false avoids their collection/serialization overhead. |
| `visualization_max_candidates` | `6000` | Maximum displayed samples per diagnostic stage. Exact stage counts remain available. |
| `selection_budget` | `12.0` | Overall selection time budget, seconds. Counts and success targets can stop search earlier. |
| `final_validation_reserve` | `2.0` | Seconds reserved within selection_budget for fresh costmaps and final checks. |
| `ranking_pool_size` | `160` | Maximum ranked unique base poses entering the shortlist. |
| `max_destination_checks` | `80` | Maximum candidate destination footprint checks. |
| `target_feasible_paths` | `3` | Stop candidate search after this many valid paths; 1 accepts the first feasible candidate subject to final revalidation. |
| `front_fov_deg` | `250.0` | Allowed position bearing span around front, degrees; cannot exceed cached workspace FOV. |
| `front_yaw_offset_deg` | `0.0` | Front bearing offset, degrees; must equal cached workspace_front_offset_deg. |
| `volume_file` | `~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz` | Completed NPZ cache path; ~ is expanded. No implicit fallback to a different cache. |
| `travel_field_budget` | `2.0` | Seconds allowed for global travel-field expansion; unsettled cells are excluded if truncated. |
| `yaw_bins` | `24` | Number of uniformly spaced base headings in a full rotation; 24 gives 15-degree steps. |
| `orientation_profiles_deg` | `[70.0, 90.0, 120.0]` | Derived minimum inward-angle profiles. Must be >= cached generation angle and <180; the cached angle is always retained. |
| `minimum_orientation_coverage` | `0.9` | Required fraction of successful allowed samples within an eligible profile; 0–1. Target orientation bit must also succeed. |
| `max_height_error` | `0.05` | Maximum difference from a sampled height, metres; only nearest eligible height layer is used. |
| `max_orientation_error_deg` | `30.0` | Maximum rotation error between requested TCP orientation and nearest stored quaternion, degrees. |
| `travel_inflation_weight` | `3.0` | Soft inflation multiplier for travel-field ranking and path navigation cost. |
| `weight_navigation` | `1.0` | Weight of normalised navigation cost. |
| `weight_destination_cost` | `2.0` | Weight of destination footprint cost. |
| `weight_working_distance` | `1.0` | Weight of squared preferred working-distance error. |
| `weight_facing` | `0.5` | Weight of squared deviation outside the preferred facing sector. |
| `distance_scale` | `0.25` | Working-distance error normalisation distance, metres; smaller means stronger distance penalty. |
| `left_preferred_distance` | `0.65` | Preferred horizontal base-to-target distance for left mode, metres. |
| `right_preferred_distance` | `0.65` | Preferred horizontal base-to-target distance for right mode, metres. |
| `both_preferred_distance` | `0.65` | Preferred horizontal base-to-target distance for both mode, metres. |
| `left_facing_min_deg` | `0.0` | Lower target-bearing bound of the zero-penalty facing sector for left mode, degrees. |
| `left_facing_max_deg` | `60.0` | Upper target-bearing bound of the zero-penalty facing sector for left mode, degrees. |
| `right_facing_min_deg` | `-60.0` | Lower target-bearing bound of the zero-penalty facing sector for right mode, degrees. |
| `right_facing_max_deg` | `0.0` | Upper target-bearing bound of the zero-penalty facing sector for right mode, degrees. |
| `both_facing_min_deg` | `-10.0` | Lower target-bearing bound of the zero-penalty facing sector for both mode, degrees. |
| `both_facing_max_deg` | `10.0` | Upper target-bearing bound of the zero-penalty facing sector for both mode, degrees. |

## Tuning examples

Allow a larger search and compare more base destinations:

```yaml
selection_budget: 30.0
max_path_checks: 20
target_feasible_paths: 10
```

A longer time budget alone does not increase the candidate/path count limits.
Nav2 computes one path for each candidate sent to it. Comparing different base
poses is separate from optimising multiple paths to the same pose. Final validation
reuses the returned path against fresh costmaps.

Prefer forward-facing base poses more strongly:

```yaml
weight_facing: 5.0
left_facing_min_deg: 0.0
left_facing_max_deg: 15.0
right_facing_min_deg: -15.0
right_facing_max_deg: 0.0
both_facing_min_deg: -10.0
both_facing_max_deg: 10.0
```

This changes ranking, not reachability. It cannot restore candidates that fail
cache coverage, target orientation, height or navigation-field filters. Inspect
the separate candidate-stage displays before adjusting limits blindly.

## Cache impact

Online budgets, scoring, profiles and display parameters do not require new IK.
Profiles can only narrow the cached orientation set. Increasing the workspace
bounds/FOV beyond what was generated, changing the robot geometry/limits/TCP/mount,
or changing offline sampling requires generating a matching cache.
