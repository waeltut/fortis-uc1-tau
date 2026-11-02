# Robot configuration

The selector supports one or several independent arm branches, identified by configurable names. Names do not need to be `left` or `right`.

## What is derived and what is explicit?

| Data | Source |
|---|---|
| Links, joints, geometry and URDF joint limits | MoveIt's `robot_description` parameter |
| Groups, SRDF allowed-collision matrix and semantic information | `robot_description_semantic` |
| IK plugins and additional planning/joint-limit settings | `robot_description_kinematics.*` and `robot_description_planning.*` copied from `source_node` during generation |
| Joint names monitored online | Selected groups' variable names stored in cache; `arm_joints: [auto]` |
| Arm collision branches, including attached tool/hand descendants in URDF | MoveIt robot model |
| Base-to-reference mount | TF during generation; checked online |
| Which groups are arms, desired TCP link and public arm label | Ordered `arm_names`, `arm_groups`, `arm_tips` arrays |
| Global/base/reference frames and Nav2 endpoints | Configuration |
| Reachability sampling bounds, resolution, tool-axis convention, field of view | Generation configuration |
| Preferred distance, facing sector and score weights | Selector configuration |
| Runtime footprint, map, robot pose and joint posture | ROS topics, services and TF |

URDF alone cannot say which group should be used, which tool point the task targets, or what a good working distance is. SRDF can contain grippers, combined groups and several possible end effectors; explicit mapping avoids selecting the wrong one. Joint-state monitoring checks freshness and motion, not online IK.

## Single-arm example

Copy `config/single_arm_engine.yaml` and replace the example group/link names:

```yaml
/**:
  ros__parameters:
    source_node: /move_group
    reference_frame: base_link
    base_frame: base_link
    arm_names: [manipulator]
    arm_groups: [arm]
    arm_tips: [tool0]
    joint_limit_margin: 0.02
    verification_position_tolerance: 0.001
    verification_orientation_tolerance: 0.01
```

Array positions correspond: `manipulator` uses group `arm` and TCP `tool0`. Names must be unique lowercase identifiers; `either`, `both` and `intersection` are reserved. Multiple branches must not share moving links. Each group must have a configured IK solver and the TCP must belong to its moving branch. Combined dual-arm groups are not substitutes for separate arm groups.

Copy `volume_selector.yaml`, choose a new `volume_file`, set the robot's frames/endpoints, remove MUR `left_*`/`right_*` preference entries and add:

```yaml
    arm_joints: [auto]
    arm_mode: either
    manipulator_preferred_distance: 0.65
    manipulator_facing_min_deg: -20.0
    manipulator_facing_max_deg: 20.0
```

Unspecified per-arm preferences default to 0.65 m and a -60° to +60° facing sector. `both_*` preferences apply only to intersection mode. The MUR profile explicitly preserves its original asymmetric left/right sectors and distance preferences.

Set `reference_frame` to a URDF link rigidly upstream of every configured arm, and `base_frame` to the planar navigation base whose pose Nav2 controls. The transform between them must remain fixed. Adjust workspace bounds and sampling in your generation YAML and set the same output cache path in the selector YAML. The installed MUR cache is never suitable merely because another robot has similarly named frames.

Generate with explicit files:

```bash
ros2 run mur_base_selector build_volume \
  --engine-config /absolute/path/my_robot_engine.yaml \
  --config /absolute/path/my_volume_generation.yaml \
  --output /absolute/path/my_robot_volume.npz --workers 2
ros2 launch mur_base_selector base_selector.launch.py \
  selector_config:=/absolute/path/my_volume_selector.yaml arm_mode:=either
```

Workers are capped at arm count. One arm uses one worker. Parallel arms have independent native model/scene owners. `--estimate-only` reports checks per arm without loading MoveIt. All arms currently share generation bounds, tool-axis convention and orientation lattice; choose bounds covering all arms and consistent TCP axes.

## When to rebuild

Generate a separate cache after changing geometry, TCP links, arm mapping, mounting, joint limits, SRDF collision policy, IK configuration or sampling settings. Scoring weights and per-arm distance/facing preferences do not require rebuilding. Runtime profile filters may tighten existing orientation evidence, but cannot create untested orientations. Increasing the sampled FOV or changing its front offset requires regeneration.

The online node checks exact URDF/SRDF strings and mount against the cache; even a harmless URDF textual change can trigger rejection. It does not currently recompare all `robot_description_planning` or kinematics parameters online, so regenerate when those change. The builder includes them in its resume fingerprint.

The provided completed two-arm schema-2 cache remains supported with its fixed original axis order: left, right. New caches explicitly store arm order. Old generation checkpoints cannot be resumed across the new native model fingerprint; keep a completed cache for selection or generate to a new output path.

## Remaining assumptions

- Planar base motion; no slope/roll/pitch search and no changing torso mount.
- Independent arm branches; shared movable torso joints are unsupported.
- Other moving arms are excluded from offline per-arm collisions. Real execution needs full-scene collision checks.
- Non-arm movable joints and hand joints outside the selected IK groups use the offline model's default posture. Changing geometry through these joints needs a matching cache/model or additional validation.
- The footprint source must describe the robot's navigation posture; a small footprint that omits protruding arms can invalidate navigation clearance assumptions.
- FOV and tool-axis constraints are application choices, not facts recoverable from URDF.
- Output names remain `/base_selection/...`; ROS remapping can change them. Nav2 and input endpoints are parameters. Use separate remappings and configurations for multiple selectors.
