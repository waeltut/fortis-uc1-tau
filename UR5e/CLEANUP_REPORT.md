# Cleanup report — 25 September 2026

## Applied changes

1. **Dual-arm interface:** removed the nonexistent Python commander's console entry; connected the existing left/right group arguments; preserved the working `execute` argument with default `false`; selected OMPL explicitly when building the commander's configuration; corrected package version/description and runtime dependencies.
2. **C++ utility package:** retained only `dual_arm_pose_commander`; removed the obsolete interface, table, and program executables; reduced CMake and package dependencies; added explicit geometry/message package discovery and thread linkage. Removed unused callback-group options, unused NodeOptions, and the unused chrono include; added the directly used functional include. Planning, IK, worker callbacks, initialization, and execution logic are unchanged.
3. **Legacy interfaces:** removed `interfaces_pkg` after confirming all in-archive consumers were retired code.
4. **Real bringup:** removed the old interface node, unused helper-node constructions, obsolete arguments, and commented launch experiments. Preserved defaults, addresses, ports, remappings, controller lists, and `use_sim_time=False`.
5. **Vendor separation:** moved the dual-arm controller YAML into `duo_ur/config`; updated its default package and kept update-rate files sourced from `ur_robot_driver`; removed the duplicate project-specific vendor launch. All remaining vendor files are byte-for-byte unchanged.
6. **Legacy description material:** removed the old `-duo_ur_ee_moveit_config` package, underscore-prefixed launch/model experiments, stale generated URDFs and unused controller/initial-position files. The extra reference scan also allowed removal of unused local UR mesh copies and the empty vineyard world. Active arm meshes continue to come from `ur_description`; all hand meshes and the connector mesh remain.
7. **Active model:** removed dead commented geometry and unused properties; renamed the two fixed TCP joints to `left_hand_base_to_tcp` and `right_hand_base_to_tcp`. Link/TF frame names, transforms, meshes, geometry and dynamic properties are unchanged. Removed 11 obsolete internal SRDF virtual joints and five collision exclusions referring to nonexistent pruner links. All valid collision exclusions are preserved.
8. **BrainCo description:** kept the meshes and fixed-joint Xacro; normalized XML formatting; removed the stale revolute-joint generated URDF, SolidWorks CSV, unused joint-name config, and ROS1 launch files; trimmed installation rules and dependencies.
9. **MoveIt configuration:** retained the active SRDF, limits, kinematics, controller mappings, OMPL settings, sensor settings, RViz config, and setup assistant. Removed unused CHOMP/LERP/Pilz/generated controller files and standalone boilerplate launches; corrected the retained setup-assistant launch and removed duplicate/stale metadata.
10. **Hand control:** preserved `hand_control.py` byte-for-byte. Reduced the helper to the existing connection procedure and direct SDK import, removed the example suite/custom logger/cache/output, added `std_msgs`, and made a missing SDK produce a useful import error. Replaced the protocol assertion with an explicit exception so it also works under optimized Python. Standard-library logging replaces the example logging framework; no log folder is created at import.

## Extra sweep fixes

- Cleaned the simulation launch's live references to the deleted interface/table nodes as well as its unused imports and commented experiments.
- Fixed `activate_joint_controller:=true` when the requested controller is already active; it previously duplicated the active entry and tried to remove an absent inactive entry. Choosing another controller now replaces the default trajectory controller for that arm, leaving the other arm's selection alone. The default path is unchanged.
- Fixed the real launch's fake-hardware branch to remove the two prefixed TCP broadcasters rather than the nonexistent unprefixed name. This addresses the Python list error; real ROS activation of this optional branch still needs testing.
- Removed duplicate passthrough-controller declarations and commented legacy controller YAML blocks while proving parsed values identical. This also removes obsolete `$(var tf_prefix)` text that launch-time parameter substitution could otherwise process even in YAML comments.
- Fixed the optional Ignition Xacro branch's undeclared `tf_prefix` reference to use the existing shared `controller_manager` name.
- Replaced the hardcoded `/root/workspace/install/...` Gazebo resource export with a package-relative path.
- Trimmed stale first-party dependencies, added missing launch/controller/model dependencies, and checked that the retained package graph has no cycles.

## Deliberate preservation

The prior plan called the sensor configuration unused. It is **retained unchanged**: `MoveItConfigsBuilder.to_moveit_configs()` automatically loads it, and removing it would alter planning-scene behavior. This was verified against the [official Humble builder implementation](https://github.com/moveit/moveit2/blob/humble/moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py).

The Cartesian support/example package and remaining upstream UR driver documentation, tests, resources, and source are retained unchanged. Existing license/author placeholders are preserved where authorship or licensing could not be inferred.

The expected behavior changes from removing the legacy nodes are that the old interface no longer applies 18 mm link padding, and simulation no longer inserts its old table. No replacement scene geometry or padding was invented.

## Validation performed

- All retained Python files parse; all XML/Xacro/SRDF/URDF files parse; all YAML/RViz/setup-assistant files load.
- The cleaned first-party Python entry points and launch files pass Ruff's undefined-name/unused-import checks.
- **36 offline regression tests pass.** These cover original/default real and simulation controller lists, all 12 advertised controller choices with fake hardware both enabled and disabled, launch-default preservation, execution/group argument wiring, protected file parity, unchanged C++ planning and worker methods, connection-helper behavior using a fake SDK, optimized-Python protocol rejection, missing-SDK diagnostics, and vendor-file preservation, and duplicate YAML key rejection.
- Expanded original and cleaned Xacro using Xacro 2.1.1 and the same official Humble `ur_description` checkout. In both real and fake hardware modes, each model has **64 links / 63 joints**, and the expanded XML matches apart from the two intended fixed-joint name changes. There are still exactly **12 movable arm joints**; every hand joint is fixed.
- All expanded mesh paths resolve. Every SRDF link and joint reference resolves. All **313 valid collision exclusions** and the group definitions, named states, and end-effector definitions match the original.
- Both Gazebo and Ignition Xacro branches expand to 65 links. The Gazebo expansion matches the original after normalizing workspace paths and the intended fixed-joint names. This is an XML expansion check, not a running simulator test.
- The corrected elbow limits, kinematics, MoveIt controller mappings, OMPL parameters, sensor configuration, initial arm positions, ROS hand node, TCP publisher, and hardware-control macro remain byte-for-byte unchanged. Real and simulation controller YAML values are identical to the original.
- All **14 package manifests** parse and the declared in-archive dependency graph is acyclic. Retired-file scans found no live first-party references to deleted resources.
- The final ZIP is checked for corruption and compared with the cleaned working tree. The original ZIP remains unchanged; its hash is recorded in the manifest.

## Runtime validation still required

This cleanup was performed on macOS without ROS 2, MoveIt C++ libraries, or the robot connected. A full `colcon` build, real ROS launch, controller activation, IK behavior, robot movement, and physical hand operation have **not** been tested here. Follow the fresh-workspace and planning-only checks in the README before enabling execution.

The offline controller tests execute the controller-selection logic extracted from the launch files. They do not emulate a ROS controller manager. The Modbus tests use a fake SDK and do not communicate with a hand. Xacro package lookup was mapped to the extracted packages and the downloaded official description dependency; no generated robot models or validation dependencies are included in the cleaned source archive.

## Existing architecture kept for a separate change

- The commander retains detached per-goal worker threads as requested. Startup goal handling, orderly worker shutdown, queue bounding, and coordination of simultaneous goals across the two arms would require a separate behavioral change. Wait for `MoveGroupInterfaces initialized` before sending goals and validate one arm at a time.
- Hardware bringup and the standalone commander still construct their descriptions separately. Any future change to calibration, UR type or prefixes must be propagated consistently. The existing single calibration-file argument and arm/connection naming conventions were preserved.
- `duo_ur` combines robot description and bringup, while the MoveIt package depends on that description. Adding a reverse package dependency would create a cycle; build/install the complete archive together as documented. Separating a dedicated bringup package is a future packaging change.
- The removed custom action/message package had no surviving consumer inside this archive. External applications using those retired interfaces would need migration; the current pose and hand topic APIs are unchanged.
