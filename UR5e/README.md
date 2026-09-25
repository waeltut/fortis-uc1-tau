# Cleaned dual-arm UR5e platform

This archive contains the cleaned source packages from `UR5e.zip`. The original archive was not modified. See [CLEANUP_REPORT.md](CLEANUP_REPORT.md) for changes, validation results, and remaining runtime checks. [CLEANUP_MANIFEST.json](CLEANUP_MANIFEST.json) lists every removed, relocated, and modified file.

## Build in a fresh workspace

Use the same ROS 2 Humble, MoveIt, UR description/client library, Cartesian controller plugins, and BrainCo SDK versions as your working robot setup. This archive does not bundle all external dependencies. In particular, keep your working `bc_stark_sdk` installation and `/dev/hand_L` and `/dev/hand_R` device rules.

Extract into a fresh source workspace, rather than merging over the previous extracted directory. Merging leaves retired files behind. Do not source the old workspace's `install/setup.bash`; it can make removed packages and executables appear to remain available. Source any separately installed dependency workspace you already use.

For example, from the directory containing the ZIP:

```bash
mkdir -p ~/ur5e_clean_ws/src
unzip UR5e_cleaned.zip -d ~/ur5e_clean_ws/src
cd ~/ur5e_clean_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Resolve missing external packages using your existing working dependency setup before building. `bc_stark_sdk` is a separate Python SDK, not a ROS package supplied by this archive. Build the complete workspace: `duo_ur` contains both the description and bringup launches, while its MoveIt configuration is in the adjacent `duo_ur5e_torso_moveit_config` package.

## Real robot: begin with planning only

In a terminal with the cleaned workspace sourced:

```bash
ros2 launch duo_ur duo_ur_real.launch.py
```

Add `launch_rviz:=true` if wanted. The existing IPs, ports, controller mappings, TF prefixes, and real-time clock setting are unchanged.

In another sourced terminal:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py execute:=false
```

Check:

```bash
ros2 param get /dual_arm_moveit_commander execute
ros2 param list /dual_arm_moveit_commander
ros2 control list_controllers
ros2 topic echo /left_arm/pose --once
ros2 topic echo /right_arm/pose --once
```

The execution parameter should be `False`, and the commander should list kinematics parameters for both planning groups. Use your existing known target poses to plan one arm at a time. Inspect `/left_arm/planned_trajectory` and `/right_arm/planned_trajectory`, including wrist travel and clearance. The old interface's automatic 18 mm link padding and the simulation table publisher have been removed as agreed; account for any scene objects or padding that your application needs explicitly.

After checking the plans, stop the planning-only commander and relaunch it with:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py execute:=true
ros2 param get /dual_arm_moveit_commander execute
```

Verify `True`, then use a small, known-clearance movement for each arm. Execution is read at node startup; relaunch to change it. Start only one commander instance.

## Hands

Run each in its own sourced terminal:

```bash
ros2 run hand_control hand_control --ros-args -p hand:=L -r __node:=revo_hand_L
ros2 run hand_control hand_control --ros-args -p hand:=R -r __node:=revo_hand_R
```

The existing topics are `/hand_L/finger_positions` and `/hand_R/finger_positions`, using `std_msgs/msg/Float32MultiArray`. Each message has six normalized values in this order: thumb, thumb auxiliary, index, middle, ring, pinky. The ROS node and its normalization, clamping, command timing, ports, and finger mapping are unchanged.

## Fake-hardware simulation

```bash
ros2 launch duo_ur duo_ur_sim.launch.py
```

This is the existing fake-hardware/RViz path, not a Gazebo launcher. It preserves `use_sim_time=False` and the existing active trajectory controllers. It no longer starts the retired interface or table nodes. Run the dual-arm driver separately if required.

## Entry points and files

| Purpose | Package / file |
| --- | --- |
| Real bringup | `duo_ur/launch/duo_ur_real.launch.py` |
| Fake-hardware bringup | `duo_ur/launch/duo_ur_sim.launch.py` |
| Headless pose interface | `dual_arm_driver/launch/dual_arm_driver.launch.py` |
| C++ pose planner | `moveit_utils_pkg/src/dual_arm_pose_commander.cpp` |
| Real controllers | `duo_ur/config/duo_ur_adv_controllers.yaml` |
| Simulation controllers | `duo_ur/config/duo_ur5e_controllers.yaml` |
| Robot model | `duo_ur/urdf/duo_ur_onehand.urdf.xacro` |
| MoveIt settings | `duo_ur5e_torso_moveit_config/config/` |
| Hand model | `BrainCoRightHandURDF/urdf/BrainCoRightHandURDF.urdf.xacro` |

The controller YAML moved out of the vendor driver. If an external launch command explicitly sets `runtime_config_package:=ur_robot_driver` for this dual-arm launch, change it to `runtime_config_package:=duo_ur` or omit it. The default has already been updated. The vendor's duplicate `duo_ur_control.launch.py` has been retired; use `ros2 launch duo_ur duo_ur_real.launch.py`.

This configuration is still intended for the existing UR5e model and `left_`/`right_` prefixes. Changing prefixes, UR type, calibration, or robot geometry requires coordinated updates to the hardware, MoveIt model, SRDF, and controller configuration.
