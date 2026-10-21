# Dual UR5e platform

ROS 2 dual-arm hardware bringup and MoveIt control for the MiR-mounted robot.

## Packages

| Directory | Responsibility |
| --- | --- |
| `duo_ur` | Robot Xacro, hand meshes, SRDF, MoveIt and ros2_control configuration; real/fake bringup |
| `dual_arm_driver` | Main launch, C++ pose commander, Python TCP publisher and payload initializer |
| `hand_control` | Existing hand interface, unchanged |
| `Universal_Robots_ROS2_Driver` | Complete supplied driver repository, unchanged, including any existing local modifications |

The custom model and MoveIt configuration share one package. There is no separate hand-description package. The external single-robot Cartesian example and the unused Cartesian plugin configuration are omitted. MoveIt Cartesian pose goals still work through IK and trajectory controllers.

## Launch

The public entry point is:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py
```

This starts both arms, one controller manager, MoveIt, the queued pose commander, TCP publishers, payload initialization and both hand nodes. It does not send an arm goal simply by starting. With the default `execute:=true`, later pose goals execute automatically.

Fake hardware, without connecting hand nodes, payload initialization, dashboards or URScript interfaces to physical hardware:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py use_fake_hardware:=true launch_rviz:=true
```

Plan-only command handling:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py execute:=false
```

`execute:=false` controls the pose commander; it does not disable other clients' direct MoveIt/controller execution requests. It also does not disable the hand nodes or payload initialization. Use `start_hands:=false initialize_payload:=false` if those are not wanted.

Lower-level entry points remain available:

```bash
ros2 launch duo_ur duo_ur_real.launch.py
ros2 launch duo_ur duo_ur_sim.launch.py
ros2 launch duo_ur setup_assistant.launch.py
```

The real launch starts hardware and MoveIt without the application commander or hand nodes. The sim launch uses the same implementation with fake hardware and RViz. Fake hardware uses wall time, not `/clock`. These are alternative launches: do not launch multiple controller managers for the same robot.

## Network and arm identity

Existing model/IP associations are preserved. Do not infer arm identity from the pendant stickers.

| Setting | Robot 1 | Robot 2 |
| --- | --- | --- |
| Logical arm / TF prefix | right / `right_` | left / `left_` |
| IP argument | `robot_ip_1` | `robot_ip_2` |
| Default IP | `192.168.1.102` | `192.168.1.103` |
| Reverse port | 50001 | 50011 |
| Script sender port | 50002 | 50012 |
| Trajectory port | 50003 | 50013 |
| Script command port | 50004 | 50014 |

Default `reverse_ip` is `192.168.1.75`; change it to the robot-accessible address of the driver computer when necessary. Dashboard and URScript node names follow this table. TF prefixes are restricted to the configured values because controller joint names, SRDF and TCP names rely on them.

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py \
  robot_ip_1:=192.168.1.102 robot_ip_2:=192.168.1.103 \
  reverse_ip:=192.168.1.75
```

Real hardware activates the two scaled joint trajectory controllers. The controller stoppers are started only after successful controller spawning, and each exempts the opposite arm's controllers. Fake hardware instead activates ordinary trajectory controllers and changes MoveIt's controller mapping accordingly.

## Settings

| File | Settings |
| --- | --- |
| `dual_arm_driver/config/commander.yaml` | Velocity/acceleration scaling, planning time/attempts, dual IK search budget, home poses and finger command |
| `dual_arm_driver/config/payload.yaml` | Existing per-arm mass, centre of gravity, inertia and retry settings |
| `duo_ur/config/controllers.yaml` | Single ros2_control configuration; controller types, joints, tolerances and scaling interfaces |
| `duo_ur/config/moveit_controllers.yaml` | Real-hardware FollowJointTrajectory action mapping |
| `duo_ur/config/ompl_planning.yaml` | OMPL planner configuration and planning-group settings |
| `duo_ur/config/kinematics.yaml` | Per-arm KDL solvers; the combined group uses the two arm subgroups |
| `duo_ur/config/joint_limits.yaml` | MoveIt velocity/acceleration limit settings |
| `duo_ur/config/duo_ur_onehand.srdf` | Planning groups, named states and collision exclusions |
| `duo_ur/config/sensors_3d.yaml` | Existing point-cloud octomap configuration, `/camera/depth/points` |
| `duo_ur/config/left_initial_positions.yaml`, `right_initial_positions.yaml` | Fake-hardware initial joint values |
| `duo_ur/urdf/duo_ur_onehand.urdf.xacro` | Robot geometry, mounts, TCP offsets and model arguments |
| `duo_ur/urdf/brainco_hand.urdf.xacro` | Fixed hand geometry; mesh assets are in `duo_ur/meshes/brainco` |

The existing geometry, TCP transforms, collision exclusions, payload settings and C++ motion logic are preserved. The supplied kinematics file defaults to nominal UR parameters for both arms; this cleanup does not introduce measured calibration data. `kinematics_params_file` retains its original shared-file behaviour.

| Main launch argument | Default | Meaning |
| --- | --- | --- |
| `reference_frame` | `mur` | Frame used by TCP pose publishers; goals use their own `header.frame_id` |
| `execute` | `true` | Execute successful commander plans |
| `headless_mode` | `false` | UR headless External Control mode |
| `use_fake_hardware` | `false` | Simulated joint interfaces, no physical hand/payload nodes |
| `start_hands` | `true` | Start both existing hand nodes on real hardware |
| `initialize_payload` | `true` | Apply configured payloads on real hardware |
| `left_group` / `right_group` | `left_ur_manipulator` / `right_ur_manipulator` | Individual planning groups |
| `dual_group` | `both_ur_manipulator` | Coordinated planning group |
| `pairing_window_sec` | `0.15` | Time window for pairing left/right goals |
| `max_queue_per_arm` | `0` | Pending goal limit; zero means unlimited |
| `launch_rviz` | `false` | Start RViz |
| `launch_dashboard_client` | `false` | Start per-arm dashboard clients on real hardware |

Hardware arguments are declared by the included `duo_ur_real` launch and can be supplied to the main launch. See all declarations with `ros2 launch duo_ur duo_ur_real.launch.py --show-args`. The internal `dual_arm_robot_description` launch configuration carries the expanded hardware model to the commander; do not override it.

## Inputs and outputs

| Interface | Type | Purpose |
| --- | --- | --- |
| `/left_arm/goal_pose`, `/right_arm/goal_pose` | `geometry_msgs/msg/PoseStamped` | TCP targets; supply a valid TF frame and unit quaternion |
| `/left_arm/pose`, `/right_arm/pose` | `geometry_msgs/msg/PoseStamped` | Current TCP poses, 10 Hz |
| `/left_arm/planned_trajectory`, `/right_arm/planned_trajectory`, `/dual_arm/planned_trajectory` | `moveit_msgs/msg/RobotTrajectory` | Planned trajectories |
| `/dual_arm/go_home` | `std_srvs/srv/Trigger` | Request configured home behaviour |
| `/dual_arm/home_status` | `std_msgs/msg/String` | Home status |
| `/hand_L/finger_positions`, `/hand_R/finger_positions` | `std_msgs/msg/Float32MultiArray` | Existing hand command interfaces; also used by home behaviour |

Arm goals are queued; goals arriving within the pairing window can be planned together. The combined group has no direct KDL solver because it is a branched tree. The existing commander obtains individual arm IK solutions and plans the combined joint target.

## External references

The main launch package/name, `duo_ur` model path, ROS topics, services, node names (except corrected dashboard names), planning groups and frames remain available. Outside this ZIP, update any direct references as follows:

| Previous reference | Current reference |
| --- | --- |
| `duo_ur5e_torso_moveit_config` package share | `duo_ur` package share; same `config/*.yaml` and SRDF filenames |
| `ros2 run moveit_utils_pkg dual_arm_pose_commander` | `ros2 run dual_arm_driver dual_arm_pose_commander` |
| `BrainCoRightHandURDF` model | `duo_ur/urdf/brainco_hand.urdf.xacro` |
| `duo_ur/config/duo_ur_adv_controllers.yaml` or `duo_ur5e_controllers.yaml` | `duo_ur/config/controllers.yaml` |
| Old RViz files | `duo_ur/rviz/moveit.rviz` |

No compatibility packages or duplicate source configurations are included. An external launch that only includes `dual_arm_driver/launch/dual_arm_driver.launch.py` retains that entry point. External code that directly reads the retired MoveIt package must change its package lookup and package.xml dependency to `duo_ur`.

## Verification

Run `python3 tools/validate.py` from this folder for offline checks, and add `--ros` after building/sourcing the workspace for installed Xacro expansion, SRDF/link checks and mesh resolution. Neither command starts hardware or sends goals.

Completed for this ZIP: custom Python syntax, XML/YAML parsing, package names and dependency cycles, bundled mesh/include references, controller joint-list agreement and SRDF/solver configuration. All 241 files in the supplied Universal Robots repository and all 10 files in `hand_control` were verified byte-identical. The moved C++ commander source is also byte-identical.

Not run in the preparation environment: colcon compilation, installed Xacro expansion, ROS launch execution, MoveIt planning/execution and physical hardware tests. ROS 2 and its dependencies are not installed there. The included checks and fake-hardware launch are provided for those remaining checks in the robot container; static validation alone is not proof of hardware operation.
