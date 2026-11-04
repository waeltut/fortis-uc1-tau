# Installation and migration

## Replace the custom source packages

Stop the old bringup and any old commander first. Back up your current `src/UR5e`
outside the colcon source tree. Do not keep a second copy of these packages anywhere
under `src`, or colcon will discover duplicate package names.

Replace these directories with the versions from this archive:

- `dual_arm_driver`
- `duo_ur`

Add these directories:

- `dual_arm_interfaces`
- `dual_arm_commander`

Do not merely overlay the old `dual_arm_driver` directory: the old C++ source and
commander YAML were removed. The included `hand_control` and external UR driver
are unchanged; keep your existing versions if they match the uploaded source.
Replace the top-level README/examples and add `docs`.

## Build in ROS 2 Humble

Use a fresh terminal that has not sourced the old workspace overlay. Your shell
startup file must not automatically source a stale workspace for this step.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to dual_arm_driver
source install/setup.bash
ros2 interface show dual_arm_interfaces/action/MoveArms
```

For an existing isolated-install workspace, remove only the stale custom package
build/install directories before building if old executables remain installed:

```bash
cd ~/ros2_ws
rm -rf build/dual_arm_driver install/dual_arm_driver \
       build/dual_arm_commander install/dual_arm_commander \
       build/dual_arm_interfaces install/dual_arm_interfaces \
       build/duo_ur install/duo_ur
```

Run the build/source commands above again afterwards. For a workspace built with
`--merge-install`, use a fresh build/install location instead of these per-package
cleanup commands. CMake `install(PROGRAMS ...)` handles executable permissions;
manual chmod after building is not required.

## Launch and settings

The external launch entry point is unchanged:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py
```

Existing `mur_launch` can keep including that launch. Update its old commander
arguments or topic publishing clients as described below. Do not start the old
commander alongside this one.

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py \
  presets_file:=/home/fortis_dev/robot_config/arm_presets.yaml \
  commander_config:=/home/fortis_dev/robot_config/arm_commander.yaml
```

Both file paths must be absolute. Files are loaded at node startup; edit and restart.

| Launch argument | Default | Purpose |
| --- | --- | --- |
| `use_fake_hardware` | `false` | Fake hardware, no physical hand/payload nodes |
| `start_hands` | `true` | Start existing hand nodes on real hardware |
| `initialize_payload` | `true` | Apply payload configuration on real hardware |
| `reference_frame` | `mur` | Frame for published TCP poses only |
| `headless_mode` | `false` | Existing UR headless External Control option |
| `left_group` | `left_ur_manipulator` | Single left MoveIt group |
| `right_group` | `right_ur_manipulator` | Single right MoveIt group |
| `dual_group` | `both_ur_manipulator` | Combined MoveIt group |
| `presets_file` | Installed `dual_arm_commander/config/presets.yaml` | Named targets |
| `commander_config` | Installed `dual_arm_commander/config/commander.yaml` | Node settings |

Hardware arguments from `duo_ur_real.launch.py` remain available:

```bash
ros2 launch duo_ur duo_ur_real.launch.py --show-args
```

The supplied mapping is preserved: `robot_ip_1=192.168.1.102` is right, and
`robot_ip_2=192.168.1.103` is left. The default `reverse_ip=192.168.1.75` must be
reachable from the arms. Pendant labels do not override this mapping.

The internal `dual_arm_robot_description` launch configuration supplies the
expanded model to the commander; do not override it manually.

## Breaking API changes

| Old interface | Replacement |
| --- | --- |
| `/left_arm/goal_pose` topic | `/arms/move`, explicit, mode `left` |
| `/right_arm/goal_pose` topic | `/arms/move`, explicit, mode `right` |
| Two topics published close together | One action with mode `both` |
| `/dual_arm/go_home` service | Preset action `home` |
| `/dual_arm/home_status` | Action feedback/result |
| `/left_arm/planned_trajectory`, `/right_arm/planned_trajectory`, `/dual_arm/planned_trajectory` | `/arms/planned_trajectory` |
| `execute:=false` | Per-goal `plan_only: true` |
| `pairing_window_sec`, `max_queue_per_arm` | Removed; caller owns sequencing |
| `dual_arm_driver/config/commander.yaml` | `dual_arm_commander/config/commander.yaml` plus `presets.yaml` |
| `dual_arm_moveit_commander` node / C++ executable | `dual_arm_commander` node / `arm_commander` executable |

Existing `mur_test` code that publishes `goal_pose` must become an action client.
Navigation and base selection are not changed in this archive. The task coordinator
should wait for navigation success, send this action, then wait for its final
status before advancing.

Home no longer sends a hand command or requires hand subscribers. If a task needs
finger movement, add that explicitly in its external sequence using the existing
hand interface. There is no hand-completion guarantee in this arm action.

Removing collision exclusions changes the semantic robot description. If your
reachability cache fingerprints the SRDF/collision model, regenerate that cache
before relying on its results.
