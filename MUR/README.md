# MUR base placement

Select a mobile-base pose from which a TCP target is supported by a cached arm
workspace, then check navigation feasibility with Nav2. Selection sends no base
or arm movement commands.

| Component | Purpose |
| --- | --- |
| `mur_base_selector build_volume` | Offline IK and self/body collision checks; saves a resumable workspace cache |
| `mur_base_selector base_selector` | Online cache lookup, base-pose scoring and Nav2 path validation |
| `mur_base_selector volume_visualizer` | Workspace cubes and candidate-stage diagnostics for Lichtblick/RViz |
| `mur_reachability` | Native MoveIt engine used by the offline generator; no standalone online node |

## Build

Place both package directories under `~/ros2_ws/src/MUR/`. Keep your generated
`.npz` file and robot-specific YAML settings when copying source files.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select mur_reachability mur_base_selector --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The cache defaults to
`~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz`.
If it is already complete and matches the robot model/settings, use it directly.

## Run

With robot TF, joint states, current footprint, scan, MoveIt and Nav2 available:

```bash
ros2 launch mur_base_selector base_selector.launch.py \
  selector_config:=$HOME/ros2_ws/src/MUR/mur_base_selector/config/volume_selector.yaml \
  arm_mode:=either
```

Start visualisation in another sourced terminal:

```bash
ros2 launch mur_base_selector volume_visualizer.launch.py
```

Publish a target and inspect the output using the commands in the
[mur_base_selector guide](mur_base_selector/README.md).

## Documentation

- [Nodes, workflow, inputs/outputs and usage](mur_base_selector/README.md)
- [Selector parameter reference](mur_base_selector/docs/selector_parameters.md)
- [Cache generation and collision policy](mur_base_selector/docs/cache_generation.md)
- [Visualisation topics and live controls](mur_base_selector/docs/visualization.md)
- [Native engine](mur_reachability/README.md)

## Tests

```bash
PYTHONPATH=./mur_base_selector python3 -m unittest discover -s mur_base_selector/test
```

Run this from the directory containing the two packages. Python tests use message
and transport doubles where ROS is involved. Native collision tests require ROS:

```bash
cd ~/ros2_ws
colcon test --packages-select mur_reachability
colcon test-result --verbose
```
