# Dual-arm UR5e command examples

Commands for Bash in the robot's ROS container. Positions are in metres and orientations are quaternions in `x, y, z, w` order. The arm examples below preserve the coordinates and identity orientation from your original README.

## Start the system

In each new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Start the complete platform interface in one terminal:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py
```

Send the following commands from another terminal. With the default `execute:=true`, arm goals request real movement after successful planning.

For planning without commander execution, launch instead:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py execute:=false
```

For fake arms without physical hand nodes or payload initialization, launch instead:

```bash
ros2 launch dual_arm_driver dual_arm_driver.launch.py use_fake_hardware:=true launch_rviz:=true
```

Do not launch these alternatives together. The fake-hardware commands refer to the cleaned package set.

## Arm target coordinates

| Arm | Frame | X | Y | Z | Quaternion (x, y, z, w) |
| --- | --- | --- | --- | --- | --- | --- |
| Left | `mur` | 0.6 | 0.15 | 1.2 | (0, 0, 0, 1) |
| Right | `mur` | 0.6 | -0.15 | 1.2 | (0, 0, 0, 1) |

These are absolute TCP poses in `mur`, not offsets from the current TCP position. Identity orientation aligns the target TCP axes with `mur`; it does not preserve the current orientation. The controlled links are `left_tcp` and `right_tcp`.

### Left arm

```bash
ros2 topic pub --once /left_arm/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: mur}, pose: {position: {x: 0.6, y: 0.15, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

### Right arm

```bash
ros2 topic pub --once /right_arm/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: mur}, pose: {position: {x: 0.6, y: -0.15, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

### Send both arm targets

Publish from two background CLI processes so the commands can arrive close together:

```bash
ros2 topic pub --once /left_arm/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: mur}, pose: {position: {x: 0.6, y: 0.15, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' &
left_pub_pid=$!

ros2 topic pub --once /right_arm/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: mur}, pose: {position: {x: 0.6, y: -0.15, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' &
right_pub_pid=$!

wait "$left_pub_pid" "$right_pub_pid"
```

The commander pairs eligible left/right goals arriving within `pairing_window_sec` (default 0.15 s). Independent CLI processes have variable startup/discovery delays, so this does **not guarantee** one combined trajectory. `wait` waits for the publishers to exit, not for arm movement to finish. For these configured home positions, the home service below inserts a protected pair atomically and also commands the hands after successful arm execution.

## Home service

```bash
ros2 service call /dual_arm/go_home std_srvs/srv/Trigger '{}'
```

With the supplied configuration, this requests:

- Left TCP: `(0.6, 0.15, 1.2)` in `mur`.
- Right TCP: `(0.6, -0.15, 1.2)` in `mur`.
- Both TCP orientations: `(0, 0, 0, 1)`.
- After successful arm execution, both hands receive `[0.9, 0, 1, 1, 1, 1]`.

A service response of `success: true` means **queued**, not completed. Existing queued work may run first. Watch the result:

```bash
ros2 topic echo /dual_arm/home_status std_msgs/msg/String --qos-durability transient_local
```

The service requires subscribers on both hand command topics before accepting a request. It therefore normally rejects requests in fake-hardware mode or when the hand nodes are disabled. Subscriber presence does not prove a physical hand is connected. The final status confirms finger commands were published, not that the hands reached their positions.

With `execute:=false`, the commander does not execute the home trajectory or send the post-execution finger commands. The home configuration is in `dual_arm_driver/config/commander.yaml`; restart the launch after changing it. This service homes the arms/hands, not the MiR mobile base.

## Hand commands

Messages contain six normalized values, each from 0 to 1:

```text
[thumb, thumb_aux, index, middle, ring, pinky]
```

The node clamps values to this range and maps them to device positions 0–1000. `thumb_aux` controls the second thumb degree of freedom. Commands received while the device is disconnected are discarded; resend after its connection message appears.

### Open left hand

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Open right hand

```bash
ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Close left hand — original all-ones example

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}'
```

### Close right hand — original all-ones example

```bash
ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}'
```

### Compact closed fists — both hands

These retain your original compact-fist values:

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.9, 0.0, 1.0, 1.0, 1.0, 1.0]}'

ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.9, 0.0, 1.0, 1.0, 1.0, 1.0]}'
```

### Open both hands

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Original “nice gesture” example

```bash
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray \
  '{data: [0.9, 0.0, 1.0, 0.0, 1.0, 1.0]}'
```

Use `/hand_R/finger_positions` for the same command on the right hand.

### Run a hand node independently

Only use these when that hand node is not already started by the main launch. Run each in its own terminal:

```bash
ros2 run hand_control hand_control --ros-args -p hand:=L
```

```bash
ros2 run hand_control hand_control --ros-args -p hand:=R
```

The device paths are `/dev/hand_L` and `/dev/hand_R`. Hand control uses topics; the supplied hand node does not define an open/close service.

## Read current poses and inspect interfaces

Read one current pose for each arm:

```bash
ros2 topic echo /left_arm/pose geometry_msgs/msg/PoseStamped --once
ros2 topic echo /right_arm/pose geometry_msgs/msg/PoseStamped --once
```

Inspect the corresponding TF transforms (Ctrl+C to stop):

```bash
ros2 run tf2_ros tf2_echo mur left_tcp
```

```bash
ros2 run tf2_ros tf2_echo mur right_tcp
```

List controller states and available services:

```bash
ros2 control list_controllers -c /controller_manager
ros2 service list -t
ros2 service type /dual_arm/go_home
```

Check whether the hand command topics have subscribers:

```bash
ros2 topic info /hand_L/finger_positions
ros2 topic info /hand_R/finger_positions
```

Watch future combined plans (this does not indicate execution completion):

```bash
ros2 topic echo /dual_arm/planned_trajectory moveit_msgs/msg/RobotTrajectory
```

CLI publication confirms a message was published, not that planning succeeded or the robot reached its target. Check commander logs, current poses and the home status topic as applicable.
