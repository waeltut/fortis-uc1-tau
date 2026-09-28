$ ros2 launch dual_arm_driver dual_arm_driver.launch.py

- Launches hand control and arm control
- Hand control using custom nodes
- Arm control using moveit referenced to world frame, see examples

**Move arms to default positions:**

ros2 topic pub --once /left_arm/goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'mur'},
  pose: {
    position: {
      x: 0.7,
      y: 0.25,
      z: 1.0
    },
    orientation: {
      x: 0,
      y: 0,
      z: 0,
      w: 1  
    }
  }
}"


ros2 topic pub --once /right_arm/goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'mur'},
  pose: {
    position: {
      x: 0.7,
      y: -0.25,
      z: 1.0
    },
    orientation: {
      x: -0.5,
      y: 0,
      z: 0,
      w: 1.0
    }
  }
}"



**Example hand_control:**

Open:
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray "{data: [0, 0, 0, 0, 0, 0]}"

Closed:
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray "{data: [1, 1, 1, 1, 1, 1]}"

Nice gesture:
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray "{data: [0.9, 0, 1.0, 0.0, 1.0, 1.0]}"