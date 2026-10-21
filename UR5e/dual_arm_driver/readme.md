$ ros2 launch dual_arm_driver dual_arm_driver.launch.py

- Launches hand control and arm control
- Hand control using custom nodes
- Arm control using moveit referenced to world frame, see examples

**Move arms to home:**

ros2 service call /dual_arm/go_home std_srvs/srv/Trigger '{}'

**Examples**

ros2 topic pub --once /left_arm/goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'mur'},
  pose: {
    position: {
      x: 0.6,
      y: 0.15,
      z: 1.2
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
      x: 0.6,
      y: -0.15,
      z: 1.2
    },
    orientation: {
      x: 0,
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

**Compact closed fists both**
ros2 topic pub --once /hand_L/finger_positions std_msgs/msg/Float32MultiArray "{data: [0.9, 0, 1, 1, 1, 1]}"
ros2 topic pub --once /hand_R/finger_positions std_msgs/msg/Float32MultiArray "{data: [0.9, 0, 1, 1, 1, 1]}"