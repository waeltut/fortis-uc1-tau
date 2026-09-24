**Only ros2 node is hand_control.**

- It connects to /dev/hand_L or /dev/hand_R
- Each finger joint takes value from 0-1.0 (percentage closed)
- Right hand is default with $ ros2 run hand_control hand_control

**Run Node**
$ ros2 run hand_control hand_control --ros-args -p hand:=L
$ ros2 run hand_control hand_control --ros-args -p hand:=R



**Subscribes to**
data:
[thumb, thumb_aux, index, middle, ring, pinky]

**Close fist**

ros2 topic pub --once /hand_R/finger_positions \
std_msgs/msg/Float32MultiArray \
"{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"


**Open fist**

ros2 topic pub --once /hand_R/finger_positions \
std_msgs/msg/Float32MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"