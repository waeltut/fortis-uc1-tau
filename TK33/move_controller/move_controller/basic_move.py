import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool, Int16
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fortis_interfaces.msg import Command as cmd, Abort as abt, Proximity as prx, Vision as vsn, CommandResponse as cmd_res
from fortis_interfaces.action import MoveCommand
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
# from fortis_interfaces.msg import MotionMsg as mc
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin, DriveOnHeading, NavigateToPose
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy as np
import time
import os
import sounddevice as sd
import wave
from enum import Enum


"""
class BasicMove(Node):
    def __init__(self):
        super().__init__("basic_move")

        # Memory
        self.commands = {
            "steven": [self.turn, False],
            "stephen": [self.turn, False],
            "turn": [self.turn, False],
            "spin": [self.spin, False],
            "follow": [self.follow, False],
            "forward": [self.linear, False],
            "back": [self.linear, False],
            "go": [self.go, False],
        }

        # Internal variables
        self.movement_status = 0
        self.goal_angle = 0.0
        self.default_distance = 2.0
        self.vision_decay = 0.0
        self.max_speed = 0.0
        self.angle_scale = 0.0

        # Odometry
        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_yaw = 0.0

        # Perception
        self.human_angle = 0.0
        self.human_distance = -1.0
        self.human_visible = False
        self.human_last_seen = 10.0
        self.obstacle_angle = 0.0
        self.obstacle_distance = -1.0

        # Flags
        self.in_progress = False
        self.stop = False

        # Action variables
        self.target_angle = 0.0

        # Other variables
        self.stop_time = self.get_clock().now()
        self.stop_source = "idle"

        # Start ros
        self.abort_pub = self.create_publisher(abt, "/steven/movement/abort_values", 1)
        self.move_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.progress_pub = self.create_publisher(Bool, "/steven/movement/move_in_progress", 1)
        self.work_pub = self.create_publisher(Bool, "/steven/movement/work_complete", 1)
        #self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.create_subscription(String, "/steven/movement/stop_command", self.handle_stop, 1)
        self.create_subscription(cmd, "/steven/movement/movement_command", self.handle_command, 1)
        self.create_subscription(vsn, "/steven/sensing/vision", self.vision_callback, 1)
        self.create_subscription(prx, "/steven/sensing/proximity", self.proximity_callback, 1)
        self.create_subscription(Odometry, "/odom", self.odometry_callback, 1)
        self.get_logger().info("Node started, waiting for commands...")
        #self.client.wait_for_server()
        self.create_timer(0.10, self.spinner)

    
    ## Callbacks ##
    def handle_stop(self, msg):
        if not rclpy.ok():
            return
        self.get_logger().info(f"Stop received for {msg.data}")
        self.stop_source = msg.data
        self.stop = True
        self.stop_time = self.get_clock().now()

    def handle_command(self, msg):
        if not rclpy.ok():
            return
        
        if not self.in_progress:
            # Setup for movement
            command = msg.command
            args = msg.args
            source = msg.source

            # Check if duplicate command (depracated)
            #if self.stop and source == self.stop_source:
            #    return
            
            # Continue with execution
            self.stop = False
            self.get_logger().info(f"Movement command received: {command}")

            # Start movement
            if command in self.commands:
                self.in_progress = True
                self.init_move = True
                self.get_logger().info(f"Initiating {command}...")
                self.start_time = self.get_clock().now()
                if len(args) > 0:
                    self.command_timer =  self.create_timer(0.1, lambda: self.commands[command][0](args))
                else:
                    self.command_timer = self.create_timer(0.1, self.commands[command][0])
            else:
                self.get_logger().warn(f"Movement command not recognized: {command}")
    
    def vision_callback(self, msg):
        if not rclpy.ok():
            return
        self.human_angle = msg.angle
        self.human_distance = msg.distance
        self.human_visible = msg.visible
        self.human_last_seen = msg.last_seen
    
    def proximity_callback(self, msg):
        if not rclpy.ok():
            return
        self.obstacle_angle = msg.angle
        self.obstacle_distance = msg.distance
    
    def odometry_callback(self, msg):
        if not rclpy.ok():
            return
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.odo_yaw = math.atan2(2*w*z, 1-2*z**2)
    

    def spinner(self):
        if not rclpy.ok():
            return
        msg = Bool()
        msg.data = True if self.in_progress else False
        self.progress_pub.publish(msg)
    

    def stop_movement(self):
        # Stop the robot
        stop_msg = Twist()
        self.move_pub.publish(stop_msg)
        if self.command_timer:
            self.command_timer.cancel()
            self.command_timer = None
        self.in_progress = False
        self.stop = False


    ## MOVEMENT ##
    def spin(self, args=[1, math.pi]):
        if not rclpy.ok():
            self.stop_movement()
            return

        # Initial target angle
        if self.init_move:
            angle_offset = args[0]*args[1]
            self.target_angle = self.odo_yaw + angle_offset
            if self.target_angle < -math.pi:
                self.target_angle = 2*math.pi - args[0]*self.target_angle
            elif self.target_angle > math.pi:
                self.target_angle = args[0]*self.target_angle - 2*math.pi
            self.init_move = False
        
        # Manual stop
        if self.stop:
            self.get_logger().warn("Stopping spin...")
            self.stop_movement()
            return
            
        # Timeout
        if self.get_clock().now() - self.start_time > Duration(seconds=10.0):
            self.get_logger().warn("Spin timeout...")
            self.stop_movement()
            return

        # Are we done
        angle_diff = (self.target_angle-self.odo_yaw + np.pi) % (2*np.pi) - np.pi
        if abs(angle_diff) < 0.2*math.pi/180:
            self.get_logger().info("Spin finished")
            self.stop_movement()
            return
            
        # Movement command
        msg = Twist()
        if angle_diff > np.pi/4:
            msg.angular.z = args[0] * 0.5
        else:
            msg.angular.z = args[0] * min(abs(angle_diff), 0.5)
        self.move_pub.publish(msg)
        
        
    def turn(self, args=0):
        if not rclpy.ok():
            self.stop_movement()
            return

        # Initialization
        if self.init_move:
            self.init_move = False
        
        # Manual stop
        if self.stop:
            self.get_logger().warn("Stopping turn...")
            self.stop_movement()
            return
        
        # Timeout (not mandatory)
        #if self.get_clock().now() - self.start_time > Duration(seconds=20.0):
        #    self.get_logger().warn("Turn timeout...")
        #    self.stop_movement()
        #    return

        # Are we done
        if abs(self.human_angle) <= math.pi / 180:
            self.get_logger().info("Turn finished")
            self.stop_movement()
            return

        # Determine angular velocity
        if not self.human_visible:
            # Blind rotation if human not visible
            angular_z = 0.5 if self.human_angle > 0 else -0.5
        else:
            # Proportional rotation based on angle
            angular_z = (self.human_angle / (math.pi/3)) ** 2
            angular_z = angular_z if self.human_angle > 0 else -angular_z
        
        # Publish rotation command
        message = Twist()
        message.angular.z = angular_z
        self.move_pub.publish(message)


    def follow(self, args=0):
        if not rclpy.ok():
            self.stop_movement()
            return

        # Initialization
        if self.init_move:
            self.vision_decay = 2.0      # Seconds to keep moving after losing sight
            self.max_speed = 0.4         # Maximum forward speed (m/s)
            self.angle_scale = 60.0      # Scale factor for angular velocity
            self.init_move = False
        
        # Manual stop
        if self.stop:
            self.get_logger().warn("Stopping spin...")
            self.stop_movement()
            return
        
        # Timeout (not mandatory)
        # if self.get_clock().now() - self.start_time > Duration(seconds=20.0):
        #    self.get_logger().warn("Turn timeout...")
        #    self.stop_movement()
        #    return


        # Gather vision data
        distance_mm = self.human_distance * 1000
        human_visible = self.human_visible
        last_seen = self.human_last_seen
        angle_deg = 180 * self.human_angle / math.pi

        # Stop if human close and visible
        if distance_mm <= 700 and human_visible:
            self.get_logger().info("Follow finished")
            self.stop_movement()
            return
            
        # Linear speed proportional to distance, decays if human not seen recently
        linear_speed = min(distance_mm / 1000.0 / 4.0, self.max_speed)
        decay_factor = max(0.0, (self.vision_decay - last_seen) / self.vision_decay)
        linear_speed *= decay_factor

        # Angular speed proportional to squared angle for sharper turns at larger angles
        angular_speed = (angle_deg / self.angle_scale) ** 2
        angular_speed = angular_speed if angle_deg > 0 else -angular_speed

        # Publish movement command
        message = Twist()
        message.linear.x = linear_speed
        message.angular.z = angular_speed
        self.move_pub.publish(message)
    

    def linear(self, args=[0.2, 0.1]):
        #if not rclpy.ok():
        #    self.stop_movement()
        #    return

        # Initialization
        if self.init_move:
            # Calculate duration needed for distance args[0]
            linear_x = args[1] if args[0] >= 0 else -args[1]
            s = args[0]
            self.duration = Duration(seconds=abs(s / linear_x))
            self.start_time = self.get_clock().now()
            self.init_move = False
        
        # Manual stop
        if self.stop:
            self.get_logger().warn("Stopping linear...")
            self.stop_movement()
            return

        # Timeout
        if self.get_clock().now() - self.start_time >= self.duration:
            self.get_logger().info("Linear finished")
            self.stop_movement()
            return

        # Movement command
        message = Twist()
        message.linear.x = args[1] if args[0] >= 0 else -args[1]
        self.move_pub.publish(message)


    def go(self, args=[0.042, -1.921, 0.980, -0.198]):
        if not rclpy.ok():
            self.stop_movement()
            return
        
        self.get_logger().warn('Command "Go" under development. Stopping...')

        # Initialization
        if self.init_move:
            self.init_move = False

        # Stopping
        self.get_logger().warn("Stopping Go...")
        self.stop_movement()
        return

        # Go to a specified location
        # goal = MoveBaseGoal()
        #goal.target_pose.header.frame_id = "map"
        #goal.target_pose.header.stamp = rospy.Time.now()
        #goal.target_pose.pose = Pose(
        #    Point(args[0], args[1], 0),
        #    Quaternion(0, 0, args[2], args[3])
        #)
        #self.client.send_goal(goal)
        
        # Loop to allow dynamic stopping
        #rate = rospy.Rate(10)
        #while not rospy.is_shutdown():
        #    if self.stop:
        #        self.client.cancel_goal()
        #        break
            
        #    state = self.client.get_state()
        #    #rospy.logwarn(f"Current state of move_base: {state}")
        #    if state in [3, 4, 5]:    # Succeeded, aborted, rejected
        #        break

        #    rate.sleep()

        # Was work reached
        #result = self.client.get_result()
        #if result and not self.stop:
        #    goal_msg = Bool()
        #    goal_msg.data = True
        #    self.work_pub.publish(goal_msg)
"""



# TODO: Consider turning this entire thing into action-based (not pub-sub)
# TODO: Would not work, as queueuye
class BasicMoveActions(Node):
    def __init__(self):
        super().__init__("basic_move")

        # Constants
        self.x0 = 12.90
        self.y0 = 35.89
        self.step = 0.075
        self.r = 0.75
        self.timeout = 2.0
        self.allowance = 10.0
        slam_path = get_package_share_directory("slam_config")
        self.follow_bt = os.path.join(slam_path, "behaviors", "follow_bt.xml")
        self.device_id = 1
        self.sample_rate = 48000

        # Actions
        self.actions = {"spin": self.spin, "linear": self.linear, "follow": self.follow, "go": self.go, "speak": self.speak, "stop": self.cancel_action}
        self.action_handle = [None, None]
        self.action_buffer = []
        self.current_id = 0

        # Flags
        self.action_cancel = False
        self.action_done = True
        self.action_ongoing = False

        # Testing
        self.cosmic_hyper_time = 25
        self.real_time = 0
        self.stop_thing = False
        self.first = True
        self.theta = 0

        # Quality of service -options
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_update", 10)
        self.received_pub = self.create_publisher(cmd_res, "/move_controller/basic/received", qos)
        self.process_pub = self.create_publisher(cmd_res, "/move_controller/basic/processing", qos)
        self.done_pub = self.create_publisher(cmd_res, "/move_controller/basic/done", qos)

        # Subscribers
        self.action_sub = self.create_subscription(cmd, "/move_controller/coordinator/action", self.command_callback, qos)

        # Clients
        self.spin_client = ActionClient(self, Spin, "spin")
        self.linear_client = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self.follow_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.go_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Timer
        self.create_timer(0.2, self.spinner)


    ## ACTIONS ##
    # Spin
    def spin(self, args):
        if not self.spin_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 Spin action server unavailable")
            self.unavailable_cancel()
            return

        # Define action
        goal = Spin.Goal()
        goal.target_yaw = args[0]   # 1.5708
        goal.time_allowance = Duration(seconds=self.allowance).to_msg()
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.goal_callback)

    # Linear
    def linear(self, args):
        if not self.linear_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 DriveOnHeading action server unavailable")
            self.unavailable_cancel()
            return

        # Define action
        goal = DriveOnHeading.Goal()
        target = Point(x=args[0], y=0.0, z=0.0)
        goal.target = target
        goal.speed = args[1]
        goal.time_allowance = Duration(seconds=self.allowance).to_msg()
        future = self.linear_client.send_goal_async(goal)
        future.add_done_callback(self.goal_callback)

    # Follow
    def follow(self, args):
        if not self.follow_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 NavigateToPose action server unavailable")
            self.unavailable_cancel()
            return

        # Define action
        goal = NavigateToPose.Goal()
        goal.pose = args
        goal.behavior_tree = self.follow_bt
        future = self.follow_client.send_goal_async(goal)
        future.add_done_callback(self.goal_callback)

    # Go
    def go(self, args):
        if not self.go_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 NavigateToPose action server unavailable")
            self.unavailable_cancel()
            return

        # Define action
        goal = NavigateToPose.Goal()
        goal.pose = args
        future = self.go_client.send_goal_async(goal)
        future.add_done_callback(self.goal_callback)

    # Speaking
    def speak(self, args):
        try:
            # Play audio from args
            audio = np.asarray(args, dtype=np.float32)
            sd.play(audio, samplerate=self.sample_rate, device=self.device_id)
    
            # Is not an action, so manually complete it after playback starts
            msg = String()
            msg.data = "Accepted"
            self.process_pub.publish(msg)
            msg.data = "Succeeded"
            self.done_pub.publish(msg)
            self.action_ongoing = False
            self.get_logger().info("speak succeeded")

        except Exception as e:
            self.get_logger().error(f"Audio device {self.device_id} unavailable: {e}")
            self.unavailable_cancel()
            return


    ## COMMUNICATION ##
    def command_callback(self, msg):
        self.get_logger().info(f"Received: {msg.command} (id: {msg.id})")
        msg_out = cmd_res()
        msg_out.data = Received.RECEIVED
        msg_out.id = msg.id
        self.received_pub.publish(msg_out)
        
        # Stop handling
        if msg.command.lower() == "stop":
            self.cancel_action(msg)
        
        # Reading
        else:
            action = {"command": msg.command, "args": msg.args, "source": msg.source, "id": msg.id}
            self.action_buffer.append(action)


    ## GOAL AND CANCELLATION ##
    # Goal
    def goal_callback(self, feedback):
        name = self.action_handle[0]
        goal_handle = feedback.result()
        msg = cmd_res()
        msg.id = self.current_id
        if not goal_handle.accepted:
            self.get_logger().warn(f"{name} goal rejected")
            msg.data = Process.REJECTED
            self.process_pub.publish(msg)
            goal_handle = None
            self.action_handle = [None, None]
            return
        self.get_logger().info(f"{name} goal accepted")
        msg.data = Process.ACCEPTED
        self.process_pub.publish(msg)
        result_handle = goal_handle.get_result_async()
        result_handle.add_done_callback(lambda f: self.result_callback(f, name))
        self.action_handle = [name, goal_handle]

    # Results
    def result_callback(self, feedback, name):
        result = feedback.result()
        msg = cmd_res()
        msg.id = self.current_id
        if result.status == 4:
            self.get_logger().info(f"{name} succeeded")
            msg.data = Done.SUCCEEDED
        elif result.status == 5:
            self.get_logger().info(f"{name} cancelled")
            msg.data = Done.CANCELED
        elif result.status == 6:
            self.get_logger().warn(f"{name} aborted")
            msg.data = Done.ABORTED
        else:
            return
        self.done_pub.publish(msg)
        self.action_ongoing = False

    # Cancel
    def cancel_action(self, msg):
        id = int(msg.id)
        args = msg.args
        num = int(args[0])

        # If associated with an id, cancel actions with that id
        if id > 0:
            self.action_buffer = [action for action in self.action_buffer if action["id"] != id]

        # Cancel specified amount of actions (<=0 is all)
        elif num > 0:
            for i in range(num-1):
                if len(self.action_buffer) <= 0:
                    break
                else:
                    self.action_buffer.pop(0)
        else:
            self.action_buffer = [self.action_buffer[0]] if len(self.action_buffer) > 0 else []

        # Cancellation
        name = self.action_handle[0]
        handle = self.action_handle[1]
        if handle is None or not handle.accepted:
            self.get_logger().info("No active goal to cancel")
            return
        self.get_logger().info(f"Requesting {name} cancellation")
        cancel_future = handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    # Callback
    def cancel_response_callback(self, future):
        name = self.action_handle[0]
        response = future.result()
        if response.goals_canceling:
            self.get_logger().info(f"{name} cancellation accepted")
            self.action_handle = [None, None]
        else:
            self.get_logger().warn(f"{name} cancellation was not accepted")

    # Unavailability
    def unavailable_cancel(self):
        # If action cannot be performed, it calls this to cleanly exit the queue
        msg = cmd_res()
        msg.data = Process.UNAVAILABLE
        msg.id = self.current_id
        self.process_pub.publish(msg)
        self.action_ongoing = False


    ## HELPERS ##
    def make_goal(self, x, y, angle):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = math.cos(angle / 2)
        msg.pose.orientation.z = math.sin(angle / 2)
        return msg

    def update_goal(self, x, y, angle):
        msg = self.make_goal(x, y, angle)
        self.goal_pub.publish(msg)
        # self.get_logger().info(f'Updated goal published: ({x:.2f}, {y:.2f})')


    ## LAUNCHING ##
    def spinner(self):
        if len(self.action_buffer) > 0 and not self.action_ongoing and not self.action_done:
            # Clear finished action and set flag
            self.action_buffer.pop(0)  # if len(self.action_buffer) > 0 else 0
            self.action_handle = [None, None]
            self.current_id = 0
            self.action_done = True

        if len(self.action_buffer) > 0 and self.action_done:
            # Set flags
            self.action_ongoing = True
            self.action_done = False

            # Call the given action
            action = self.action_buffer[0]
            name = action["command"]
            args = action["args"]
            self.current_id = action["id"]
            self.action_handle[0] = name
            action_fun = self.actions[name]
            action_fun(args)

        elif len(self.action_buffer) == 0:
            # Failsafe for empty buffer
            self.action_done = True
            self.action_ongoing = False
            self.action_handle = [None, None]
            self.current_id = 0

    def goal_spinner(self):
        if self.first:
            self.theta = 0
            self.first = False

        # Stepping
        x = self.x0 + self.r*math.cos(self.theta)
        y = self.y0 + self.r*math.sin(self.theta)
        self.theta = self.theta + self.step
        angle = (self.theta + math.pi / 2) % (2 * math.pi)
        self.update_goal(x, y, angle)

        if self.real_time >= self.cosmic_hyper_time:
            self.cancel_action()
        self.real_time += 1




class BasicMoveActions2(Node):
    # All action-codes
    ACCEPTED = 0
    REJECTED = 1
    UNAVAILABLE = 2
    SUCCEEDED = 3
    CANCELED = 4
    ABORTED = 5
    UNKNOWN = 6


    def __init__(self):
        super().__init__("basic_move")

        # Constants
        self.x0 = 12.90
        self.y0 = 35.89
        self.step = 0.075
        self.r = 0.75
        self.timeout = 2.0
        self.allowance = 10.0
        slam_path = get_package_share_directory("slam_config")
        self.follow_bt = os.path.join(slam_path, "behaviors", "follow_bt.xml")
        self.recovery_bt = os.path.join(slam_path, "behaviors", "recovery_navigation.xml")
        self.device_id = 37
        self.sample_rate = 48000

        # Actions
        self.actions = {"spin": self.spin,
                        "linear": self.linear,
                        "follow": self.follow,
                        "inform_go": self.inform_go,
                        "go": self.go,
                        "speak": self.speak,
                        "stop": self.nav2_cancel_action}
        self.action_handle = [None, None]
        self.current_id = 0
        self.action_finished = True
        self.result_status = None

        # Action server
        self.cb_group = ReentrantCallbackGroup()    # Need to run this shit asynchronously
        self.action_server = ActionServer(
            self,
            MoveCommand,
            "basic_move",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # Queueue
        self.goal_handle_queue = []    # Needs to support selective deletions
        self.canceled_handles = []    # All handles need to be canceled, if removed
        self.current_goal_handle = None
        self.current_cancellation_handle = None
        self.timestep = 0.1

        # Quality of service -options
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_update", 10)

        # Clients
        self.spin_client = ActionClient(self, Spin, "spin")
        self.linear_client = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self.follow_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.go_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Timer
        self.create_timer(0.1, self.dequererer)


    ## ACTION SERVER CALLBACKS ##
    def goal_callback(self, goal_request):
        if goal_request.command not in self.actions:
            self.get_logger().warn(f"Unknown command: {goal_request.command}")
            return GoalResponse.REJECT
        self.get_logger().info(f"Accepted command: {goal_request.command} (id: {goal_request.id})")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, accepted_handle):
        if accepted_handle.request.command == "stop":
            self.current_cancellation_handle = accepted_handle
            accepted_handle.execute()
        else:
            self.goal_handle_queue.append(accepted_handle)

    def execute_callback(self, goal_handle):
        # Check if this is a canceled handle or a stop-command
        if goal_handle in self.canceled_handles:
            self.canceled_handles.remove(goal_handle)
            result_msg = MoveCommand.Result()
            result_msg.status = BasicMoveActions2.CANCELED
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()    # Don't ask me to explain this :)
            return result_msg
        elif goal_handle.request.command == "stop":
            self.nav2_cancel_action(goal_handle.request.args, goal_handle.request.id)
            result_msg = MoveCommand.Result()
            result_msg.status = BasicMoveActions2.SUCCEEDED
            goal_handle.succeed()
            self.current_cancellation_handle = None
            return result_msg
        
        # First, the correct function
        request = goal_handle.request
        name = request.command
        args = request.args
        self.current_id = request.id
        action_fun = self.actions[name]
        action_fun(args)

        # Keep checking for cancellation
        while not self.action_finished:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.result_status = BasicMoveActions2.CANCELED
                break

            # God I hate threading
            time.sleep(self.timestep)

        # If action was aborted, unavailable or rejected, then all that share id must die
        if self.result_status == self.ABORTED or self.result_status == self.UNAVAILABLE or self.result_status == self.REJECTED:
            id = self.current_id
            self.current_id = None
            self.nav2_cancel_action([0.0], id)

        # Final countdown (badadaadaa dadadadadaa)
        result_msg = MoveCommand.Result()
        result_msg.status = self.result_status
        self.current_goal_handle = None
        self.current_id = None
        return result_msg

    def cancel_callback(self, cancel_handle):
        # We need to check if this goal is currently being executed
        goal_uuid = bytes(cancel_handle.goal_id.uuid)    # Thank you mr. GPT
        executing_uuid = bytes(self.current_goal_handle.goal_id.uuid)
        if goal_uuid == executing_uuid:
            id = self.current_goal_handle.request.id
            self.nav2_cancel_action([0.0], id)
            return CancelResponse.ACCEPT

        # Not, so it has to be in the queue
        for goal_handle in self.goal_handle_queue:
            queued_uuid = bytes(goal_handle.goal_id.uuid)
            if queued_uuid == goal_uuid:
                id = goal_handle.request.id
                break
        self.nav2_cancel_action([0.0], id)
        return CancelResponse.ACCEPT


    ## ACTION SERVER DEQUEUEUEING ##
    def dequererer(self):
        if self.current_goal_handle is not None or not self.goal_handle_queue:
            return
        self.action_finished = False
        self.current_goal_handle = self.goal_handle_queue.pop(0)
        self.current_goal_handle.execute()


    ## NAV2 GOAL AND CANCELLATION ##
    # Goal
    def nav2_goal_callback(self, future):
        # Get variables
        request = self.current_goal_handle.request
        name = request.command
        goal_handle = future.result()
        feedback_msg = MoveCommand.Feedback()

        # If nav2 did not accept goal
        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected {name} (id: {self.current_id})")
            self.result_status = BasicMoveActions2.REJECTED
            self.current_goal_handle.abort()
            self.action_finished = True
            return

        # Otwerhise
        self.get_logger().info(f"Nav2 accepted {name} (id: {self.current_id})")
        feedback_msg.status = BasicMoveActions2.ACCEPTED
        self.current_goal_handle.publish_feedback(feedback_msg)

        # Continue action-bananza
        result_handle = goal_handle.get_result_async()
        result_handle.add_done_callback(lambda f: self.nav2_result_callback(f, name))
        self.action_handle = [name, goal_handle]

    # Results
    def nav2_result_callback(self, feedback, name):
        result = feedback.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{name} succeeded (id: {self.current_id})")
            self.result_status = BasicMoveActions2.SUCCEEDED
            self.current_goal_handle.succeed()
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"{name} cancelled (id: {self.current_id})")
            self.result_status= BasicMoveActions2.CANCELED
            self.current_goal_handle.abort()    # Technically, because the client was different
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"{name} aborted (id: {self.current_id})")
            self.result_status = BasicMoveActions2.ABORTED
            self.current_goal_handle.abort()
        else:
            self.get_logger().warn(f"I'mma be real with you: I have no clue what happened to {name} (id: {self.current_id})")
            self.result_status = BasicMoveActions2.UNKNOWN
            self.current_goal_handle.abort()
        self.action_finished = True

    # Cancel
    def nav2_cancel_action(self, args, id):
        num = int(args[0])

        # If associated with an id, cancel actions with that id
        if id > 0:
            new_queue = []
            for goal_handle in self.goal_handle_queue:
                if goal_handle.request.id != id:
                    new_queue.append(goal_handle)
                else:
                    self.canceled_handles.append(goal_handle)
                    goal_handle.execute()
            self.goal_handle_queue = new_queue

            # If the current id is not the same as the "stop" -id, don't stop current action
            if id != self.current_id:
                return

        # Cancel specified amount of actions (<0 is all, ==0 is just the ongoing action)
        elif num > 0:
            for i in range(num):
                if len(self.goal_handle_queue) <= 0:
                    break
                else:
                    goal_handle = self.goal_handle_queue.pop(0)
                    self.canceled_handles.append(goal_handle)
                    goal_handle.execute()
        elif num < 0 and len(self.goal_handle_queue) > 0:
            for goal_handle in self.goal_handle_queue:
                self.canceled_handles.append(goal_handle)
                goal_handle.execute()
            self.goal_handle_queue = []

        # Cancellation
        name = self.action_handle[0]
        handle = self.action_handle[1]
        if handle is None or not handle.accepted:
            self.get_logger().info("No active goal to cancel")
            self.current_cancellation_handle.abort()
            return
        self.get_logger().info(f"Requesting {name} cancellation (id: {self.current_id})")
        cancel_future = handle.cancel_goal_async()
        cancel_future.add_done_callback(self.nav2_cancel_response_callback)

    # Callback
    def nav2_cancel_response_callback(self, future):
        name = self.action_handle[0]
        response = future.result()
        if response.goals_canceling:
            self.get_logger().info(f"{name} cancellation accepted (id: {self.current_id})")
            self.action_handle = [None, None]
            self.current_cancellation_handle.succeed() if self.current_cancellation_handle is not None else None
        else:
            self.get_logger().warn(f"{name} cancellation was not accepted (id: {self.current_id})")
            self.current_cancellation_handle.abort() if self.current_cancellation_handle is not None else None

    # Unavailability
    def nav2_unavailable_cancel(self):
        # If action cannot be performed, it calls this to cleanly exit the queue
        self.current_goal_handle.abort()
        self.result_status = BasicMoveActions2.UNAVAILABLE
        self.action_finished = True


    ## ACTIONS ##
    # Spin
    def spin(self, args):
        if not self.spin_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error(f"Nav2 Spin unavailable (id: {self.current_id})")
            self.nav2_unavailable_cancel()
            return

        # Define action
        goal = Spin.Goal()
        goal.target_yaw = args[0]   # 1.5708
        goal.time_allowance = Duration(seconds=self.allowance).to_msg()
        future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_callback)

    # Linear
    def linear(self, args):
        if not self.linear_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 DriveOnHeading action server unavailable")
            self.nav2_unavailable_cancel()
            return

        # Define action
        goal = DriveOnHeading.Goal()
        target = Point(x=args[0], y=0.0, z=0.0)
        goal.target = target
        goal.speed = args[1]
        goal.time_allowance = Duration(seconds=self.allowance).to_msg()
        future = self.linear_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_callback)

    # Follow
    def follow(self, args):
        if not self.follow_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 NavigateToPose action server unavailable")
            self.nav2_unavailable_cancel()
            return

        # Define action
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = args[0]
        pose.pose.position.y = args[1]
        pose.pose.orientation.z = args[2]
        pose.pose.orientation.w = args[3]
        goal.pose = pose
        goal.behavior_tree = self.follow_bt
        future = self.follow_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_callback)

    # Specifically used for informing about shit
    def inform_go(self, args):
        if not self.go_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 NavigateToPose action server unavailable")
            self.nav2_unavailable_cancel()
            return

        # Define action
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = args[0]
        pose.pose.position.y = args[1]
        pose.pose.orientation.z = args[2]
        pose.pose.orientation.w = args[3]
        goal.pose = pose
        goal.behavior_tree = self.recovery_bt
        future = self.follow_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_callback)

    # Go
    def go(self, args):
        if not self.go_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error("Nav2 NavigateToPose action server unavailable")
            self.nav2_unavailable_cancel()
            return

        # Define action
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = args[0]
        pose.pose.position.y = args[1]
        pose.pose.orientation.z = args[2]
        pose.pose.orientation.w = args[3]
        goal.pose = pose
        future = self.go_client.send_goal_async(goal)
        future.add_done_callback(self.nav2_goal_callback)

    # Speaking
    def speak(self, args):
        try:
            # Play audio from args
            audio = np.asarray(args, dtype=np.float32)
            sd.play(audio, samplerate=self.sample_rate, device=self.device_id)
    
            # Is not an action, so manually complete it after playback starts
            self.current_goal_handle.succeed()
            self.result_status = BasicMoveActions2.SUCCEEDED
            self.action_finished = True
            self.get_logger().info(f"speak succeeded (id: {self.current_id})")

        except Exception as e:
            self.get_logger().error(f"Audio device {self.device_id} unavailable: {e}")
            self.nav2_unavailable_cancel()
            return


    ## HELPERS ##
    def make_goal(self, x, y, angle):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = math.cos(angle / 2)
        msg.pose.orientation.z = math.sin(angle / 2)
        return msg

    def update_goal(self, x, y, angle):
        msg = self.make_goal(x, y, angle)
        self.goal_pub.publish(msg)
        # self.get_logger().info(f'Updated goal published: ({x:.2f}, {y:.2f})')




## ENUMERATION ##
class Received:
    RECEIVED = 0

class Process:
    ACCEPTED = 0
    REJECTED = 1
    UNAVAILABLE = 2

class Done:
    SUCCEEDED = 0
    CANCELED = 1
    ABORTED = 2


def main(args=None):
    rclpy.init(args=args)
    basic_move = BasicMoveActions2()
    executor = MultiThreadedExecutor()
    executor.add_node(basic_move)

    try:
        executor.spin()
    except KeyboardInterrupt:
        basic_move.get_logger().warn("Shutdown called")
    #except Exception as e:
        #basic_move.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        executor.shutdown()
        basic_move.destroy_node()


if __name__ == '__main__':
    main()