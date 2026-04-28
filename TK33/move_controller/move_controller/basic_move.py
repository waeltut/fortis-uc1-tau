import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fortis_interfaces.msg import Command as cmd
from fortis_interfaces.msg import Abort as abt
from fortis_interfaces.msg import Proximity as prx
from fortis_interfaces.msg import Vision as vsn
from nav_msgs.msg import Odometry
import math
import numpy as np
import time

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
        #self.move_pub.publish(msg)
        
        
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
        """
        if self.get_clock().now() - self.start_time > Duration(seconds=20.0):
            self.get_logger().warn("Turn timeout...")
            self.stop_movement()
            return
        """

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
        #self.move_pub.publish(message)


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
        """
        if self.get_clock().now() - self.start_time > Duration(seconds=20.0):
            self.get_logger().warn("Turn timeout...")
            self.stop_movement()
            return
        """

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
        #self.move_pub.publish(message)
    

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
        #self.move_pub.publish(message)


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

        """
        # Go to a specified location
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(args[0], args[1], 0),
            Quaternion(0, 0, args[2], args[3])
        )
        self.client.send_goal(goal)
        
        # Loop to allow dynamic stopping
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.stop:
                self.client.cancel_goal()
                break
            
            state = self.client.get_state()
            #rospy.logwarn(f"Current state of move_base: {state}")
            if state in [3, 4, 5]:    # Succeeded, aborted, rejected
                break

            rate.sleep()

        # Was work reached
        result = self.client.get_result()
        if result and not self.stop:
            goal_msg = Bool()
            goal_msg.data = True
            self.work_pub.publish(goal_msg)
        """



def main(args=None):
    rclpy.init(args=args)
    basic_move = BasicMove()
    try:
        rclpy.spin(basic_move)
    except KeyboardInterrupt:
        basic_move.get_logger().warn("Shutdown called")
    #except Exception as e:
        #basic_move.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        pass


if __name__ == '__main__':
    main()