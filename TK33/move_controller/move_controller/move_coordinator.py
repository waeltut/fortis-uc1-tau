import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_msgs.msg import Float64
from fortis_interfaces.msg import Command as cmd
from fortis_interfaces.msg import Hearing as hrn
from fortis_interfaces.msg import Vision as vsn
from fortis_interfaces.msg import Proximity as prx
from fortis_interfaces.msg import AggMotivation as agm
from nav_msgs.msg import Odometry
import math
import csv
import pygame

class Coordinator(Node):
    def __init__(self):
        # Initialize
        super().__init__("coordinator")
        self.keywords = []
        self.motivated = False
        self.move_in_progress = False
        self.previous_motivation = "idle"
        self.baseline = 50
        self.record = False

        # Odometry
        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_yaw = 0.0
        self.odo_yaw_base = 0.0
        self.odo_first = True

        # Motivations
        self.motivations = {}

        # Ros
        self.get_logger().info("Node started. Listening to various topics...")
        self.command_pub = self.create_publisher(cmd, "/steven/movement/movement_command", 1)
        self.stop_pub = self.create_publisher(String, "/steven/movement/stop_command", 1)
        self.reset_pub = self.create_publisher(Bool, "/steven/movement/dist_reset", 1)
        self.mode_pub = self.create_publisher(String, "/steven/movement/mot_mode", 1)
        self.create_subscription(hrn, "/steven/sensing/hearing", self.hearing_callback, 1)
        self.create_subscription(Odometry, "/odom", self.odometry_callback, 1)
        self.create_subscription(agm, "/steven/reasoning/agg_motivation", self.agg_mot_callback, 1)
        self.create_subscription(Bool, "/steven/movement/move_in_progress", self.move_callback, 1)
        #self.pkg_path = get_package_share_directory('sensing_controller')

        # Messages
        self.stop_msg = String()
        self.stop_msg.data = "Stop"
        self.cmd_msg = cmd()
        self.cmd_msg.header = Header()
        self.reset_msg = Bool()
        self.reset_msg.data = True
        self.mode_msg = String()

        # Audio
        #pygame.mixer.init()
        #pygame.mixer.music.load(self.pkg_path + "/audio/output.mp3")
        #pygame.mixer.music.play()

        # Recording subscribers
        if self.record:
            self.create_subscription("/steven/sensing/time_since_interaction", Float64, self.social_callback)
            self.create_subscription("/steven/sensing/vision", vsn, self.vision_callback)
            self.create_subscription("/steven/movement/work_complete", Bool, self.work_callback)
            self.create_subscription("/steven/reasoning/dist_goal", Float64, self.dist_callback)
            self.create_subscription("/steven/sensing/proximity", prx, self.persistent_dist_callback)

            self.reset = False
            self.pDistCooldown = Duration(seconds=2)
            self.pDistReset = self.get_clock().now()

            self.tsi = 0
            self.hv = False
            self.tsw = self.get_clock().now().nanoseconds * 1e9
            self.dtg = 10
            self.pdto = 5

            #file_path = "/home/txpela/Misc/Fortis_demos/motivation_log.csv"    # Change to where-ever you like
            #log_file = open(file_path, "w", newline="")
            #self.csv_writer = csv.writer(log_file, dialect='excel')
            #self.csv_writer.writerow([
            #            "time_since_interaction",
            #            "human_visible",
            #            "time_since_work",
            #            "distance_to_goal",
            #            "persistent_distance_to_obstacle",
            #            "camera_downtime",
            #            "respond",
            #            "work",
            #            "inform",
            #            "capability",])
        self.create_timer(0.10, self.spinner)

    
    def hearing_callback(self, msg):
        self.keywords = msg.data
    
    def odometry_callback(self, msg):
        self.odo_x = msg.pose.pose.position.x
        self.odo_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.odo_yaw = math.atan2(2*w*z, 1-2*z**2)
        if self.odo_first:
            self.odo_yaw_base = self.odo_yaw
            self.odo_first = False
    
    def agg_mot_callback(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            name = name[12:]
            self.motivations[name] = msg.value[i]
    
    def move_callback(self, msg):
        self.move_in_progress = msg.data
    
    def social_callback(self, msg):
        self.tsi = self.get_clock().now().nanoseconds*1e9 - msg.data

    def vision_callback(self, msg):
        self.hv = msg.visible
        self.cdt = msg.header.stamp.to_sec()

    def work_callback(self, msg):
        if msg:
            self.tsw = self.get_clock().now().nanoseconds*1e9

    def dist_callback(self, msg):
        self.dtg = msg.data

    def persistent_dist_callback(self, msg):
        if msg.distance < self.pdto and self.get_clock().now() - self.pDistReset > self.pDistCooldown:
            self.pdto = msg.distance

        # Distance reset
        if self.reset:
            self.pdto = 5
            self.pDistReset = self.get_clock().now()
            self.reset = False


    def spinner(self):
        # Loop
        keywords = self.keywords.copy()
        self.keywords = []

        # Switch between motivational arbitration and manual commands
        if len(keywords) > 0:
            if "stop" in keywords:
                self.motivated = False
                self.stop_msg.data = "manual"
                self.mode_msg.data = "manual"
                self.mode_pub.publish(self.mode_msg)
                self.stop_pub.publish(self.stop_msg)
                self.get_logger().info("Switching to manual mode")
                return
            elif "begin" in keywords:
                self.motivated = True
                self.mode_msg = "automatic"
                self.mode_pub.publish(self.mode_msg)
                self.get_logger().info("Switching to automatic mode")
                keywords.remove("begin")
        
        # Non-motivated actions
        if len(keywords) > 0 and not self.motivated:
            self.get_logger().info(f'Processing keywords: {keywords}')
            priority = keywords[0]
            self.cmd_msg.command = priority
            self.cmd_msg.args = []
            if priority == "forward":
                self.cmd_msg.args = [1.0, 2.0]
            elif priority == "back":
                self.cmd_msg.args = [-1.0, 2.0]
            self.cmd_msg.header.stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(self.cmd_msg)

        # Motivated actions
        if self.motivated:
            # Recording motivations
            #if self.record:
            #    self.csv_writer.writerow([self.tsi,
            #                              self.hv,
            #                              self.get_clock().now().nanoseconds*1.9 - self.tsw,
            #                              self.dtg,
            #                              self.pdto,
            #                              self.get_clock().now().nanoseconds*1.9 - self.cdt,
            #                              self.motivations["respond"],
            #                              self.motivations["work"],
            #                              self.motivations["inform"],
            #                              self.motivations["capability"]])

            # Check and reset "parallel" motivations
            if self.motivations["inform"] > self.baseline:
                self.reset = True
                self.reset_pub.publish(self.reset_msg)
                self.motivations["inform"] = 0
                #pygame.mixer.music.play()

            # Get maximum motivation
            max_mot = max(self.motivations, key=self.motivations.get)
            max_val = self.motivations[max_mot]
            
            # Check for "enough motivation"
            #rospy.logwarn(f"[COORDINATOR] Current motivation: {self.previous_motivation}")
            if max_val < self.baseline:
                max_mot = "idle"
                max_val = self.baseline

            # Switch-case what we do
            if max_mot == "work":
                if self.previous_motivation != "work":
                    self.get_logger().warn('Switched over to "work"')
                    self.stop_msg.data = self.previous_motivation
                    self.stop_pub.publish(self.stop_msg)
                    self.previous_motivation = "work"
                elif not self.move_in_progress:
                    self.cmd_msg.command = "go"
                    self.cmd_msg.args = [0.042, -1.921, 0.980, -0.198]
                    self.cmd_msg.source = "work"
                    self.command_pub.publish(self.cmd_msg)
            
            elif max_mot == "respond":
                if self.previous_motivation != "respond":
                    self.get_logger().warn('Switched over to "respond"')
                    self.stop_msg.data = self.previous_motivation
                    self.stop_pub.publish(self.stop_msg)
                    self.previous_motivation = "respond"
                elif not self.move_in_progress:
                    self.cmd_msg.command = "turn"
                    self.cmd_msg.args = []
                    self.cmd_msg.source = "respond"
                    self.command_pub.publish(self.cmd_msg)
            
            elif max_mot == "idle":
                if self.previous_motivation != "idle":
                    self.get_logger().warn('Switched over to "idle"')
                    self.stop_msg.data = self.previous_motivation
                    self.stop_pub.publish(self.stop_msg)
                    self.previous_motivation = "idle"



def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        coordinator.get_logger().warn("Shutdown called")
    #except Exception as e:
        #basic_move.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        pass #coordinator.csv_writer.close()


if __name__ == '__main__':
    main()