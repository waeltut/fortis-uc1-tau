import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool, Header, Float64, Int16
from fortis_interfaces.msg import Command as cmd, Hearing as hrn, Vision as vsn, Proximity as prx, AggMotivation as agm, CommandResponse as cmd_res
from fortis_interfaces.action import MoveCommand
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_controller.basic_move import BasicMoveActions2
import math
import csv
import pygame
import json
import numpy as np
import random
import time


"""
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
"""



"""
class Coordinator2(Node):
    def __init__(self):
        super().__init__("coordinator")

        # Constants
        self.movement_types = {
            "UP": {"type": "linear", "FAST": [0.2, 0.1], "SLOW": [0.2, 0.05]},
            # "FORWARD": {"type": "linear", "FAST": [0.2, 0.1], "SLOW": [0.2, 0.05]},
            "DOWN": {"type": "linear", "FAST": [-0.2, -0.1], "SLOW": [-0.2, -0.05]},
            "BACKWARD": {"type": "linear", "FAST": [-0.2, -0.1], "SLOW": [-0.2, -0.05]},
            "LEFT": {"type": "spin", "FAST": [math.pi/4], "SLOW": [math.pi/8]},
            "RIGHT": {"type": "spin", "FAST": [-math.pi/4], "SLOW": [-math.pi/8]},
            "STOP": {"type": "stop", "FAST": [-1.0], "SLOW": [-1.0]},
            }

        # Variables
        self.cognition_command = {"mode_trigger": None, "action": None, "direction": None, "parameter": None}    # Example of structure
        self.job_list = [{"priority": 1.0, "behavior": ["BACK_N_FORTH", 3], "sent": 1.0, "id": random.randint(1, 2**32-1)}]
        #                 {"priority": 1.0, "behavior": ["BACK_N_FORTH", 1], "sent": 1.0, "id": random.randint(1, 2**32-1)}]    # {"priority": 1.0(0-1), "behavior": ["ACTION", "ITEM", "DESTINATION", "REPEAT_N"], "sent": unix}
        self.job_handle = []
        self.job_id = 0
        self.step_id = 0
        self.action_n = 0

        # Substep counters
        self.sent_substeps = 0
        self.received_substeps = 0
        self.accepted_substeps = 0
        self.succeeded_substeps = 0

        # Flags
        self.new_cognition = False
        self.job_ongoing = False
        self.job_done = True

        # Quality of service -options
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # Publishers
        self.command_publisher = self.create_publisher(cmd, "/move_controller/coordinator/action", qos)

        # Subscribers
        self.create_subscription(String, "/cognition/fused/result", self.cognition_callback, 1)
        self.create_subscription(String, "/something/job", self.job_callback, 1)
        self.create_subscription(cmd_res, "/move_controller/basic/received", self.received_callback, qos)
        self.create_subscription(cmd_res, "/move_controller/basic/processing", self.processing_callback, qos)
        self.create_subscription(cmd_res, "/move_controller/basic/done", self.done_callback, qos)

        # Testing / placeholders
        self.test_audio = np.load("/home/fortis_dev/ros2_ws/src/TK33/move_controller/audio_test/audio_test.npy")
        self.test_audio = self.test_audio.astype(float).tolist()

        # Spinning in the nineties
        # self.create_timer(0.2, self.interaction)
        # self.test_zone()
        # self.job_handler(self.job_list[0])
        # self.step_handler()
        # self.step_sender(Movement.BACK_N_FORTH)
        # self.step_sender(Movement.BACKWARD_S_N)
        self.create_timer(0.2, self.working)


    ## CALLBACKS ##
    def cognition_callback(self, msg):
        cognition_command = json.loads(msg.data)
        if cognition_command["mode_trigger"] == "TURN_OFF":
            return
        elif cognition_command["action"] == "STOP":
            self.cognition_command = cognition_command
            self.cognition_command["direction"] = "STOP"
            self.new_cognition = True
            self.get_logger().info(f"Recognized direction {cognition_command['direction']}")
        elif cognition_command["direction"] not in self.movement_types:
            return
        else:
            self.cognition_command = cognition_command
            self.new_cognition = True
            self.get_logger().info(f"Recognized direction {cognition_command['direction']}")

    def job_callback(self, msg):
        job = json.loads(msg.data)
        job["id"] = random.randint(1, 2**32-1)
        self.job_list.append(job)
        self.job_list = sorted(self.job_list, key=lambda x: x["priority"])

    def received_callback(self, msg):
        # Ignore callbacks from other id's
        if msg.id != self.step_id:
            return

        # Handle callback
        self.received_substeps += 1

    def processing_callback(self, msg):
        # Ignore callbacks from other id's
        if msg.id != self.step_id:
            return

        # Handle callback
        if msg.data == Process.ACCEPTED:
            # self.get_logger().info(f"Step id: {self.step_id} substep {self.accepted_substeps} accepted")
            self.accepted_substeps += 1
        elif msg.data == Process.REJECTED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps} rejected")
            self.cancel_job()
        elif msg.data == Process.UNAVAILABLE:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps}'s actionserver unavailable")
            self.cancel_job()
        else:
            print("No clue what else could be")

    def done_callback(self, msg):
        # Ignore callbacks from other id's
        if msg.id != self.step_id:
            return

        # Handle callback
        if msg.data == Done.SUCCEEDED:
            # self.get_logger().info(f"Step id: {self.step_id} substep {self.succeeded_substeps} succeeded")
            self.succeeded_substeps += 1
        elif msg.data == Done.CANCELED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps} cancelled")
            self.cancel_job()
        elif msg.data == Done.ABORTED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps} aborted")
            self.cancel_job()
        
        # When done with step    TODO: Figure out timeout for not all steps going through (as in messages not going through)
        if self.sent_substeps == self.succeeded_substeps:
            self.get_logger().info(f"Step: {self.step_id} completed")
            self.step_handler()


    ## MODES ##
    def interaction(self):
        if self.new_cognition and self.cognition_command is not None:
            direction = self.cognition_command["direction"]
            parameter = self.cognition_command["parameter"]
            if direction in self.movement_types:
                movement = self.movement_types[direction]
                msg = cmd()
                msg.command = movement["type"]
                msg.args = movement[parameter]
                self.command_publisher.publish(msg)
                self.get_logger().info(f"Command '{msg.command}' published")
            else:
                self.get_logger().info(f"Failed to recognize direction {direction}")
            self.cognition_command = None
            self.new_cognition = False

    def working(self):
        if len(self.job_list) > 0 and not self.job_ongoing and not self.job_done:
            # Clear finished job and set flag
            self.job_list.pop(0) if len(self.job_list) > 0 else 0
            self.job_id = 0
            self.step_id = 0
            self.job_done = True

        elif len(self.job_list) > 0 and self.job_done:
           # Set flags
            self.job_ongoing = True
            self.job_done = False

            # Perform the given job
            job = self.job_list[0]
            self.job_handler(job)
            self.step_handler()    # Kickstart job

        elif len(self.job_list) == 0:
            # Failsafe for empty buffer
            self.action_done = True
            self.action_ongoing = False
            self.job_handle = []
            self.job_id = 0
            self.step_id = 0


    ## JOB PROCESSOR ##  TODO: Could make this recursive, but good for now
    # Decrypt into lower-level actions
    def job_handler(self, job):
        behavior = job["behavior"]
        action = behavior[0]
        self.job_id = job["id"]

        # No switches nor cases
        # TODO: Error handling
        if action == "DELIVER":
            # Decomposition
            item = getattr(Movement, behavior[1])
            destination = Movement.STATIONS[behavior[2]] if isinstance(behavior[2], int) else getattr(Movement, behavior[2])
            repeats = behavior[3]

            # Job thingy
            self.job_handle = [item, Movement.SHAKE, destination] * int(repeats)
            self.action_n = len(self.job_handle)
        elif action == "RETRIEVE":
            # TODO: Retrieve to position
            print("asfwe")
        elif hasattr(Movement, action):
            # Job
            repeats = behavior[1]
            self.job_handle = [getattr(Movement, action)] * int(repeats)
            self.action_n = int(repeats)
        else:
            # Everything else
            self.get_logger().error(f"Action type '{action}' not recognized")
            return
        self.get_logger().info(f"Job: {self.job_id} accepted")

    # Separates steps from jobs
    def step_handler(self):
        # If no steps left in a job, finish job
        if len(self.job_handle) <= 0:
            self.get_logger().info(f"Job {self.job_id} completed")
            self.job_ongoing = False
            return
        step = self.job_handle.pop(0)
        self.step_sender(step)

    # Send individual job-steps to controller
    def step_sender(self, steps):
        # Figure out if step consists of multiple primitives
        steps = steps if any(isinstance(x, tuple) for x in steps) else (steps,)    # If step is not nested, nest it

        # Reset substep counters
        self.sent_substeps = 0
        self.received_substeps = 0
        self.accepted_substeps = 0
        self.succeeded_substeps = 0

        # Assign id for step
        self.step_id = random.randint(1, 2**32-1)
        msg = cmd()
        msg.id = self.step_id

        # Send all substeps
        for step in steps:
            msg.command = step[0]
            msg.args = list(step[1:])
            self.command_publisher.publish(msg)
            self.sent_substeps += 1
        self.get_logger().info(f"Sent step: {msg.id}")

    # Cancellation
    def cancel_job(self):
        # Cancel the rest of the step
        remaining_substeps = self.accepted_substeps - self.succeeded_substeps
        msg = cmd()
        msg.command = "stop"
        msg.id = self.step_id
        msg.args = [float(remaining_substeps)]
        self.command_publisher.publish(msg)

        # Clear job handle and mark as done
        self.job_handle = []
        self.step_handler()


    ## TESTING ##
    def test_zone(self):
        msg = cmd()
        msg.command = "speak"
        msg.args = self.test_audio
        self.command_publisher.publish(msg)
"""



class Coordinator3(Node):
    def __init__(self):
        super().__init__("coordinator")

        # Constants
        self.movement_types = {
            "UP": {"type": "linear", "FAST": [0.2, 0.1], "SLOW": [0.2, 0.05]},
            # "FORWARD": {"type": "linear", "FAST": [0.2, 0.1], "SLOW": [0.2, 0.05]},
            "DOWN": {"type": "linear", "FAST": [-0.2, -0.1], "SLOW": [-0.2, -0.05]},
            "BACKWARD": {"type": "linear", "FAST": [-0.2, -0.1], "SLOW": [-0.2, -0.05]},
            "LEFT": {"type": "spin", "FAST": [math.pi/4], "SLOW": [math.pi/8]},
            "RIGHT": {"type": "spin", "FAST": [-math.pi/4], "SLOW": [-math.pi/8]},
            "STOP": {"type": "stop", "FAST": [-1.0], "SLOW": [-1.0]},
            }
        self.motivation_swap_cycles = 5    # A full second

        # Variables
        self.cognition_command = {"mode_trigger": None, "action": None, "direction": None, "parameter": None}    # Example of structure
        self.job_list = [{"priority": 1.0, "behavior": ["DELIVER", "SCREW", "HOME", 3], "transient": False, "id": random.randint(1, 2**32-1)}]
                        # {"priority": 1.0, "behavior": ["HOME", 1], "transient": False, "id": random.randint(1, 2**32-1)}]    # {"priority": 1.0(0-1), "behavior": ["ACTION", "ITEM", "DESTINATION", "REPEAT_N"], "transient": bool}
        self.job_handle = []
        self.current_job = None
        self.step_queueueue = []
        self.job_id = 0
        self.step_id = 0
        self.action_n = 0
        self.motivation = "work"
        self.previous_motivation = "work"
        self.motivations = {}
        self.abort_pose = None
        self.human_pose = None
        self.swap_ticker = 0

        # Substep counters
        self.sent_substeps = 0
        self.accepted_substeps = 0
        self.succeeded_substeps = 0

        # Flags
        self.new_cognition = False
        self.job_ongoing = False
        self.job_done = True
        self.stopping = False
        self.seen = False
        self.acknowledged = False
        self.swapping = False
        self.test_done = False
        self.another_flag = False

        # Quality of service -options
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        # Transforming my will to live, into whatever this is
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.command_publisher = self.create_publisher(cmd, "/move_controller/coordinator/action", qos)
        self.acknowledged_publisher = self.create_publisher(String, "/professional_namespace/assistance_required", 1)
        self.human_pose_publisher = self.create_publisher(PoseStamped, "/goal_update", 1)

        # Acton's nightmare
        self.move_client = ActionClient(self, MoveCommand, "basic_move")
        self.step_timer = None
        self.steps = None

        # Subscribers
        self.create_subscription(String, "/cognition/fused/result", self.cognition_callback, 1)
        self.create_subscription(String, "/something/job", self.job_callback, 1)
        self.create_subscription(agm, "/steven/reasoning/agg_motivation", self.agg_callback, 1)
        self.create_subscription(PoseStamped, "/steven/sensing/pose", self.pose_callback, 1)
        self.create_subscription(hrn, "/steven/sensing/hearing", self.hearing_callback, 1)

        # Testing / placeholders
        self.test_audio = np.load("/home/fortis_dev/ros2_ws/src/TK33/move_controller/audio_test/audio_test.npy")
        self.test_audio = self.test_audio.astype(float).tolist()

        # Spinning in the nineties
        # self.create_timer(0.2, self.interaction)
        # self.test_zone()
        # self.job_handler(self.job_list[0])
        # self.step_handler()
        # self.step_sender(Movement.BACK_N_FORTH)
        # self.step_sender(Movement.BACKWARD_S_N)
        self.create_timer(0.2, self.work)
        # self.inform()
        # self.create_timer(1.0, self.test_zone)
        # self.create_timer(0.2, self.respond)


    ## CALLBACKS ##
    def cognition_callback(self, msg):
        self.get_logger().info("Message arrived")
        cognition_command = json.loads(msg.data)
        if cognition_command["mode_trigger"] == "TURN_OFF":
            return
        elif cognition_command["action"] == "STOP":
            self.cognition_command = cognition_command
            self.cognition_command["direction"] = "STOP"
            self.new_cognition = True
            self.get_logger().info(f"Recognized direction {cognition_command['direction']}")
        elif cognition_command["direction"] not in self.movement_types:
            return
        else:
            self.cognition_command = cognition_command
            self.new_cognition = True
            self.get_logger().info(f"Recognized direction {cognition_command['direction']}")

    def job_callback(self, msg):
        job = json.loads(msg.data)
        job["id"] = random.randint(1, 2**32-1)
        self.job_list.append(job)
        self.job_list = sorted(self.job_list, key=lambda x: x["priority"], reverse=True)

    def agg_callback(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            name = name[29:]
            self.motivations[name] = msg.value[i]

    def pose_callback(self, msg):
        self.human_pose = msg

    def hearing_callback(self, msg):
        if self.seen and 'yes' in msg.data:
            self.acknowledged = True
        elif self.another_flag and 'done' in msg.data[0]:
            msg_sent = String()
            msg_sent.data = "done"
            self.acknowledged_publisher.publish(msg_sent)


    ## ACTION BULLSHIT ##    TODO: Handling of "abort not successful"
    def basic_goal_callback(self, future):
        # Handle feedback
        goal_handle = future.result()
        result_handle = goal_handle.get_result_async()
        result_handle.add_done_callback(self.basic_result_callback)

    def basic_feedback_callback(self, feedback):
        if feedback.feedback.status == BasicMoveActions2.ACCEPTED:
            # self.get_logger().info(f"Step id: {self.step_id} substep {self.accepted_substeps} accepted")
            self.accepted_substeps += 1
        elif feedback.feedback.status == BasicMoveActions2.REJECTED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps} rejected")
        elif feedback.feedback.status == BasicMoveActions2.UNAVAILABLE:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps}'s actionserver unavailable")
        else:
            print("No clue what else could be")

    def basic_result_callback(self, feedback):
        # Something, something callback
        result = feedback.result()
        if result.result.status == BasicMoveActions2.SUCCEEDED:
            self.get_logger().info(f"Step id: {self.step_id} substep {self.succeeded_substeps} succeeded")
            self.succeeded_substeps += 1
        elif result.result.status == BasicMoveActions2.CANCELED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.succeeded_substeps} cancelled")
            self.cancel_job()
        elif result.result.status == BasicMoveActions2.ABORTED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.succeeded_substeps} aborted")
            if self.current_job and not self.current_job["transient"]:
                self.job_list.append(self.current_job)    # We can redo this job later
                self.job_list = sorted(self.job_list, key=lambda x: x["priority"], reverse=True)
                self.get_pose()
                self.cancel_job("aborted")
            else:
                self.cancel_job()
        elif result.result.status == BasicMoveActions2.UNAVAILABLE:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps}'s actionserver unavailable")
            self.cancel_job()
        elif result.result.status == BasicMoveActions2.REJECTED:
            self.get_logger().warn(f"Job: {self.job_id}, Step: {self.step_id}, Substep: {self.accepted_substeps} rejected")
        
        # if done, then done
        if self.sent_substeps == self.succeeded_substeps:
            self.get_logger().info(f"Step: {self.step_id} completed")
            self.step_handler()


    ## MODES ##
    def motivated_reasoning(self):
        # Check for motivations not arriving
        if not self.motivations:
            return

        # Switch behavior based on motivation
        self.motivation = max(self.motivations, key=self.motivations.get)

        # Only switch modes when a succifient amount of cycles in a row have the required motivation (helps with "flickering")
        if self.motivation != self.previous_motivation:
            self.swap_ticker += 1
        else:
            self.swap_ticker = 0
        if self.swap_ticker >= self.motivation_swap_cycles:
            self.swap_ticker = 0
            self.mode_switcher()
            return
        if self.swapping:
            return

        # Get correct operating cycle
        if hasattr(self, self.previous_motivation):
            function_handle = getattr(self, self.previous_motivation)
            function_handle()
        else:
            self.get_logger().info(f"Unknown mode: {self.previous_motivation}")
            self.previous_motivation = "work"    # Default to work

    def respond(self):
        if self.new_cognition and self.cognition_command is not None:
            direction = self.cognition_command["direction"]
            parameter = self.cognition_command["parameter"]
            if direction in self.movement_types:
                movement = self.movement_types[direction]
                msg = MoveCommand.Goal()
                msg.command = movement["type"]
                msg.args = movement[parameter]
                self.move_client.send_goal_async(msg, self.basic_feedback_callback)
                self.get_logger().info(f"Command '{msg.command}' published")
            else:
                self.get_logger().info(f"Failed to recognize direction {direction}")
            self.cognition_command = None
            self.new_cognition = False

    def work(self):
        if len(self.job_list) > 0 and not self.job_ongoing and not self.job_done:
            # Clear finished job and set flag
            self.job_id = 0
            self.step_id = 0
            self.stopping = False
            self.job_done = True
            self.current_job = None

        elif len(self.job_list) > 0 and self.job_done:
           # Set flags
            self.job_ongoing = True
            self.job_done = False

            # Perform the given job
            self.current_job = self.job_list.pop(0)
            self.job_handler(self.current_job)
            self.step_handler()    # Kickstart job

        elif len(self.job_list) == 0 and self.job_done:
            # Failsafe for empty buffer
            self.action_done = True
            self.action_ongoing = False
            self.current_job = None
            self.job_handle = []
            self.job_id = 0
            self.step_id = 0

    def inform(self):
        # Let motivation know that human on the case
        if not self.seen and self.human_pose:
            self.get_logger().info('Here')
            self.seen = True
            self.job_ongoing = True
            self.job_done = False

            # Request help
            yaw = 2.0 * math.atan2(self.human_pose.pose.orientation.z, self.human_pose.pose.orientation.w)
            motion = ["follow", self.human_pose.pose.position.x - 1.0*math.cos(yaw), self.human_pose.pose.position.y - 1.0*math.sin(yaw), self.human_pose.pose.orientation.z, self.human_pose.pose.orientation.w]
            speech = self.test_audio
            speech.insert(0, "speak")
            self.job_id = random.randint(1, 2**32-1)
            self.job_handle = [tuple(motion), tuple(speech)]
            self.step_handler()

        if self.acknowledged and self.abort_pose:
            self.get_logger().info('There')
            self.acknowledged = False
            self.another_flag = True
            msg = String()
            msg.data = "acknowledged"
            self.acknowledged_publisher.publish(msg)
            motion = ["inform_go", self.abort_pose.pose.position.x, self.abort_pose.pose.position.y, self.abort_pose.pose.orientation.z, self.abort_pose.pose.orientation.w]
            self.job_handle = [tuple(motion)]
            self.step_handler()

    # For handling mode-transitions
    def mode_switcher(self):
        if not self.swapping:
            self.swapping = True
            # If current job not transient, save it
            if self.current_job and not self.current_job["transient"]:
                self.job_list.append(self.current_job)
                self.job_list = sorted(self.job_list, key=lambda x: x["priority"], reverse=True)
    
            # Send a cancellation request for current job
            self.get_logger().error("Not supposed to be here until human")
            msg = MoveCommand.Goal()
            msg.id = self.step_id
            msg.command = "stop"
            msg.args = [0.0]
            self.move_client.send_goal_async(msg)

        # When completely stopped, allow swap
        if not self.job_ongoing:
            self.previous_motivation = self.motivation
            self.job_id = 0
            self.step_id = 0

            # Reset all flags
            self.stopping = False
            self.job_done = True
            self.current_job = None
            self.swapping = False
            self.seen = False
            self.acknowledged = False
            self.another_flag = False



    ## JOB PROCESSOR ##  TODO: Could make this recursive, but good for now
    # Decrypt into lower-level actions
    def job_handler(self, job):
        behavior = job["behavior"]
        action = behavior[0]
        self.job_id = job["id"]

        # No switches nor cases
        # TODO: Error handling
        if action == "DELIVER":
            # Decomposition
            item = getattr(Movement, behavior[1])
            destination = Movement.STATIONS[behavior[2]] if isinstance(behavior[2], int) else getattr(Movement, behavior[2])
            repeats = behavior[3]

            # Job thingy
            self.job_handle = [item, Movement.SHAKE, destination] * int(repeats)
            self.action_n = len(self.job_handle)
        elif action == "RETRIEVE":
            # TODO: Retrieve to position
            print("asfwe")
        elif hasattr(Movement, action):
            # Job
            repeats = behavior[1]
            self.job_handle = [getattr(Movement, action)] * int(repeats)
            self.action_n = int(repeats)
            self.get_logger().info(f"{action}")
        else:
            # Everything else
            self.get_logger().error(f"Job type '{action}' not recognized")
            return
        self.get_logger().info(f"Job: {self.job_id} accepted")

    # Separates steps from jobs
    def step_handler(self):
        # If no steps left in a job, finish job
        if len(self.job_handle) <= 0:
            self.get_logger().info(f"Job: {self.job_id} completed")
            self.job_ongoing = False
            self.stopping = False
            return
        step = self.job_handle.pop(0)
        self.step_sender(step)

    # Send individual job-steps to controller
    def step_sender(self, steps):
        # Figure out if step consists of multiple primitives
        self.steps = steps if any(isinstance(x, tuple) for x in steps) else (steps,)    # If step is not nested, nest it
        self.steps = list(self.steps)

        # Zero step-counters
        self.sent_substeps = 0
        self.accepted_substeps = 0
        self.succeeded_substeps = 0

        # Assign id for step
        self.step_id = random.randint(1, 2**32-1)

        # Sequence and non-blocking
        self.step_timer = self.create_timer(0.05, self.substep_sender)

    def substep_sender(self):
        if not self.steps:
            self.step_timer.cancel()
            self.get_logger().info(f"Sent step: {self.step_id}")
            return

        # Send substep
        step = self.steps.pop(0)
        self.get_logger().info(f"{step[0]}, {step[1:]}")
        msg = MoveCommand.Goal()
        msg.id = self.step_id
        msg.command = step[0]
        msg.args = list(step[1:])
        future = self.move_client.send_goal_async(msg, self.basic_feedback_callback)
        future.add_done_callback(self.basic_goal_callback)
        self.sent_substeps += 1
        

    # Cancellation    # TODO: I should really make this shit thread-safe
    def cancel_job(self, reason=""):
        # Check if already stopping
        if self.stopping:
            return
        self.stopping = True

        # If stopped due to error, return home
        if reason=="aborted":
            self.job_list.append({"priority": 1.5, "behavior": ["HOME", 1], "transient": True, "id": random.randint(1, 2**32-1)})
            self.job_list = sorted(self.job_list, key=lambda x: x["priority"], reverse=True)

        # Clear job handle and mark as done
        self.job_handle = []
        self.step_handler()

    # Get current pose
    def get_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            self.abort_pose = PoseStamped()
            self.abort_pose.header.frame_id = "map"
            self.abort_pose.pose.position.x = transform.transform.translation.x
            self.abort_pose.pose.position.y = transform.transform.translation.y
            self.abort_pose.pose.orientation.z = transform.transform.rotation.z
            self.abort_pose.pose.orientation.w = transform.transform.rotation.w
            self.get_logger().info("")
        except Exception as e:
            self.get_logger().warning(f"Could not get robot pose: {e}")


    ## TESTING ##
    def test_zone(self):
        """
        msg = MoveCommand.Goal()
        msg.id = random.randint(1, 2**32-1)
        msg.command = "speak"
        msg.args = self.test_audio
        self.move_client.send_goal_async(msg, self.basic_feedback_callback)
        """

        if self.human_pose and not self.test_done:
            msg = MoveCommand.Goal()
            msg.command = "follow"
            msg.id = random.randint(1, 2**32-1)
            yaw = 2.0 * math.atan2(self.human_pose.pose.orientation.z, self.human_pose.pose.orientation.w)
            msg.args = [self.human_pose.pose.position.x - 1.0*math.cos(yaw), self.human_pose.pose.position.y - 1.0*math.sin(yaw), self.human_pose.pose.orientation.z, self.human_pose.pose.orientation.w]
            self.move_client.send_goal_async(msg)
            self.test_done = True
            self.get_logger().info("Sent request")
        elif self.human_pose:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            yaw = 2.0 * math.atan2(self.human_pose.pose.orientation.z, self.human_pose.pose.orientation.w)
            msg.pose.position.x = self.human_pose.pose.position.x - 1.0*math.cos(yaw)
            msg.pose.position.y = self.human_pose.pose.position.y - 1.0*math.sin(yaw)
            msg.pose.orientation.z = self.human_pose.pose.orientation.z
            msg.pose.orientation.w = self.human_pose.pose.orientation.w
            self.human_pose_publisher.publish(msg)




## MOVEMENT TYPES ##
class Movement:
    # Position templates
    SCREW = ("go", 9.913, 43.21, -0.472, 0.882)
    HOME = ("go", 12.70, 35.99, 0.765, 0.644)
    BACK = ("go", -1.0, -1.0, -1.0, -1.0)    # -1.0 means position when command was processed
    STATIONS = (
        ("go", 12.70, 35.99, 0.0, 0.0),    # Totally not home 1
        ("go", 12.70, 35.99, 1.0, 1.0),    # Totally not home 2
    )

    # Primitive action templates (spin)
    SPIN_NEG = ("spin", -2*math.pi)
    SPIN_POS = ("spin", 2*math.pi)
    TURN_NEG_S = ("spin", -20*math.pi/180)
    TURN_NEG_M = ("spin", -40*math.pi/180)
    TURN_NEG_L = ("spin", -80*math.pi/180)
    TURN_POS_S = ("spin", 20*math.pi/180)
    TURN_POS_M = ("spin", 40*math.pi/180)
    TURN_POS_L = ("spin", 80*math.pi/180)

    # Primitive action templates (forward)
    FORWARD_S_N = ("linear", 0.2, 0.1)
    FORWARD_S_M = ("linear", 0.4, 0.1)
    FORWARD_S_F = ("linear", 0.8, 0.1)
    FORWARD_F_N = ("linear", 0.2, 0.2)
    FORWARD_F_M = ("linear", 0.4, 0.2)
    FORWARD_F_F = ("linear", 0.8, 0.2)

    # Primitive action templates (backward)
    BACKWARD_S_N = ("linear", -0.2, -0.1)
    BACKWARD_S_M = ("linear", -0.4, -0.1)
    BACKWARD_S_F = ("linear", -0.8, -0.1)
    BACKWARD_F_N = ("linear", -0.2, -0.2)
    BACKWARD_F_M = ("linear", -0.4, -0.2)
    BACKWARD_F_F = ("linear", -0.8, -0.2)

    # Composite actions (mostly testing)
    SHAKE = (TURN_NEG_S, TURN_POS_M, TURN_NEG_M, TURN_POS_M, TURN_NEG_M, TURN_POS_S)
    DANCE = (FORWARD_F_N, BACKWARD_F_N, SPIN_NEG, FORWARD_F_N, BACKWARD_F_N, SPIN_POS)
    DANCE_SAFE = (FORWARD_F_N, BACKWARD_F_N, TURN_NEG_S, TURN_POS_M, TURN_NEG_M, TURN_POS_S)
    BACK_N_FORTH = (FORWARD_S_N, BACKWARD_S_N, FORWARD_S_N, BACKWARD_S_N)
    BACK_N_FORTH_F = (FORWARD_F_M, BACKWARD_F_M, FORWARD_F_M, BACKWARD_F_M)

    # Commands
    RETRIEVE = "Deliver item X to position from whence the command came"
    DELIVER = "Deliver item X to station Y"
    CAUSE_A_SCENE = "Go home and start dancing"
    STATIONARY_TEST = "Perform back and forth twice"



def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator3()
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