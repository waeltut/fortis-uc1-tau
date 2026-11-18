import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory

from datetime import datetime
import os
import csv
import math



class deb_node(Node):

    def __init__(self):
        super().__init__('back_and_forth_patrol')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ---------- Pose A ----------
        pose_a = PoseStamped()
        pose_a.header.frame_id = 'map'
        pose_a.pose.position.x = 20.5
        pose_a.pose.position.y = 22.0
        pose_a.pose.position.z = 0.0
        pose_a.pose.orientation.z = 0.95
        pose_a.pose.orientation.w = 0.3

        # ---------- Pose B ----------
        pose_b = PoseStamped()
        pose_b.header.frame_id = 'map'
        pose_b.pose.position.x = 13.6
        pose_b.pose.position.y = 34.9
        pose_b.pose.position.z = 0.0
        pose_b.pose.orientation.z = -0.95
        pose_b.pose.orientation.w = 0.3

        self.poses = [pose_a, pose_b]
        self.current_goal_index = 0

        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        pose = self.poses[self.current_goal_index]
        pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal {self.current_goal_index + 1}: "
            f"({pose.pose.position.x}, {pose.pose.position.y})"
        )

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):
        result = future.result()

        self.get_logger().info(
            f"Goal completed with status: {result.status}"
        )

        # Switch to next pose
        self.current_goal_index = (
            self.current_goal_index + 1
        ) % len(self.poses)

        self.send_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f"Distance remaining: "
                f"{feedback.distance_remaining:.2f} m"
            )



class deb_node2(Node):
    def __init__(self):
        super().__init__('tuning_info')

        # Update with new odom
        self.update = False

        # Latest measurements
        self.amcl_yaw = None
        self.odom_yaw = None
        self.amcl_x = None
        self.amcl_y = None
        self.odom_x = None
        self.odom_y = None

        # Initial values
        self.amcl_initial_yaw = None
        self.odom_initial_yaw = None
        self.amcl_initial_x = None
        self.amcl_initial_y = None
        self.odom_initial_x = None
        self.odom_initial_y = None

        # ROS timestamps
        self.amcl_stamp = None
        self.odom_stamp = None

        # Experiment start time
        self.start_time = self.get_clock().now()

        # Create output file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        package_dir = os.path.expanduser("~/ros2_ws/src/SA41/debugging_node/")
        self.filename = os.path.join(
            package_dir,
            'tuning_data',
            'amcl',
            f'comparison_{timestamp}.csv'
        )
        self.file = open(
            self.filename,
            'w',
            newline='',
            buffering=1
        )
        self.writer = csv.writer(self.file)

        # CSV header
        self.writer.writerow([
            'elapsed_time_sec',
            'amcl_stamp_sec',
            'odom_stamp_sec',
            'amcl_yaw_deg',
            'odom_yaw_deg',
            'amcl_x',
            'amcl_y',
            'odom_x',
            'odom_y',
            'yaw_difference_deg',
            'diff_x',
            'diff_y',
        ])

        # I love buffers
        # self.file.flush()

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Logging
        self.get_logger().info('Yaw comparison started.')
        self.get_logger().info(f'Logging data to:\n{self.filename}')

        # Spinner
        self.writer_timer = self.create_timer(
            0.1,   # 10 Hz
            self.write_data
        )


    # Helpers
    @staticmethod
    def message_stamp(msg):
        stamp = msg.header.stamp
        return (stamp.sec + stamp.nanosec * 1e-9)
    
    def get_elapsed_time(self):
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9


    # Callbacks
    def amcl_callback(self, msg):
        # Want angle
        self.amcl_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        self.amcl_stamp = self.message_stamp(msg)

        # First AMCL measurement becomes reference
        if self.amcl_initial_yaw is None:
            self.amcl_initial_yaw = self.amcl_yaw
        if self.amcl_initial_x is None:
            self.amcl_initial_x = self.amcl_x
        if self.amcl_initial_y is None:
            self.amcl_initial_y = self.amcl_y

    def odom_callback(self, msg):
        # Want angle
        self.odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_stamp = self.message_stamp(msg)

        # First odom measurement becomes reference
        if self.odom_initial_yaw is None:
            self.odom_initial_yaw = self.odom_yaw
        if self.odom_initial_x is None:
            self.odom_initial_x = self.odom_x
        if self.odom_initial_y is None:
            self.odom_initial_y = self.odom_y

        # New row required
        self.update = True


    # Data writing
    def write_data(self):
        # Don't write anything until update required
        if not self.update:
            return

        # Timing
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds * 1e-9

        # Odom
        odom_yaw = wrap_angle(self.odom_yaw - self.odom_initial_yaw)
        odom_yaw_deg = math.degrees(odom_yaw)

        # Difference and amcl
        if (self.amcl_yaw is not None) and (self.amcl_x is not None) and (self.amcl_y is not None):
            amcl_yaw = wrap_angle(self.amcl_yaw - self.amcl_initial_yaw)
            amcl_yaw_deg = math.degrees(amcl_yaw)
            amcl_x = self.amcl_x - self.amcl_initial_x
            amcl_y = self.amcl_y - self.amcl_initial_y
            amcl_stamp = self.amcl_stamp
            
            # Diff
            yaw_difference = wrap_angle(amcl_yaw - odom_yaw)
            yaw_difference_deg = math.degrees(yaw_difference)
            diff_x = amcl_x - self.odom_x
            diff_y = amcl_y - self.odom_y

        else:
            amcl_yaw_deg = None
            amcl_x = None
            amcl_y = None
            amcl_stamp = None
            yaw_difference_deg = None
            diff_x = None
            diff_y = None

        # Writing
        self.writer.writerow([
            f'{elapsed_time:.6f}',
            amcl_stamp,
            self.odom_stamp,
            amcl_yaw_deg,
            odom_yaw_deg,
            amcl_x,
            amcl_y,
            self.odom_x - self.odom_initial_x,
            self.odom_y - self.odom_initial_y,
            yaw_difference_deg,
            diff_x,
            diff_y,
        ])
        self.file.flush()

        # Update finished
        self.update = False

    # The writing monster (not in use, as extremely unintuitive)
    def write_sample(self, source):
        # Timing stuff
        elapsed_time = self.get_elapsed_time()
        ros_time = (self.get_clock().now().nanoseconds * 1e-9)

        # Current values
        amcl_yaw_deg = None
        odom_yaw_deg = None
        amcl_change_deg = None
        odom_change_deg = None
        yaw_change_error_deg = None

        # amcl -handler
        if self.amcl_yaw is not None:
            amcl_yaw_deg = math.degrees(self.amcl_yaw)
            if self.amcl_initial_yaw is not None:
                amcl_change = wrap_angle(self.amcl_yaw - self.amcl_initial_yaw)
                amcl_change_deg = math.degrees(amcl_change)

        # odom -handler
        if self.odom_yaw is not None:
            odom_yaw_deg = math.degrees(self.odom_yaw)
            if self.odom_initial_yaw is not None:
                odom_change = wrap_angle(self.odom_yaw - self.odom_initial_yaw)
                odom_change_deg = math.degrees(odom_change)

        # Only calculate error if both changes exist
        if (
            self.amcl_yaw is not None and
            self.odom_yaw is not None and
            self.amcl_initial_yaw is not None and
            self.odom_initial_yaw is not None
        ):
            amcl_change = wrap_angle(self.amcl_yaw - self.amcl_initial_yaw)
            odom_change = wrap_angle(self.odom_yaw - self.odom_initial_yaw)
            yaw_change_error = wrap_angle(amcl_change - odom_change)
            yaw_change_error_deg = math.degrees(yaw_change_error)

        # ROS timestamps
        amcl_stamp = (self.amcl_stamp if self.amcl_stamp is not None else '')
        odom_stamp = (self.odom_stamp if self.odom_stamp is not None else '')

        # Write row
        self.writer.writerow([
            f'{elapsed_time:.6f}',
            f'{ros_time:.6f}',
            f'{amcl_stamp:.6f}' if amcl_stamp != '' else '',
            f'{odom_stamp:.6f}' if odom_stamp != '' else '',
            f'{amcl_yaw_deg:.6f}' if amcl_yaw_deg != '' else '',
            f'{odom_yaw_deg:.6f}' if odom_yaw_deg != '' else '',
            f'{amcl_change_deg:.6f}' if amcl_change_deg != '' else '',
            f'{odom_change_deg:.6f}' if odom_change_deg != '' else '',
            f'{yaw_change_error_deg:.6f}' if yaw_change_error_deg != '' else ''
        ])

        # Buff
        self.file.flush()


    # Close file
    def destroy_node(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            self.get_logger().info(f'Data saved to:\n{self.filename}')
        super().destroy_node()



def quaternion_to_yaw(q):
    # Convert quaternion to yaw in radians
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)



def wrap_angle(angle):
    # Normalize angle to [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))



def main(args=None):
    rclpy.init(args=args)

    # node = deb_node()
    node = deb_node2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()