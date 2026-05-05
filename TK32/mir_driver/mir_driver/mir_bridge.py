import time
import rclpy
from rclpy import qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from rclpy.clock import Clock
import roslibpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Header,Float64, UInt8
from mir_msgs.msg import MirState
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import signal
import yaml



## MESSAGE CONVERTERS ##
def tf_conv(msg):
    tf_msg = TFMessage()
    
    for t in msg['transforms']:
        ts = TransformStamped()
        
        # Header
        ts.header.stamp = Clock().now().to_msg()
        ts.header.frame_id = t['header']['frame_id']
        
        # Child frame
        ts.child_frame_id = t['child_frame_id']
        
        # Translation
        ts.transform.translation = Vector3(
            x=t['transform']['translation']['x'],
            y=t['transform']['translation']['y'],
            z=t['transform']['translation']['z']
        )
        
        # Rotation
        ts.transform.rotation = Quaternion(
            x=t['transform']['rotation']['x'],
            y=t['transform']['rotation']['y'],
            z=t['transform']['rotation']['z'],
            w=t['transform']['rotation']['w']
        )
        
        tf_msg.transforms.append(ts)
    
    return tf_msg


def laser_conv(msg):
    scan = LaserScan()

    # Header
    scan.header.stamp = Clock().now().to_msg()
    scan.header.frame_id = msg['header']['frame_id']

    # Laser parameters
    scan.angle_min = msg['angle_min']
    scan.angle_max = msg['angle_max']
    scan.angle_increment = msg['angle_increment']
    scan.time_increment = msg['time_increment']
    scan.scan_time = msg['scan_time']
    scan.range_min = msg['range_min']
    scan.range_max = msg['range_max']

    # Data
    scan.ranges = list(msg['ranges'])
    scan.intensities = list(msg['intensities'])

    return scan

def odom_conv(msg):
    odom = Odometry()

    # Header
    odom.header.stamp = Clock().now().to_msg()
    odom.header.frame_id = msg['header']['frame_id']

    # Child frame
    odom.child_frame_id = msg['child_frame_id']

    # Pose
    odom.pose.pose.position = Point(
        x=msg['pose']['pose']['position']['x'],
        y=msg['pose']['pose']['position']['y'],
        z=msg['pose']['pose']['position']['z']
    )
    odom.pose.pose.orientation = Quaternion(
        x=msg['pose']['pose']['orientation']['x'],
        y=msg['pose']['pose']['orientation']['y'],
        z=msg['pose']['pose']['orientation']['z'],
        w=msg['pose']['pose']['orientation']['w']
    )
    odom.pose.covariance = list(msg['pose']['covariance'])

    # Twist
    odom.twist.twist.linear = Vector3(
        x=msg['twist']['twist']['linear']['x'],
        y=msg['twist']['twist']['linear']['y'],
        z=msg['twist']['twist']['linear']['z']
    )
    odom.twist.twist.angular = Vector3(
        x=msg['twist']['twist']['angular']['x'],
        y=msg['twist']['twist']['angular']['y'],
        z=msg['twist']['twist']['angular']['z']
    )
    odom.twist.covariance = list(msg['twist']['covariance'])

    return odom

def cmd_vel_conv(msg):
    return {
        'header': {
            'stamp': {
                'secs': 0,
                'nsecs': 0
            },
            'frame_id': 'base_link'
        },
        'twist': {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        }
    }

def std_msg_f64(msg):
    f64=Float64()
    f64.data = msg['data']
    return f64

def mir_state(msg):
    state=MirState()
    state.mir_state = msg['robotState']
    state.mir_state_string = msg['robotStateString']

    return state




## CONSTANTS ##
TYPES = {
    "/f_scan": [LaserScan, laser_conv],
    "/b_scan": [LaserScan, laser_conv],
    "/odom": [Odometry, odom_conv],
    "/tf": [TFMessage, tf_conv],
    "/tf_static": [TFMessage, tf_conv, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)],
    "/cmd_vel": [Twist, cmd_vel_conv],
    "/MC/battery_percentage": [Float64, std_msg_f64],
    "/robot_state": [MirState, mir_state],
    "/scan": [LaserScan, laser_conv],
}    # The qos-profiles can/should be changed



## FUNCTIONALITY ##
# Publishers (remote)
class Publisher(object):
    def __init__(self, topic, node, client):
        #super().__init__(f'{topic["topic"][1:]}_publisher')
        # Variables
        self.topic = topic
        self.node = node
        self.client = client
        self.converter = TYPES[topic["topic"]][1]
        self.shutdown_flag = False

        # Publish remotely, subscribe locally
        try:
            self.pub_remote = roslibpy.Topic(client, topic["topic"], topic["type"])
            self.sub_local = node.create_subscription(
                TYPES[topic["topic"]][0],
                topic["topic"],
                self.callback,
                1,
            )
            node.get_logger().info(f"Publishing to: {self.topic['topic']}")
        except:
            pass
    
    # When info from ros2, then publish remotely
    def callback(self, msg):
        if not rclpy.ok():
            return
        self.pub_remote.publish(self.converter(msg))



# Subscribers (remote)
class Subscriber(object):
    def __init__(self, topic, node, client):
        # Variables
        self.last_static_msg = None
        self.topic = topic
        self.node = node
        self.client = client
        self.converter = TYPES[topic["topic"]][1]
        self.shutdown_flag = False
        self.static_valve = False

        # Transforms are hillarious
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)

        # Publish locally, subscribe remotely
        try:
            self.pub_local = node.create_publisher(
                TYPES[topic["topic"]][0],
                topic["topic"],
                1 if topic["topic"] != "/tf_static" else TYPES[topic["topic"]][2],    # tf_static needs specific qos
            )
            self.sub_remote = roslibpy.Topic(client, topic["topic"], topic["type"])
            
            # /tf_static 
            if self.topic["topic"] == "/tf_static":
                self.sub_remote.subscribe(self.tf_static_callback)
            else:
                self.sub_remote.subscribe(self.callback)
            node.get_logger().info(f"Subscribed to: {self.topic['topic']}")
        except Exception as e:
            self.get_logger().error(str(e))
    
    # When info from mir, then publish locally
    def callback(self, msg):
        if not rclpy.ok():
            return

        # Thank you MiR. Very cool
        ros2_msg = self.converter(msg)

        # Filter out map->odom transform (doesn't clash with slam)
        if isinstance(ros2_msg, TFMessage):
            ros2_msg.transforms = [
                t for t in ros2_msg.transforms if not (t.header.frame_id == "map" and t.child_frame_id == "odom")
            ]
            if not ros2_msg.transforms:
                return
            for t in ros2_msg.transforms:
                self.tf_broadcaster.sendTransform(t)
        else:
            self.pub_local.publish(ros2_msg)
        return

       
    # Thank you /tf_static. Very cool
    def tf_static_callback(self, msg):
        if not rclpy.ok():
            return
        
        # Convert
        ros2_msg = self.converter(msg)

        # Filter out map->odom from MiR
        ros2_msg.transforms = [
            t for t in ros2_msg.transforms if not (t.header.frame_id == "map" and t.child_frame_id == "odom")
        ]
        if not ros2_msg.transforms:
            return
        
        # Extend transforms
        if self.last_static_msg is None:
            self.last_static_msg = ros2_msg    
        else:
            self.last_static_msg.transforms.extend(ros2_msg.transforms)
        
        # set a fixed timestamp for all transforms
        now = self.node.get_clock().now().to_msg()
        for t in self.last_static_msg.transforms:
            t.header.stamp = now
        
        # Publish (we know there are exactly eight transforms from mir)
        self.tf_static_broadcaster.sendTransform(self.last_static_msg.transforms)



# The bridge
class MiR_Bridge(Node):
    def __init__(self, rosbridge_host='192.168.12.20', rosbridge_port=9090):
        super().__init__('mir_bridge')
        
        try:
            # Variables
            self.target_topics = {}
            self.used_topics = {}
            self.topics_ready = False
            self.pubs = []
            self.subs = []

            # Connect to rosbridge
            try:
                self.client = roslibpy.Ros(host=rosbridge_host, port=rosbridge_port)
                self.client.run()
                self.get_logger().info('Succesfully connected to rosbridge.')
            except Exception as e:
                self.get_logger().error(str(e))
                return

            # Rosapi
            self.subs_service = roslibpy.Service(self.client, '/rosapi/subscribers', 'rosapi/Subscribers')
            self.pubs_service = roslibpy.Service(self.client, '/rosapi/publishers', 'rosapi/Publishers')
            
            # Read topic configuration folder, if it exists
            config_file = get_package_share_directory('mir_driver') + "/config/topic_config.yaml"
            try:
                with open(config_file, 'r') as f:
                    data = yaml.safe_load(f)
                    for target in data['topics']:
                        self.target_topics[target['topic']] = target['pub/sub']
                #self.get_logger().info(f'Targets found: {self.target_topics}')
            except Exception as e:
                self.get_logger().error(str(e))

            # Figure out which topics we have
            self.client.get_topics(self._on_topics)
        except KeyboardInterrupt:
            pass

    def _on_topics(self, topics):
        try:
            # Topics and types
            topics_arr = [topic for topic in topics["topics"]]
            types_arr = [type_var for type_var in topics["types"]]

            # Only use topics we want to use
            i = 0
            while i < len(topics_arr):
                if topics_arr[i] in self.target_topics.keys():
                    self.used_topics[topics_arr[i]] = ({"topic": topics_arr[i], "type": types_arr[i], "pub/sub": self.target_topics[topics_arr[i]]})
                    if self.used_topics[topics_arr[i]]["pub/sub"] == "sub":
                        self.subs.append(Subscriber(self.used_topics[topics_arr[i]], self, self.client))
                    else:
                        self.pubs.append(Publisher(self.used_topics[topics_arr[i]], self, self.client))
                i += 1
        except Exception as e:
            self.get_logger().error(str(e))



def main(args=None):
    rclpy.init(args=args)
    bridge = MiR_Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().warn("Shutdown called")
    except Exception as e:
        bridge.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        return
        
        


if __name__ == '__main__':
    main()
