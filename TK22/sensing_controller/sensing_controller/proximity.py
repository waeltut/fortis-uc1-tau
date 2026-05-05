import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs import do_transform_cloud
from sensor_msgs_py import point_cloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from fortis_interfaces.msg import Proximity as prx
from builtin_interfaces.msg import Time
from rclpy import qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import numpy as np


# Coordinate transform
def polar_2_cartesian(angles, ranges, lidar_name, lidar_offsets):
    MIN_VALID_DISTANCE = 0.1  # meters (ignore noise/nearby clutter)
    
    # Compute x, y, z
    angles = angles
    ranges = ranges
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    z = np.zeros_like(x)

    # Turn into pointcloud
    points = np.vstack([x, y, z]).T
    header = Header()
    header.frame_id = lidar_name
    pc = point_cloud2.create_cloud_xyz32(header, points.tolist())

    # Transform to base_link
    transformed_pc = do_transform_cloud(pc, lidar_offsets[lidar_name])
    # Convert transformed PointCloud2 -> Nx3 NumPy array
    points = list(pc2.read_points(transformed_pc, field_names=("x", "y", "z"), skip_nans=True))
    x_coords = np.array([p[0] for p in points])
    y_coords = np.array([p[1] for p in points])
    #zs = [p[2] for p in points]
    #x_coords = transformed_points[:, 0]
    #y_coords = transformed_points[:, 1]

    # Stuff
    valid_mask = ranges > MIN_VALID_DISTANCE
    return x_coords[valid_mask], y_coords[valid_mask]

# Coordinate transform (remember to change)
def cartesian_2_polar(x, y):
    # MiR centered around the origin    # Change these
    LENGTH = 0.901900*2
    WIDTH = 0.375000*2
    xmin, xmax = -LENGTH/2, LENGTH/2
    ymin, ymax = -WIDTH/2, WIDTH/2
    
    # Prevent divisions by zero
    EPS = 1e-12
    x_safe = np.where(x == 0, EPS, x)
    y_safe = np.where(y == 0, EPS, y)
    
    # Intersection with vertical side (xmin, xmax)
    t_xmin = xmin / x_safe
    t_xmax = xmax / x_safe
    y_at_xmin = t_xmin * y
    y_at_xmax = t_xmax * y
    valid_xmin = (t_xmin > 0) & (y_at_xmin >= ymin) & (y_at_xmin <= ymax)
    valid_xmax = (t_xmax > 0) & (y_at_xmax >= ymin) & (y_at_xmax <= ymax)
    
    # Intersection with horizontal side (ymin, ymax)
    t_ymin = ymin / y_safe
    t_ymax = ymax / y_safe
    x_at_ymin = t_ymin * x
    x_at_ymax = t_ymax * x
    valid_ymin = (t_ymin > 0) & (x_at_ymin >= xmin) & (x_at_ymin <= xmax)
    valid_ymax = (t_ymax > 0) & (x_at_ymax >= xmin) & (x_at_ymax <= xmax)

    # Collect valid intersection ratios
    t_candidates = np.minimum.reduce([
        np.where(valid_xmin, t_xmin, np.inf),
        np.where(valid_xmax, t_xmax, np.inf),
        np.where(valid_ymin, t_ymin, np.inf),
        np.where(valid_ymax, t_ymax, np.inf)
    ])
    
    # Compute ranges
    zero_ranges = np.linalg.norm(np.column_stack((x, y)), axis=1)
    exit_range = t_candidates * np.linalg.norm(np.column_stack((x, y)), axis=1)
    ranges = zero_ranges - exit_range
    
    # Compute angle
    mask = ranges>0.1
    angles = np.arctan2(y, x)
    return ranges[mask], zero_ranges[mask], angles[mask]

# Bins lidar data to make it compatible with laser scan
def lidar_combiner(r, a, da, min_r, max_r):
    # Sort both
    p = a.argsort()
    a = a[p]
    r = r[p]

    # Constants
    bin_diff = da
    min = -np.pi
    max = np.pi
    diff = max-min

    # Bin splitting
    binsN = int(diff / bin_diff) + 1
    bins = np.linspace(min, max, binsN)
    r_binned = np.zeros(bins.shape)
    a_binned = bins
    inds = np.digitize(a,bins)

    # Loopy
    for i in range(len(bins)):
        # Get the entire bin
        mask = inds==i
        r_mask = r[mask]

        # Check if bin emtpy
        if len(r_mask) < 1:
            r_binned[i] = float("nan")
            continue
        
        # if not empty, get the closest range
        ind_min = np.argmin(r_mask)
        ind_min = ind_min if isinstance(ind_min, np.int64) else ind_min[0]
        r_binned[i] = r_mask[ind_min] if r_mask[ind_min] >= min_r and r_mask[ind_min] <= max_r else float("nan")
    return r_binned, a_binned

class Proximity(Node):
    def __init__(self):
        # Initialization
        super().__init__('proximity')
        self.angles = {"lidar_f": [],
                       "lidar_b": []}
        self.ranges = {"lidar_f": [],
                       "lidar_b": []}
        self.min_angle = 0
        self.min_dist = 10
        self.last_log = self.get_clock().now()
        self.log_interval = Duration(seconds=10)
        self.publish_rate = 100
        self.last_publish_time = 0

        # Messages
        self.prox_msg = prx()
        self.prox_msg.header = Header()
        self.prox_msg.header.frame_id = "base_footprint"
        self.comp_lidar_msg = LaserScan()
        self.comp_lidar_msg.header = Header()
        self.comp_lidar_msg.header.frame_id = "base_footprint"

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.f_scan_transform = None
        self.b_scan_transform = None
        self.offsets = {
            "front_laser_link": None,
            "back_laser_link": None,
        }
        self.offsets = {}

        # Ros
        self.proximity_pub = self.create_publisher(prx, "/steven/sensing/proximity", 1)
        self.combined_pub = self.create_publisher(LaserScan, "/steven/sensing/scan_composite", 1)
        self.create_subscription(LaserScan, "/f_scan", lambda msg, laser="lidar_f": self.lidar_callback(msg, laser), 1)
        self.create_subscription(LaserScan, "/b_scan", lambda msg, laser="lidar_b": self.lidar_callback(msg, laser), 1)

        self.get_logger().info("Node started. Listening to lidar topics...")
        self.timer = self.create_timer(0.1, self.proximity_worker)
    

    def get_transforms(self, frame, base_frame="base_link"):
        try:
            t = self.tf_buffer.lookup_transform(base_frame,
                    frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0)
            )
            
            self.offsets[frame] = t
            
        except Exception as e:
            self.get_logger().warn(f"Could not get transform for {frame}: {e}")


    def lidar_callback(self, message, laser):
        # Get intrinsics
        angle_min = message.angle_min
        angle_max = message.angle_max
        #angle_increment = message.angle_increment
        num_readings = len(message.ranges)
        
        # Calculate angles and ranges
        self.angles[laser] = np.linspace(angle_min, angle_max, num_readings)    # Rotated for alignment
        self.ranges[laser] = np.array(message.ranges)

        # TODO: Error handling :)
    

    def proximity_worker(self):
        # Fetch LIDAR inputs
        lidar_f_angle = self.angles["lidar_f"]
        lidar_f_range = self.ranges["lidar_f"]
        lidar_b_angle = self.angles["lidar_b"]
        lidar_b_range = self.ranges["lidar_b"]

        # If no scans yet, then skip
        if len(lidar_f_angle)==0 or len(lidar_f_range)==0 or len(lidar_b_angle)==0 or len(lidar_b_range)==0:
            return
        
        # Combine lidars (Polar -> Cartesian)
        if "front_laser_link" not in self.offsets:
            self.get_transforms("front_laser_link")
            return
        else:
            #self.get_logger().info(f"{self.offsets['front_laser_link']}")
            x1, y1 = polar_2_cartesian(lidar_f_angle, lidar_f_range, "front_laser_link", self.offsets)
        if "back_laser_link" not in self.offsets:
            self.get_transforms("back_laser_link")
            return
        else:
            #self.get_logger().info(f"{self.offsets['back_laser_link']}")
            x2, y2 = polar_2_cartesian(lidar_b_angle, lidar_b_range, "back_laser_link", self.offsets)
        x = np.concatenate((x1, x2))
        y = np.concatenate((y1, y2))

        # Convert back (Cartesian -> Polar)
        ranges, zero_ranges, angles = cartesian_2_polar(x, y)    # In meters and radians

        # Find closest object
        min_index = int(np.argmin(ranges[~np.isnan(ranges)]))
        #self.get_logger().info(f"{min(ranges)}")
        self.min_angle = angles[min_index]
        self.min_dist = ranges[min_index]

        # Publish data
        self.prox_msg.angle = float(self.min_angle)
        self.prox_msg.distance = float(self.min_dist)
        self.prox_msg.header.stamp = self.get_clock().now().to_msg()
        self.proximity_pub.publish(self.prox_msg)

        # Get the composite lidar
        r = zero_ranges
        a = angles
        da = 0.007
        max_r = 29.0                    # From the docs
        min_r = 0.05000000074505806     # From the docs
        rb, ab = lidar_combiner(r, a, da, min_r, max_r)     # May produce NaN

        # Publish composite lidar
        self.comp_lidar_msg.angle_increment = da
        self.comp_lidar_msg.angle_min = np.min(ab)
        self.comp_lidar_msg.angle_max = np.max(ab)
        self.comp_lidar_msg.ranges = [float(x) for x in rb.flatten()]
        self.comp_lidar_msg.range_max = float(max_r)
        self.comp_lidar_msg.range_min = float(min_r)
        self.comp_lidar_msg.intensities = []
        self.comp_lidar_msg.time_increment = 5.545286330743693e-05
        self.comp_lidar_msg.scan_time = 0.1
        self.comp_lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self.combined_pub.publish(self.comp_lidar_msg)

        # Periodic logging
        if self.get_clock().now() - self.last_log > self.log_interval:
            dist_m = self.min_dist
            angle_deg = 180 * self.min_angle / math.pi
            self.get_logger().info(f"[PROXIMITY] Closest object: {dist_m:0,.2f} m, angle: {angle_deg:0,.2f}°")
            self.last_log = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    prox = Proximity()
    try:
        rclpy.spin(prox)
    except KeyboardInterrupt:
        prox.get_logger().warn("Shutdown called")
    #except Exception as e:
    #    prox.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        print("Done")

if __name__ == '__main__':
    main()