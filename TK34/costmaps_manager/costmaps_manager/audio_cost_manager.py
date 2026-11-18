#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Header
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, Point
from fortis_interfaces.msg import MapCost, MapUpdateCost
import numpy as np
import math

class AudioCostManager(Node):
    def __init__(self):
        super().__init__('audio_cost_manager')

        # PARAMETERS (match costmap)
        self.width = None
        self.height = None
        self.resolution = None
        self.publish_rate = 5 # Hz    (was 0.1)
        self.size = None
        self.origin = None
        self.global_costmap = None
        self.path = None
        self.map_update = True
        self.min_cost = 100
        self.max_cost = 250


        # More placeholders
        self.gaussians = [
            {
                "mean": (800, 500),
                "covariance": [[5, 0], [0, 5]],
                "weight": 0
            },
            {
                "mean": (480, 300),
                "covariance": [[10, 0], [0, 10]],
                "weight": 1.2
            },
            {
                "mean": (350, 500),
                "covariance": [[2, 0], [0, 2]],
                "weight": 1.2
            },
            {
                "mean": (800, 150),
                "covariance": [[10, 0], [0, 10]],
                "weight": 1.2
            }
        ]


        # Quality of Service
        qos_init = QoSProfile(    # The initial map needs to stay available for later robots
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )


        # Publishers
        self.init_audio_cost_publisher_ = self.create_publisher(MapCost, '/global_costmap/global_costmap/init_audio_costs', qos_init)
        self.audio_cost_update_publisher_ = self.create_publisher(MapCost, 'global_costmap/global_costmap/audio_costs', 1)
        self.audio_grid_publisher_ = self.create_publisher(OccupancyGrid, '/costmaps_manager/audio_costs', 1)

        # Subscribers
        self.map_subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback,1)
        self.point_subscription = self.create_subscription(Point, '/clicked_point', self.point_callback,1)
        self.plan_subscription = self.create_subscription(Path, '/plan', self.plan_callback,1)
        self.map_update_subscription = self.create_subscription(MapUpdateCost, '/placeholder_audio_update', self.update_callback, 1)

        # Timers
        self.timer = self.create_timer(1.0 / self.publish_rate, self.costmap_publisher)
        
        self.get_logger().info('Audio Costmap Manager is initilized')


    def costmap_callback(self, msg: OccupancyGrid):
        self.global_costmap = msg
        new_width = msg.info.width
        new_height = msg.info.height

        # Only react if size changes or first message
        if self.width != new_width or self.height != new_height:
            self.width = new_width
            self.height = new_height
            self.size = self.width*self.height
            self.resolution = msg.info.resolution
            self.origin = msg.info.origin

            self.get_logger().info(
                f'Global costmap size: '
                f'{self.width} x {self.height} cells | '
                f'resolution: {self.resolution:.3f} m | '
                f'origin: ({self.origin.position.x:.2f}, {self.origin.position.y:.2f})'
            )
    
    def point_callback(self, msg):
        self.get_logger().info('A point has been clicked!!')
        self.get_logger().info(msg)

    def plan_callback(self, msg:Path):
        self.path = msg
        if self.global_costmap:
            cost = compute_path_cost(self.path, self.global_costmap, self.resolution, self.origin)
            self.get_logger().info(f'A path hase been recieved with cost: {cost}')

    def update_callback(self, msg):
        ### PLACEHOLDER STARTS HERE ###
        data = msg.data
        mode = int(data[0])
        if mode == 0:
            try:
                gauss = self.gaussians.pop(int(data[1]))
            except IndexError:
                self.get_logger().info("Index out of range")
                return

            # Have to remove the cost
            bounding_box, costs = gaussian_costmap(
                mean=gauss["mean"],
                covariance=gauss["covariance"],
                width=self.width,
                height=self.height,
                max_cost=self.max_cost,
                min_cost=self.min_cost
            )
            costs = np.array(costs)
            costs[costs!=0] = 255
            costs = costs.tolist()
        elif mode == 1:
            gauss = {
                "mean": (int(msg.i), int(msg.j)),
                "covariance": [[data[1], 0], [0, data[2]]],
                "weight": data[3],
            }
            self.gaussians.append(gauss)

            # Then count the gaussian costmap for one gaussian
            bounding_box, costs = gaussian_costmap(
                mean=gauss["mean"],
                covariance=gauss["covariance"],
                width=self.width,
                height=self.height,
                max_cost=self.max_cost,
                min_cost=self.min_cost
            )

        # Rossified programming
        msg_cost = MapCost()
        msg_cost.header = Header()
        msg_cost.header.stamp = self.get_clock().now().to_msg()
        msg_cost.data = costs
        msg_cost.i = float(bounding_box[0])
        msg_cost.j = float(bounding_box[1])
        msg_cost.width = float(bounding_box[2])
        msg_cost.height = float(bounding_box[3])
        self.audio_cost_update_publisher_.publish(msg_cost)
        self.get_logger().info("Local update calculated")
        ### PLACEHOLDER ENDS HERE ###

        self.map_update = True

    def costmap_publisher(self):
        if not self.map_update or not (self.width and self.height):
            return
        self.map_update = False
        
        msg_cost = MapCost()
        #Nav2 possible cost values
        #0      :        Completely free. Fully traversible
        #1–252  :        Increasing traversal cost
        #253    :        Inside robot’s inscribed radius (Practically forbidden)
        #254    :        Collision guaranteed
        #255    :        Unknown

        gaussians_1 = [
            {
                "mean": (800, 500),
                "covariance": [[5, 0], [0, 5]],
                "weight": 0.0
            },
            {
                "mean": (480, 300),
                "covariance": [[10, 0], [0, 10]],
                "weight": 0.0
            },
            {
                "mean": (350, 500),
                "covariance": [[2, 0], [0, 2]],
                "weight": 0.0
            },
            {
                "mean": (800, 150),
                "covariance": [[10, 0], [0, 10]],
                "weight": 0.0
            }
        ]

        costs = multi_gaussian_costmap(
            gaussians=self.gaussians,
            width=self.width,
            height=self.height,
            max_cost=self.max_cost,
            min_cost=self.min_cost
        )

        msg_cost.header = Header()
        msg_cost.header.stamp = self.get_clock().now().to_msg()
        msg_cost.data = costs
        self.init_audio_cost_publisher_.publish(msg_cost)

        self.get_logger().info("Global update calculated")
        
        #msg_grid = OccupancyGrid()

        # Header
        #msg_grid.header = Header()
        #msg_grid.header.frame_id = "map"

        # Map metadata
        #msg_grid.info.width = self.width
        #msg_grid.info.height = self.height
        #msg_grid.info.resolution = self.resolution

        #msg_grid.info.origin= self.origin
        #msg_grid.info.origin.orientation.w = self.origin.orientation.w  # no rotation

        
        #data = []

        #for c in costs:
        #    if c == 255:
        #        data.append(-1)  # unknown
        #    else:
        #        occ = int((c / 252.0) * 100.0)
        #        data.append(max(0, min(100, occ)))

        #msg_grid.data = data
        #self.audio_grid_publisher_.publish(msg_grid)


def gaussian_costmap(mean, covariance, width, height, max_cost=252, min_cost=0):
    """
    Generate a Gaussian-distributed costmap with a minimum cost floor.

    Parameters
    ----------
    mean : tuple (mx, my)
        Mean of the Gaussian in grid coordinates (cells).
    covariance : 2x2 array-like
        Covariance matrix.
    width : int
        Costmap width (X dimension).
    height : int
        Costmap height (Y dimension).
    max_cost : int
        Maximum cost value (Nav2: <= 252).
    min_cost : int
        Minimum cost for non-zero values.

    Returns
    -------
    costs : list of uint8
        Flattened costmap (row-major, ROS-compatible).
    """

    mx, my = mean
    cov = np.array(covariance, dtype=float)
    inv_cov = np.linalg.inv(cov)
    det_cov = np.linalg.det(cov)

    norm = 1.0 / (2.0 * np.pi * np.sqrt(det_cov))

    costs = np.zeros(width * height, dtype=np.uint8)
    values = np.zeros((height, width), dtype=float)
    max_val = 0.0

    # Compute Gaussian values
    for j in range(height):
        for i in range(width):
            d = np.array([i - mx, j - my])
            val = norm * np.exp(-0.5 * d.T @ inv_cov @ d)
            values[j, i] = val
            max_val = max(max_val, val)

    # Normalize to [0, 1]
    if max_val > 0.0:
        values /= max_val

    # Bounding box
    rows, cols = np.where(values != 0)
    box = [cols.min(), rows.min(), cols.max()-cols.min(), rows.max()-rows.min()]
    values = values[rows.min():rows.max(), cols.min():cols.max()]

    # God I love matlab
    costs = values.flatten()
    costs[costs>0] = min_cost + costs[costs>0] * (max_cost - min_cost)
    costs = costs.astype(np.uint8)

    # Scale to [min_cost, max_cost]
    #for j in range(height):
    #    for i in range(width):
    #        index = i + j * width
    #        v = values[j, i]

    #        if v <= 0.0:
    #            costs[index] = 0
    #        else:
    #            costs[index] = int(
    #                min_cost + v * (max_cost - min_cost)
    #            )

    return box, costs.tolist()

def multi_gaussian_costmap(gaussians, width, height, max_cost=252, min_cost=0):
    """
    Generate a costmap from multiple Gaussian distributions.

    Parameters
    ----------
    gaussians : list of dicts
        Each dict must contain:
        {
            "mean": (mx, my),
            "covariance": [[cxx, cxy], [cyx, cyy]],
            "weight": float (optional, default=1.0)
        }
    width : int
    height : int
    max_cost : int
    min_cost : int

    Returns
    -------
    costs : list of uint8
        Flattened costmap (row-major)
    """

    values = np.zeros((height, width), dtype=float)

    for g in gaussians:
        mx, my = g["mean"]
        cov = np.array(g["covariance"], dtype=float)
        weight = g.get("weight", 1.0)

        inv_cov = np.linalg.inv(cov)
        det_cov = np.linalg.det(cov)
        norm = 1.0 / (2.0 * np.pi * np.sqrt(det_cov))

        for j in range(height):
            for i in range(width):
                d = np.array([i - mx, j - my])
                val = norm * np.exp(-0.5 * d.T @ inv_cov @ d)
                values[j, i] += weight * val

    # Normalize to [0, 1]
    max_val = np.max(values)
    if max_val > 0.0:
        values /= max_val

    # Convert to Nav2 costs
    costs = np.zeros(width * height, dtype=np.uint8)
    for j in range(height):
        for i in range(width):
            index = i + j * width
            v = values[j, i]

            if v <= 0.0:
                costs[index] = 0
            else:
                costs[index] = int(
                    min_cost + v * (max_cost - min_cost)
                )

    return costs.tolist()

def compute_path_cost(path, costmap, resolution, origin):
    """
    path: nav_msgs/Path
    costmap: flattened uint8 array (row-major)
    resolution: meters/cell
    origin: geometry_msgs/Pose
    """

    width = costmap.info.width
    height = costmap.info.height
    data = costmap.data

    total_cost = 0.0
    prev_pose = None

    for pose_stamped in path.poses:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y

        mx = int((x - origin.position.x) / resolution)
        my = int((y - origin.position.y) / resolution)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            continue

        index = mx + my * width
        cell_cost = data[index]

        if cell_cost >= 253:
            return float("inf")  # lethal or unknown

        if prev_pose is not None:
            dx = x - prev_pose.position.x
            dy = y - prev_pose.position.y
            step_dist = math.hypot(dx, dy)
            total_cost += cell_cost * step_dist

        prev_pose = pose_stamped.pose

    return total_cost

def main(args=None):
    rclpy.init(args=args)
    node = AudioCostManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutdown called")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()