#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class AudioCostManager(Node):
    def __init__(self):
        super().__init__('audio_cost_manager')

        # PARAMETERS (match costmap)
        self.width = None
        self.height = None
        self.resolution = None
        self.publish_rate = 0.2 # Hz
        self.size = None
        self.origin = None

        # Publishers
        self.audio_cost_publisher_ = self.create_publisher(UInt8MultiArray, '/global_costmap/global_costmap/audio_costs', 1)
        self.audio_grid_publisher_ = self.create_publisher(OccupancyGrid, '/costmaps_manager/audio_costs', 1)

        # Subscribers
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback,1)

        # Timers
        self.timer = self.create_timer(1.0 / self.publish_rate, self.costs_publisher)
        
        self.get_logger().info('Audio Costmap Manager is initilized')

    def costmap_callback(self, msg: OccupancyGrid):
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

    def costs_publisher(self):
        if not (self.width and self.height):
            return
        
        msg_cost = UInt8MultiArray()
        #Nav2 possible cost values
        #0      :        Completely freeFully traversable
        #1–252  :        Increasing traversal cost
        #253    :        Inside robot’s inscribed radius (Practically forbidden)
        #254    :        Collision guaranteed
        #255    :        Unknown
        
        mean = (250, 1000) # width , height
        mean = (550, 1000) # width , height
        covariance = [[300, 0],
                    [0, 500]]

        costs = gaussian_costmap(
            mean=mean,
            covariance=covariance,
            width=self.width,
            height=self.height,
            max_cost=200
        )
    
        msg_cost.data = costs
        self.audio_cost_publisher_.publish(msg_cost)

        
        msg_grid = OccupancyGrid()

        # Header
        msg_grid.header = Header()
        msg_grid.header.frame_id = "map"

        # Map metadata
        msg_grid.info.width = self.width
        msg_grid.info.height = self.height
        msg_grid.info.resolution = self.resolution

        msg_grid.info.origin= self.origin
        msg_grid.info.origin.orientation.w = self.origin.orientation.w  # no rotation

        
        data = []

        for c in costs:
            if c == 255:
                data.append(-1)  # unknown
            else:
                occ = int((c / 252.0) * 100.0)
                data.append(max(0, min(100, occ)))

        msg_grid.data = data
        self.audio_grid_publisher_.publish(msg_grid)
    
def gaussian_costmap(mean, covariance, width, height, max_cost=252):
    """
    Generate a Gaussian-distributed costmap.

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

    Returns
    -------
    costs : list of uint8
        Flattened costmap (row-major, ROS-compatible).
    """

    mx, my = mean
    cov = np.array(covariance, dtype=float)
    inv_cov = np.linalg.inv(cov)
    det_cov = np.linalg.det(cov)

    # Normalization factor (not strictly required, but correct)
    norm = 1.0 / (2.0 * np.pi * np.sqrt(det_cov))

    costs = np.zeros(width * height, dtype=np.uint8)

    max_val = 0.0
    values = np.zeros((height, width), dtype=float)

    # Compute Gaussian values
    for j in range(height):
        for i in range(width):
            d = np.array([i - mx, j - my])
            val = norm * np.exp(-0.5 * d.T @ inv_cov @ d)
            values[j, i] = val
            max_val = max(max_val, val)

    # Normalize and scale to cost values
    if max_val > 0.0:
        values = values / max_val

    for j in range(height):
        for i in range(width):
            index = i + j * width
            costs[index] = int(values[j, i] * max_cost)

    return costs.tolist()


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