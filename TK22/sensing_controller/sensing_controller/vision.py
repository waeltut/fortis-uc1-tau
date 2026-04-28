import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy import qos
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from fortis_interfaces.msg import Vision as vsn
from fortis_interfaces.msg import VisionObs as vsn_obs
from ultralytics import YOLO
import math
import numpy as np
import cv2
import os
import torch

class Vision(Node):
    def __init__(self, record):
        # Initialization
        super().__init__('vision')
        self.record = record
        self.motivated = False

        # Cameras
        self.color_info = []
        self.color_frame = []
        self.color_stamp = Time(clock_type=self.get_clock().clock_type)
        self.depth_info = []
        self.depth_frame = []
        self.depth_stamp = Time(clock_type=self.get_clock().clock_type)

        # Logging
        self.frame_skip = 3
        self.frame_n = 0
        self.last_log = Time(clock_type=self.get_clock().clock_type)
        self.log_interval = Duration(seconds=10)
        self.start_log = self.get_clock().now()
        self.end_log = self.get_clock().now()

        # Error handling
        self.last_distance = 7.0
        self.last_angle = 0.0
        self.last_seen = Time(clock_type=self.get_clock().clock_type)
        self.time_tolerance = Duration(seconds=10)

        # ArUco setup
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.marker_size = 0.07
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # Model    Figure those nvidia-things out
        pkg_path = get_package_share_directory('sensing_controller')
        model_path = os.path.join(pkg_path, 'models', 'yolov8n.pt')
        #engine_path = os.path.join(pkg_path, 'models')
        self.model = YOLO(model_path)
        self.model.fuse()
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, classes=[0], device='cuda', verbose=False)

        # Recording
        if self.record:
            # I really need to not hardcode camera constants
            video_path = "/home/txpela/Misc/Fortis_demos/demo_video.mp4"
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter(video_path, self.fourcc, 10.0, (848, 480))

        # Messages
        self.vsn_message = vsn()
        self.vsn_message.header = Header()
        self.obs_message = vsn_obs()
        self.obs_message.header = Header()

        # Ros
        self.get_logger().info("Node started. Listening to camera topics...")
        self.vision_pub = self.create_publisher(vsn, "/steven/sensing/vision", 1)
        self.obs_pub = self.create_publisher(vsn_obs, "/steven/sensing/vision_obs", 1)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.color_info_callback, 1)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info", self.depth_info_callback, 1)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_callback, 1)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 1)
        self.create_subscription(String, "/mot_mode", self.mode_callback, 1)
        #self.create_timer(0.10, self.spinner)

        # Optimization testing
        self.last_loop = self.get_clock().now()
        self.get_logger().info(f"{torch.version.cuda}")
        if torch.cuda.is_available():
            n_gpus = torch.cuda.device_count()
            for i in range(n_gpus):
                self.get_logger().info(f"GPU {i}: {torch.cuda.get_device_name(i)}")
                self.get_logger().info(f"  Memory Allocated: {torch.cuda.memory_allocated(i)/1024**2:.2f} MB")
                self.get_logger().info(f"  Memory Reserved:  {torch.cuda.memory_reserved(i)/1024**2:.2f} MB")
        else:
            self.get_logger().info("No CUDA GPU detected")


    def spinner(self):
        # Check if image has arrived
        if ((self.get_clock().now() - self.color_stamp) > self.time_tolerance) or ((self.get_clock().now() - self.depth_stamp) > self.time_tolerance):
            if self.get_clock().now() - self.last_log > self.log_interval:
                self.get_logger().warn("Image did not arrive within tolerance")
                self.last_log = self.get_clock().now()
            #return

        # Run inference
        results = self.model.predict(self.color_frame, conf=0.40, device='cuda', verbose=False)
        annotated = results[0].plot()

        # Camera params
        fx = self.color_info[0]
        fy = self.color_info[4]
        cx_intr = self.color_info[2]
        cy_intr = self.color_info[5]
        camera_matrix = np.array([[fx, 0, cx_intr],
                                        [0, fy, cy_intr],
                                        [0, 0, 1]])
        dist_coeffs = np.zeros([1,5])

        # Find closest bounding box
        min_distance = 10000.0
        min_cx, min_cy = 0, 0
        for box in results[0].boxes.xyxy:
            x1, y1, x2, y2 = map(int, box[:4])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            distance_m = self.get_depth(self.depth_frame/1000, cx, cy)
            if distance_m < min_distance:
                min_distance = distance_m
                min_cx, min_cy = cx, cy
        
        # Angle calculation
        if len(results[0].boxes) > 0:
            angle = -math.atan2(min_cx - cx_intr, fx)

        # Fallback for invalid distance
        if min_distance == 0 or min_distance == 10000:
            min_distance = self.last_distance
        else:
            self.last_distance = min_distance

        # Convert color image to grayscale for detection
        gray = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        marker_points = np.array([
            [-self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        # More marker stuff
        if len(corners) > 0:
            object_points = []
            for i in range(0, len(ids)):
            
                success, rvec, tvec = cv2.solvePnP(marker_points, corners[i], camera_matrix, dist_coeffs)
                #cv2.aruco.drawDetectedMarkers(annotated, corners)

                # Depth unit vector
                u = int(corners[i][0][:,0].mean())
                v = int(corners[i][0][:,1].mean())
                Z = self.depth_frame[v, u] / 1000
                X = (u - cx_intr) * Z / fx
                Y = (v - cy_intr) * Z / fy
                depth_meters = np.sqrt(X**2 + Y**2 + Z**2)

                # We want to use the more accurate depth value
                tvec = tvec.flatten()
                tvec_unit = tvec / np.linalg.norm(tvec)
                tvec_mag = tvec_unit * depth_meters if depth_meters > 0 else tvec_unit * np.linalg.norm(tvec)

                # Offset
                R, _ = cv2.Rodrigues(rvec)
                offset_local = np.array([0, 0, -0.17])
                offset_camera = R @ offset_local
                
                # Vector calculus
                tvec_act = tvec_mag + offset_camera
                object_points.append(tvec_act)
                #annotated[v, u] = (255, 255, 255)

            # The average center
            object_points = np.array(object_points)
            object_center = np.mean(object_points, axis=0)
            #cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, object_center, 0.035)

            # Send message
            self.obs_message.header.stamp = self.get_clock().now().to_msg()
            self.obs_message.angle = float(math.atan2(object_center[0], object_center[2]))
            self.obs_message.visible = True
            self.obs_message.distance = np.linalg.norm(object_center)
            self.obs_message.shape = 0
        else:
            self.obs_message.header.stamp = self.get_clock().now().to_msg()
            self.obs_message.angle = 0.0
            self.obs_message.visible = False
            self.obs_message.distance = 0.0
            self.obs_message.shape = -1
        
        # Publishing
        if len(results[0].boxes) > 0:
            self.vsn_message.angle = float(angle)
            self.vsn_message.distance = float(min_distance)
            self.vsn_message.visible = True
            self.vsn_message.last_seen = 0.0
            self.last_angle = float(angle)
            self.last_seen = self.get_clock().now()
        else:
            self.vsn_message.angle = self.last_angle
            self.vsn_message.distance = float(min_distance)
            self.vsn_message.visible = False
            self.vsn_message.last_seen = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        self.vsn_message.header.stamp = self.get_clock().now().to_msg()
        self.vision_pub.publish(self.vsn_message)
        self.obs_pub.publish(self.obs_message)

        # Logging
        if len(results[0].boxes) > 0 and (self.get_clock().now() - self.last_log > self.log_interval):
            # Testing (thank you c++)
            #tf = 0.45
            #d = math.sqrt(self.vsn_message.distance**2 + tf*tf - 2*self.vsn_message.distance*tf*math.cos(math.pi - abs(self.vsn_message.angle)))
            #cos_alpha = (self.vsn_message.distance**2 + tf*tf - d*d) / (2*self.vsn_message.distance*tf)
            #cos_alpha = min(max(cos_alpha, -1.0), 1.0)    # Just in case
            #alpha = math.pi - math.acos(cos_alpha)

            self.get_logger().info(f"Detected {len(results[0].boxes)} person(s)")
            self.get_logger().info(f"Closest person: {min_distance:.2f} m, angle: {180 * angle / math.pi:.2f}°")
            #self.get_logger().info(f"Closest person from base_link: {d:.2f} m, angle: {180 * alpha / math.pi:.2f}°")
            self.last_log = self.get_clock().now()

            # Optimization testing
            self.get_logger().info(f"Execution time: {(self.get_clock().now() - self.last_loop).nanoseconds/1e9}")

        # Show the color-frame
        #if self.record and self.motivated:
        #    self.out.write(annotated)

        # This is slow, so let's skip frames
        #if self.frame_n >= self.frame_skip:
        #    cv2.imshow("Vision", annotated)
        #    cv2.waitKey(1)
        #    self.frame_n = 0
        #self.frame_n += 1

        self.last_loop = self.get_clock().now()


    ## Callbacks ##
    def depth_info_callback(self, msg):
        self.depth_info = msg.k
    
    def color_info_callback(self, msg):
        self.color_info = msg.k

    def depth_callback(self, msg):
        self.depth_frame = self.ros_to_cv2(msg)
        self.depth_stamp = Time(seconds=msg.header.stamp.sec,
                                nanoseconds=msg.header.stamp.nanosec,
                                clock_type=self.get_clock().clock_type)
    
    def color_callback(self, msg):
        self.color_frame = self.ros_to_cv2(msg)
        self.color_stamp = Time(seconds=msg.header.stamp.sec,
                                nanoseconds=msg.header.stamp.nanosec,
                                clock_type=self.get_clock().clock_type)
        self.spinner()
    
    def mode_callback(self, msg):
        if msg.data == "automatic":
            self.motivated = True
            self.start_log = self.get_clock().now()
            self.get_logger().info(f"Logging started at {self.start_log.to_sec()}")
        else:
            self.motivated = False
            self.end_log = self.get_clock().now()
            self.get_logger().info(f"Logging ended at {self.end_log.to_sec()}")
            self.get_logger().info(f"Total recording time: {(self.end_log - self.start_log).to_sec()}")

    
    ## Convenience ##
    def ros_to_cv2(self, msg):
        if msg.encoding == "rgb8":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "16UC1":
            img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding == "8UC1":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
            return None
        img = img[::-1, ::-1]
        return img

    def get_depth(self, depth_frame, cx, cy, size=25):
        # Define the bounds of the square patch, clamped to the image edges
        x1 = max(cx - size // 2, 0)
        y1 = max(cy - size // 2, 0)
        x2 = min(cx + size // 2 + 1, depth_frame.shape[1])
        y2 = min(cy + size // 2 + 1, depth_frame.shape[0])

        # Extract the patch from the depth frame
        patch = depth_frame[y1:y2, x1:x2]

        # Filter out invalid values (zeros)
        valid_depths = patch[patch > 0]

        # Return 0 if no valid depths, otherwise return the minimum
        if valid_depths.size == 0:
            return 0
        return np.min(valid_depths)



def main(args=None):
    rclpy.init(args=args)
    record = False
    vision = Vision(record)
    try:
        rclpy.spin(vision)
    except KeyboardInterrupt:
        vision.get_logger().warn("Shutdown called")
    #except Exception as e:
    #    vision.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        if record and (vision.out is not None):
            vision.out.release()

if __name__ == '__main__':
    main()