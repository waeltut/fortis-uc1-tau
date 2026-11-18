import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
# from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from std_msgs.msg import Header, String, Float32, Float64
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from fortis_interfaces.msg import Vision as vsn
from fortis_interfaces.msg import VisionObs as vsn_obs
from cv_bridge import CvBridge

import cv2
import mediapipe as mp
import numpy as np
import math
import os
import json
import torch
from collections import deque, Counter
from abc import ABC, abstractmethod
from ultralytics import YOLO


# --- INTENSITY CALCULATION ---
def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    try:
        if is_log_scale:
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return float(k_factor * diff if diff > 0 else np.finfo(float).eps)
        else:
            strength_safe = np.maximum(physical_strength, 0.001)
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold
            return float(k_factor * np.log(ratio) if ratio > 1.0 else np.finfo(float).eps)
    except Exception:
        return np.finfo(float).eps

# --- VISUAL DESCRIPTORS ---
class VisualDescriptor(ABC):
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        pass

    def _brightness(self, img):
        if img is None or img.size == 0:
            return 0.0
        if len(img.shape) == 3:
            return float(np.average(np.linalg.norm(img, axis=2)) / np.sqrt(3))
        return float(np.average(img))

    def _calculate_contrast(self, roi_gray):
        if roi_gray is None or roi_gray.size == 0:
            return 0.0
        return float(roi_gray.std())

class LightDescriptor(VisualDescriptor):
    def __init__(self):
        super().__init__("visual_light")

    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        return self._brightness(image_bgr)

class ContrastDescriptor(VisualDescriptor):
    def __init__(self):
        super().__init__("visual_contrast")

    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        return self._calculate_contrast(hand_roi_gray)

class SpeedDescriptor(VisualDescriptor):
    def __init__(self):
        super().__init__("visual_speed")
        self.prev_coord = None

    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        if hand_lms:
            landmark = hand_lms.landmark[4]  # Thumb tip landmark
            if self.prev_coord is None:
                self.prev_coord = [landmark.x, landmark.y, landmark.z]
                return 0.0
            current_speed = float(np.linalg.norm([
                landmark.x - self.prev_coord[0],
                landmark.y - self.prev_coord[1],
                landmark.z - self.prev_coord[2]
            ]))
            self.prev_coord = [landmark.x, landmark.y, landmark.z]
            return current_speed
        else:
            self.prev_coord = None
            return 0.0


class AdvancedGestureRecognizer:
    def __init__(self):
        self.WRIST = 0
        self.TIPS = [4, 8, 12, 16, 20]
        self.PIPS = [2, 6, 10, 14, 18]
        self.MCPS = [1, 5, 9, 13, 17]
        
        self.movement_frames = 0
        self.silence_frames = 0
        self.gesture_buffer = []
        self.descriptor_during_gesture = []

        # New: Delimiter posture persistence counter
        self.stop_gesture_frames = 0
        self.STOP_GESTURE_THRESHOLD = 3

    def is_fist(self, landmarks):
        """Returns True if all 5 fingers are curled shut."""
        states = self._get_finger_states(landmarks)
        # states = [thumb, index, middle, ring, pinky]
        return sum(states) == 0

    def is_thumbs_down(self, landmarks):
        """Optional delimiter: thumb pointing downwards, other fingers curled."""
        states = self._get_finger_states(landmarks)
        thumb_tip = landmarks[4]
        thumb_mcp = landmarks[2]
        # Only thumb extended and pointing downward (tip Y > mcp Y)
        return states[0] and (sum(states[1:]) == 0) and (thumb_tip.y > thumb_mcp.y)
    
    def _get_finger_states(self, landmarks):
        """
        Calculates individual finger curl states.
        Returns: [thumb, index, middle, ring, pinky]
        """
        wrist = landmarks[self.WRIST]
        states = []

        # Thumb: compare tip (4) to pinky MCP base (17) vs IP joint (3)
        pinky_mcp = landmarks[17]
        d_tip_pinky = math.hypot(landmarks[4].x - pinky_mcp.x, landmarks[4].y - pinky_mcp.y)
        d_ip_pinky = math.hypot(landmarks[3].x - pinky_mcp.x, landmarks[3].y - pinky_mcp.y)
        states.append(d_tip_pinky > d_ip_pinky)

        # 4 Fingers: fingertip must be farther from both wrist and MCP than PIP
        for i in range(1, 5):
            tip = landmarks[self.TIPS[i]]
            pip = landmarks[self.PIPS[i]]
            mcp = landmarks[self.MCPS[i]]

            d_tip_wrist = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
            d_pip_wrist = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
            d_tip_mcp = math.hypot(tip.x - mcp.x, tip.y - mcp.y)
            d_pip_mcp = math.hypot(pip.x - mcp.x, pip.y - mcp.y)

            states.append((d_tip_wrist > d_pip_wrist) and (d_tip_mcp > d_pip_mcp))

        return states

    def _analyze_frame_posture(self, landmarks):
        """
        Returns (posture_type, pointing_direction)
        posture_type: 'pointing', 'open_hand', 'fist', 'other'
        """
        states = self._get_finger_states(landmarks)
        four_fingers_open = sum(states[1:])

        # 1. Pointing check: index extended, remaining 3 curled
        if states[1] and not states[2] and not states[3] and not states[4]:
            dx = landmarks[8].x - landmarks[5].x
            dy = landmarks[8].y - landmarks[5].y

            if abs(dx) > abs(dy):
                direction = "move_right" if dx > 0 else "move_left"
            else:
                direction = "move_down" if dy > 0 else "move_up"
            return "pointing", direction

        # 2. Open Palm (Stop or Swipe candidate): 4 or 5 fingers extended
        if four_fingers_open >= 4 or sum(states) >= 4:
            return "open_hand", "none"

        # 3. Fist
        if sum(states) == 0:
            return "fist_idle", "none"

        return "other", "none"

    def process_sequence(self, gesture_sequence, swipe_threshold=0.08):
        """
        Evaluates the sequence of landmarks collected during the gesture.
        gesture_sequence: list of MediaPipe landmark objects
        """
        if not gesture_sequence or len(gesture_sequence) < 1:
            return "none"

        postures = []
        pointing_votes = []

        for lms in gesture_sequence:
            posture, point_dir = self._analyze_frame_posture(lms.landmark)
            postures.append(posture)
            if point_dir != "none":
                pointing_votes.append(point_dir)

        dominant_posture = Counter(postures).most_common(1)[0][0]

        # Scenario 1: Pointing gesture -> emit dominant static direction
        if dominant_posture == "pointing" and pointing_votes:
            return Counter(pointing_votes).most_common(1)[0][0]

        # Scenario 2: Open hand -> check trajectory for Swipe vs Stop
        if dominant_posture == "open_hand":
            coords = np.array([
                [lms.landmark[self.WRIST].x, lms.landmark[self.WRIST].y]
                for lms in gesture_sequence
            ])

            # Average start and end positions to filter out frame jitter
            start_pos = np.mean(coords[:3], axis=0)
            end_pos = np.mean(coords[-3:], axis=0)

            delta_x = end_pos[0] - start_pos[0]
            delta_y = end_pos[1] - start_pos[1]
            displacement = math.hypot(delta_x, delta_y)

            if displacement >= swipe_threshold:
                if abs(delta_x) > abs(delta_y):
                    return "move_right" if delta_x > 0 else "move_left"
                else:
                    return "move_down" if delta_y > 0 else "move_up"
            else:
                return "stop"

        if dominant_posture == "fist_idle":
            return "fist_idle"

        return "unknown"



class VisionNode(Node):
    def __init__(self, record=False):
        super().__init__('vision')
        self.bridge = CvBridge()
        self.record = record
        self.motivated = False

        # --- Parameters ---
        self.declare_parameter('flip_image', True)
        self.declare_parameter('base_S_light', 50.0)
        self.declare_parameter('k_light', 5.0)
        self.declare_parameter('base_S_contrast', 30.0)
        self.declare_parameter('k_contrast', 5.0)
        self.declare_parameter('base_S_speed', 0.2)
        self.declare_parameter('k_speed', 5.0)
        self.declare_parameter('movement_threshold', 0.01)

        self.flip_image = False #self.get_parameter('flip_image').get_parameter_value().bool_value
        self.base_S_light = self.get_parameter('base_S_light').get_parameter_value().double_value
        self.k_light = self.get_parameter('k_light').get_parameter_value().double_value
        self.base_S_contrast = self.get_parameter('base_S_contrast').get_parameter_value().double_value
        self.k_contrast = self.get_parameter('k_contrast').get_parameter_value().double_value
        self.base_S_speed = self.get_parameter('base_S_speed').get_parameter_value().double_value
        self.k_speed = self.get_parameter('k_speed').get_parameter_value().double_value
        self.movement_threshold = self.get_parameter('movement_threshold').get_parameter_value().double_value

        # --- Frame & Camera Buffers ---
        self.color_info = []
        self.color_frame = None
        self.color_stamp = Time(clock_type=self.get_clock().clock_type)
        self.depth_info = []
        self.depth_frame = None
        self.depth_stamp = Time(clock_type=self.get_clock().clock_type)

        self.last_distance = 7.0
        self.last_angle = 0.0
        self.last_seen = self.get_clock().now()
        self.time_tolerance = Duration(seconds=10)
        self.last_log = self.get_clock().now()
        self.log_interval = Duration(seconds=10)
        self.STOP_GESTURE_THRESHOLD =3 

        # --- Models Setup ---
        # 1. YOLOv8
        pkg_path = get_package_share_directory('human_perception')
        model_path = os.path.join(pkg_path, 'models', 'yolov8n.pt')
        self.yolo = YOLO(model_path)
        self.yolo.fuse()

        # 2. ArUco
        """    # Not needed
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        self.marker_size = 0.07
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        """

        # 3. MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1,min_detection_confidence=0.4, min_tracking_confidence=0.4)
        self.mp_drawing = mp.solutions.drawing_utils

        # 4. Descriptors & Recognizer
        self.descriptors = [LightDescriptor(), ContrastDescriptor(), SpeedDescriptor()]
        self.recognizer = AdvancedGestureRecognizer()

        # --- GAD (Gesture Activity Detection) State ---
        self.state = "WAITING"
        self.movement_frames = 0
        self.silence_frames = 0
        self.gesture_buffer = []
        self.descriptor_during_gesture = []

        # --- ROS 2 Publishers ---
        # Spatial / Environmental
        self.vision_pub = self.create_publisher(vsn, "/steven/sensing/vision", 1)
        self.obs_pub = self.create_publisher(vsn_obs, "/steven/sensing/vision_obs", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/steven/sensing/pose", 1)

        # Multimodal & Visual Cognition
        self.content_pub = self.create_publisher(String, '/cognition/visual/content', 10)
        self.desc_pub = self.create_publisher(String, '/cognition/visual/descriptors', 10)
        self.box_pub = self.create_publisher(Image, '/cognition/visual/gesture_box', 10)

        # Continuous Sensory Streams
        self.light_pub = self.create_publisher(Float32, '/cognition/visual/light_pub', 10)
        self.speed_pub = self.create_publisher(Float32, '/cognition/visual/speed_pub', 10)
        self.contrast_pub = self.create_publisher(Float32, '/cognition/visual/contrast_pub', 10)

        # Something about intensity
        self.intensity_light_pub = self.create_publisher(Float32, '/cognition/visual/intensity_light_pub', 10)
        self.intensity_speed_pub = self.create_publisher(Float32, '/cognition/visual/intensity_speed_pub', 10)
        self.intensity_contrast_pub = self.create_publisher(Float32, '/cognition/visual/intensity_contrast_pub', 10)

        # --- ROS 2 Subscriptions ---
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.color_info_callback, 1)
        self.create_subscription(CameraInfo, "/camera/camera/depth/camera_info", self.depth_info_callback, 1)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_callback, 1)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 1)
        self.create_subscription(String, "/mot_mode", self.mode_callback, 1)

        # --- ROS 2 Transforms ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Unified Vision Node started.")
        self.create_timer(0.1, self.process_frame)

    # --- CALLBACKS ---
    def color_info_callback(self, msg):
        self.color_info = msg.k

    def depth_info_callback(self, msg):
        self.depth_info = msg.k

    def depth_callback(self, msg):
        self.depth_frame = self.ros_to_cv2(msg)
        self.depth_stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec, clock_type=self.get_clock().clock_type)

    def color_callback(self, msg):
        self.color_frame = self.ros_to_cv2(msg)
        self.color_stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec, clock_type=self.get_clock().clock_type)

    def mode_callback(self, msg):
        self.motivated = (msg.data == "automatic")

    # --- IMAGE CONVERSION ---
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

        if self.flip_image:
            img = img[::-1, ::-1]
        return img

    def get_depth(self, depth_frame, cx, cy, size=25):
        x1 = max(cx - size // 2, 0)
        y1 = max(cy - size // 2, 0)
        x2 = min(cx + size // 2 + 1, depth_frame.shape[1])
        y2 = min(cy + size // 2 + 1, depth_frame.shape[0])
        patch = depth_frame[y1:y2, x1:x2]
        valid_depths = patch[patch > 0]
        return float(np.min(valid_depths)) if valid_depths.size > 0 else 0.0

    # --- PROCESSING PIPELINE ---
    def process_frame(self):
        if self.color_frame is None or len(self.color_info) == 0:
            return

        image_bgr = self.color_frame
        image_out = image_bgr.copy()
        image_height, image_width, _ = image_bgr.shape
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

        # -------------------------------------------------------------
        # 1. YOLOv8: Person Tracking & Spatial Calculations
        # -------------------------------------------------------------
        results = self.yolo.predict(image_bgr, classes=[0], conf=0.40, device='cuda' if torch.cuda.is_available() else 'cpu', verbose=False)
        fx, fy = self.color_info[0], self.color_info[4]
        cx_intr, cy_intr = self.color_info[2], self.color_info[5]

        min_distance = 10000.0
        min_cx, min_cy = 0, 0
        detected_person = False

        if len(results[0].boxes) > 0 and self.depth_frame is not None:
            detected_person = True
            for box in results[0].boxes.xyxy:
                x1, y1, x2, y2 = map(int, box[:4])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                distance_m = self.get_depth(self.depth_frame / 1000.0, cx, cy)
                if 0.0 < distance_m < min_distance:
                    min_distance = distance_m
                    min_cx, min_cy = cx, cy

        angle = -math.atan2(min_cx - cx_intr, fx) if detected_person else self.last_angle
        if min_distance == 10000.0 or min_distance == 0.0:
            min_distance = self.last_distance
        else:
            self.last_distance = min_distance

        vsn_msg = vsn()
        vsn_msg.header.stamp = self.get_clock().now().to_msg()
        vsn_msg.visible = detected_person
        vsn_msg.angle = float(angle)
        vsn_msg.distance = float(min_distance)
        vsn_msg.last_seen = 0.0 if detected_person else (self.get_clock().now() - self.last_seen).nanoseconds / 1e9

        if detected_person:
            self.last_angle = float(angle)
            self.last_seen = self.get_clock().now()

        self.vision_pub.publish(vsn_msg)

        # Transform lookup (I don't trust anything beyond 3.0 meters)
        if vsn_msg.distance <= 3.0:
            try:
                t = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time())
                pose = PoseStamped()
                pose.header.frame_id = "camera_link"
                pose.pose.position.x = vsn_msg.distance * math.cos(vsn_msg.angle)
                pose.pose.position.y = vsn_msg.distance * math.sin(vsn_msg.angle)
                pose.pose.orientation.z = math.sin(vsn_msg.angle/2)
                pose.pose.orientation.w = math.cos(vsn_msg.angle/2)
                pose.pose = do_transform_pose(pose.pose, t)
                pose.header.frame_id = "map"
                self.pose_pub.publish(pose)
            except Exception as e:
                self.get_logger().warn(f"Transform lookup failed: {e}")
        

        # -------------------------------------------------------------
        # 2. ArUco Marker Obstacle Tracking
        # -------------------------------------------------------------
        """    # ArUco, not even once
        obs_msg = vsn_obs()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if len(corners) > 0 and self.depth_frame is not None:
            camera_matrix = np.array([[fx, 0, cx_intr], [0, fy, cy_intr], [0, 0, 1]])
            dist_coeffs = np.zeros([1, 5])
            marker_points = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            object_points = []
            for i in range(len(ids)):
                _, rvec, tvec = cv2.solvePnP(marker_points, corners[i], camera_matrix, dist_coeffs)
                u = int(corners[i][0][:, 0].mean())
                v = int(corners[i][0][:, 1].mean())
                Z = self.depth_frame[v, u] / 1000.0
                X = (u - cx_intr) * Z / fx
                Y = (v - cy_intr) * Z / fy
                depth_meters = np.sqrt(X**2 + Y**2 + Z**2)

                tvec = tvec.flatten()
                tvec_unit = tvec / np.linalg.norm(tvec)
                tvec_mag = tvec_unit * depth_meters if depth_meters > 0 else tvec
                R, _ = cv2.Rodrigues(rvec)
                tvec_act = tvec_mag + (R @ np.array([0, 0, -0.17]))
                object_points.append(tvec_act)

            object_center = np.mean(object_points, axis=0)
            obs_msg.visible = True
            obs_msg.angle = float(math.atan2(object_center[0], object_center[2]))
            obs_msg.distance = float(np.linalg.norm(object_center))
            obs_msg.shape = 0
        else:
            obs_msg.visible = False
            obs_msg.angle = 0.0
            obs_msg.distance = 0.0
            obs_msg.shape = -1

        self.obs_pub.publish(obs_msg)
        """

        # -------------------------------------------------------------
        # 3. MediaPipe: Hands Detection & Visual Descriptors
        # -------------------------------------------------------------
        detected_hands = self.hands.process(image_rgb)
        hand_lms = detected_hands.multi_hand_landmarks[0] if detected_hands.multi_hand_landmarks else None
        hand_roi, hand_roi_gray = None, None

        if hand_lms:
            self.mp_drawing.draw_landmarks(image_out, hand_lms, self.mp_hands.HAND_CONNECTIONS)

            x_coords = [lm.x for lm in hand_lms.landmark]
            y_coords = [lm.y for lm in hand_lms.landmark]
            min_x = max(0, int(min(x_coords) * image_width) - 20)
            max_x = min(image_width, int(max(x_coords) * image_width) + 20)
            min_y = max(0, int(min(y_coords) * image_height) - 20)
            max_y = min(image_height, int(max(y_coords) * image_height) + 20)

            hand_roi = image_bgr[min_y:max_y, min_x:max_x]
            if hand_roi.size > 0:
                hand_roi_gray = cv2.cvtColor(hand_roi, cv2.COLOR_BGR2GRAY)
            cv2.rectangle(image_out, (min_x, min_y), (max_x, max_y), (0, 255, 0), 2)

        descriptor_data = {desc.name: desc.process(image_bgr, hand_lms, hand_roi, hand_roi_gray) for desc in self.descriptors}

        # -------------------------------------------------------------
        # 4. Continuous Descriptors & Intensities Publishing
        # -------------------------------------------------------------
        self.light_pub.publish(Float32(data=descriptor_data["visual_light"]))
        self.intensity_light_pub.publish(Float32(data=calculate_fechner_intensity(descriptor_data["visual_light"], self.base_S_light, self.k_light, False, False)))

        self.contrast_pub.publish(Float32(data=descriptor_data["visual_contrast"]))
        self.intensity_contrast_pub.publish(Float32(data=calculate_fechner_intensity(descriptor_data["visual_contrast"], self.base_S_contrast, self.k_contrast, False, False)))

        self.speed_pub.publish(Float32(data=descriptor_data["visual_speed"]))
        self.intensity_speed_pub.publish(Float32(data=calculate_fechner_intensity(descriptor_data["visual_speed"], self.base_S_speed, self.k_speed, False, True)))

        # -------------------------------------------------------------
        # 5. Gesture Activity Detection (GAD) & In-Memory Classification
        # -------------------------------------------------------------
        current_speed = descriptor_data.get('visual_speed', 0.0)
        is_moving = (hand_lms is not None) and (current_speed > self.movement_threshold)

        if self.state == "WAITING":
            if hand_lms is not None:
                self.movement_frames += 1
                # Trigger recording after hand is stable for 2 frames
                if self.movement_frames >= 2:
                    self.state = "RECORDING"
                    self.gesture_buffer = [hand_lms]
                    self.descriptor_during_gesture = [descriptor_data]
            else:
                self.movement_frames = 0

        elif self.state == "RECORDING":
            if hand_lms:
                self.gesture_buffer.append(hand_lms)
                self.descriptor_during_gesture.append(descriptor_data)
                self.silence_frames = 0

                # Check for explicit delimiter posture (e.g., Fist)
                if self.recognizer.is_fist(hand_lms.landmark):
                    self.stop_gesture_frames += 1
                else:
                    self.stop_gesture_frames = 0
            else:
                self.silence_frames += 1
                self.stop_gesture_frames = 0

            # End recording when hand disappears (>5 frames) or buffer reaches max window
            # if self.silence_frames > 5 or len(self.gesture_buffer) >= 30:
            fist_cut = self.stop_gesture_frames >= self.STOP_GESTURE_THRESHOLD
            silence_cut = self.silence_frames > 6
            buffer_cap_cut = len(self.gesture_buffer) >= 40

            if fist_cut or silence_cut or buffer_cap_cut:
                # If cut by fist, trim the trailing fist frames so they don't skew swipe classification
                if fist_cut and len(self.gesture_buffer) > self.STOP_GESTURE_THRESHOLD:
                    self.gesture_buffer = self.gesture_buffer[:-self.STOP_GESTURE_THRESHOLD]
                    self.descriptor_during_gesture = self.descriptor_during_gesture[:-self.STOP_GESTURE_THRESHOLD]
                self.state = "PROCESSING"

        elif self.state == "PROCESSING":
            # Classify gesture using remaining action trajectory
            final_label = self.recognizer.process_sequence(
                self.gesture_buffer,
                swipe_threshold=0.08
            )

            avg_descriptors = {
                k: float(np.average([d[k] for d in self.descriptor_during_gesture]))
                for k in ["visual_light", "visual_contrast", "visual_speed"]
            } if self.descriptor_during_gesture else {"visual_light": 0.0, "visual_contrast": 0.0, "visual_speed": 0.0}

            final_payload = {
                "content": final_label,
                "descriptor": avg_descriptors
            }

            self.content_pub.publish(String(data=json.dumps(final_payload)))
            self.get_logger().info(f"Classified Gesture: {final_label}")

            # Reset all counters and buffers
            self.state = "WAITING"
            self.gesture_buffer.clear()
            self.descriptor_during_gesture.clear()
            self.movement_frames = 0
            self.silence_frames = 0
            self.stop_gesture_frames = 0

        # State stream & debug visualizer
        descriptor_data["state"] = self.state
        self.desc_pub.publish(String(data=json.dumps(descriptor_data)))
        self.box_pub.publish(self.bridge.cv2_to_imgmsg(image_out, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        vision_node.get_logger().warn("Shutdown called")
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()