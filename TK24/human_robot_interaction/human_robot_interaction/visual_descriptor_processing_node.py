import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import json
import math
from collections import Counter
from abc import ABC, abstractmethod

# --- CORE FUNCTIONS ---
def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    try:
        if is_log_scale:
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return np.float32(k_factor * diff if diff > 0 else np.finfo(float).eps)
        else:
            strength_safe = np.maximum(physical_strength, 0.001)
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold
            return np.float32(k_factor * np.log(ratio) if ratio > 1.0 else np.finfo(float).eps)
    except Exception as e:
        
        return np.finfo(float).eps

# --- DESCRIPTORS ---
class VisualDescriptor(ABC):
    def __init__(self, name):
        self.name = name
    @abstractmethod
    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        pass
    def _brightness(self, img):
        if img is None or img.size == 0: return 0
        if len(img.shape) == 3: return np.average(np.linalg.norm(img, axis=2)) / np.sqrt(3)
        return np.average(img)
    def _calculate_contrast(self, roi_gray):
        if roi_gray is None or roi_gray.size == 0: return 0.0
        return roi_gray.std()

class LightDescriptor(VisualDescriptor):
    def __init__(self): super().__init__("visual_light")
    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        return self._brightness(image_bgr)

class ContrastDescriptor(VisualDescriptor):
    def __init__(self): super().__init__("visual_contrast")
    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        return self._calculate_contrast(hand_roi_gray)

class SpeedDescriptor(VisualDescriptor):
    def __init__(self):
        super().__init__("visual_speed")
        self.prev_coord = None
    def process(self, image_bgr, hand_lms, hand_roi, hand_roi_gray):
        if hand_lms:
            lm_list = hand_lms.landmark
            landmark = lm_list[4]
            if self.prev_coord is None:
                self.prev_coord = [landmark.x, landmark.y, landmark.z]
                return 0.0
            current_speed = np.linalg.norm([
                landmark.x - self.prev_coord[0],
                landmark.y - self.prev_coord[1],
                landmark.z - self.prev_coord[2]
            ])
            self.prev_coord = [landmark.x, landmark.y, landmark.z]
            return current_speed
        else:
            self.prev_coord = None
            return 0.0

# --- DATA RECONSTRUCTION (For Node 2) ---
class DummyLandmark:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

class DummyHandLMS:
    def __init__(self, landmarks_dicts):
        self.landmark = [DummyLandmark(lm['x'], lm['y'], lm['z']) for lm in landmarks_dicts]
        
class VisualDescriptorNode(Node):
    def __init__(self):
        super().__init__('visual_descriptor_action_detection_node')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.gesture_trigger_pub = self.create_publisher(String, '/cognition/visual/gesture_buffer', 10)
        self.desc_pub = self.create_publisher(String, '/cognition/visual/descriptors', 10)
        self.box_pub = self.create_publisher(Image, '/cognition/visual/gesture_box', 10)
        
        self.light_pub = self.create_publisher(Float32, '/cognition/visual/light_pub', 10)
        self.speed_pub = self.create_publisher(Float32, '/cognition/visual/speed_pub', 10)
        self.contrast_pub = self.create_publisher(Float32, '/cognition/visual/contrast_pub', 10)
        
        self.intensity_light_pub = self.create_publisher(Float32, '/cognition/visual/intensity_light_pub', 10)
        self.intensity_speed_pub = self.create_publisher(Float32, '/cognition/visual/intensity_speed_pub', 10)
        self.intensity_contrast_pub = self.create_publisher(Float32, '/cognition/visual/intensity_contrast_pub', 10)
        
        # MediaPipe Setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.descriptors = [LightDescriptor(), ContrastDescriptor(), SpeedDescriptor()]
        
        # GAD State
        self.state = "WAITING"
        self.movement_frames = 0
        self.silence_frames = 0
        self.gesture_buffer = [] 
        self.descriptor_during_gesture = []
        self.MOVEMENT_THRESHOLD = 0.01

        self.declare_parameter('base_S_light', 50.0)
        self.declare_parameter('k_light', 5.0)
        self.declare_parameter('base_S_contrast', 30.0)
        self.declare_parameter('k_contrast', 5.0)
        self.declare_parameter('base_S_speed', 0.2)
        self.declare_parameter('k_speed', 5.0)
        
        self.declare_parameter('base_S_noise', -10.0)
        self.declare_parameter('k_noise', 0.2)
        self.declare_parameter('base_S_volume', -30.0)
        self.declare_parameter('k_volume', 0.1)
        
        self.base_S_light = self.get_parameter('base_S_light').get_parameter_value().double_value
        self.k_light = self.get_parameter('k_light').get_parameter_value().double_value
        self.base_S_contrast = self.get_parameter('base_S_contrast').get_parameter_value().double_value
        self.k_contrast = self.get_parameter('k_contrast').get_parameter_value().double_value
        self.base_S_speed = self.get_parameter('base_S_speed').get_parameter_value().double_value
        self.k_speed = self.get_parameter('k_speed').get_parameter_value().double_value
        
        self.base_S_noise = self.get_parameter('base_S_noise').get_parameter_value().double_value
        self.k_noise = self.get_parameter('k_noise').get_parameter_value().double_value
        self.base_S_volume = self.get_parameter('base_S_volume').get_parameter_value().double_value
        self.k_volume = self.get_parameter('k_volume').get_parameter_value().double_value
        self.get_logger().info(f"param test: {self.base_S_light}")
    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            image_out = image_bgr.copy()
            image_height, image_width, _ = image_bgr.shape
            
            detected_image = self.hands.process(image_rgb)
            hand_lms = detected_image.multi_hand_landmarks[0] if detected_image.multi_hand_landmarks else None
            hand_detected = hand_lms is not None
            
            hand_roi, hand_roi_gray = None, None 
            
            if hand_detected:
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
            
            # GAD Logic
            current_speed = descriptor_data.get('visual_speed', 0)
            is_moving = hand_detected and current_speed > self.MOVEMENT_THRESHOLD

            if self.state == "WAITING" and is_moving:
                self.movement_frames += 1
                if self.movement_frames > 3:
                    self.state = "RECORDING"
                    self.gesture_buffer = [hand_lms]
                    self.descriptor_during_gesture = [descriptor_data]
            
            elif self.state == "RECORDING":
                if hand_lms: self.gesture_buffer.append(hand_lms)
                self.descriptor_during_gesture.append(descriptor_data)
                
                self.silence_frames = 0 if is_moving else self.silence_frames + 1
                if self.silence_frames > 15:
                    self.state = "PROCESSING"

            elif self.state == "PROCESSING":
                # Serialize landmarks for transmission
                serialized_buffer = []
                for frame_lms in self.gesture_buffer:
                    frame_data = [{'x': lm.x, 'y': lm.y, 'z': lm.z} for lm in frame_lms.landmark]
                    serialized_buffer.append(frame_data)
                
                avg_desc = {k: float(np.average([d[k] for d in self.descriptor_during_gesture])) for k in descriptor_data.keys()}
                
                payload = {
                    "gesture_frames": serialized_buffer,
                    "avg_descriptors": avg_desc
                }
                
                # Trigger Node 2
                self.gesture_trigger_pub.publish(String(data=json.dumps(payload)))
                
                self.state = "WAITING"
                self.gesture_buffer.clear()
                self.descriptor_during_gesture.clear()

            # Publish Streams
            descriptor_data["state"] = self.state
            self.desc_pub.publish(String(data=json.dumps(descriptor_data)))
            self.box_pub.publish(self.bridge.cv2_to_imgmsg(image_out))
            
            self.light_pub.publish(Float32(data=descriptor_data["visual_light"]))
            
            intensity_light_msg = Float32()
            intensity_light_msg.data = calculate_fechner_intensity(descriptor_data["visual_light"], self.base_S_light, self.k_light, False, False).astype(float)
            self.intensity_light_pub.publish(intensity_light_msg)
            
            self.contrast_pub.publish(Float32(data=descriptor_data["visual_contrast"]))
            
            intensity_contrast_msg = Float32()
            intensity_contrast_msg.data = calculate_fechner_intensity(descriptor_data["visual_contrast"], self.base_S_contrast, self.k_contrast, False, False).astype(float)
            self.intensity_contrast_pub.publish(intensity_contrast_msg)
            
            self.speed_pub.publish(Float32(data=descriptor_data["visual_speed"]))
            intensity_speed_msg = Float32()
            intensity_speed_msg.data = calculate_fechner_intensity(descriptor_data["visual_speed"], self.base_S_speed, self.k_speed, False, True).astype(float)
            self.intensity_speed_pub.publish(intensity_speed_msg)  

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    classification_node = VisualDescriptorNode()
    
    
    try:
        rclpy.spin(classification_node)
    except KeyboardInterrupt:
        pass
    finally:
        classification_node.shutdown()
        classification_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()