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
            return np.float32(k_factor * diff if diff > 0 else 0.0)
        else:
            strength_safe = np.maximum(physical_strength, 0.001)
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold
            return np.float32(k_factor * np.log(ratio) if ratio > 1.0 else 0.0)
    except Exception as e:
        return np.float32(0.0)

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
        
class ActionDetectionNode(Node):
    def __init__(self):
        super().__init__('action_detection_node')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # Publishers
        self.gesture_trigger_pub = self.create_publisher(String, '/cognition/visual/gesture_buffer', 10)
        self.desc_pub = self.create_publisher(String, '/cognition/visual/descriptors', 10)
        self.box_pub = self.create_publisher(Image, '/cognition/visual/gesture_box', 10)
        
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
            
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
class AdvancedGestureRecognizer:
    def __init__(self):
        self.WRIST = 0
        self.TIPS = [4, 8, 12, 16, 20] 
        self.PIPS = [2, 6, 10, 14, 18] 

    def _get_finger_states(self, landmarks):
        states = []
        is_thumb_open = landmarks[self.TIPS[0]].x < landmarks[self.TIPS[0] - 1].x
        states.append(is_thumb_open)
        for i in range(1, 5):
            tip = landmarks[self.TIPS[i]]
            pip = landmarks[self.PIPS[i]]
            wrist = landmarks[self.WRIST]
            dist_tip = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
            dist_pip = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
            states.append(dist_tip > dist_pip)
        return states

    def _get_pointing_direction(self, landmarks):
        index_tip = landmarks[8]
        index_mcp = landmarks[5]
        dx = index_tip.x - index_mcp.x
        dy = index_tip.y - index_mcp.y
        if abs(dx) > abs(dy):
            return "move_right" if dx > 0 else "move_left"
        else:
            return "move_down" if dy > 0 else "move_up"

    def recognize(self, hand_lms):
        if not hand_lms: return "none"
        landmarks = hand_lms.landmark
        fingers_up = self._get_finger_states(landmarks)
        fingers_count = sum(fingers_up[1:]) 
        
        if fingers_count >= 3: return "stop"
        elif fingers_up[1] and not fingers_up[2] and not fingers_up[3]: return self._get_pointing_direction(landmarks)
        elif fingers_count == 0: return "fist_idle"
        return "unknown"

class GestureClassificationNode(Node):
    def __init__(self):
        super().__init__('gesture_classification_node')
        self.create_subscription(String, '/cognition/visual/gesture_buffer', self.buffer_callback, 10)
        self.content_pub = self.create_publisher(String, '/cognition/visual/content', 10)
        self.recognizer = AdvancedGestureRecognizer()

    def buffer_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            raw_frames = payload.get("gesture_frames", [])
            avg_descriptors = payload.get("avg_descriptors", {})
            
            # Reconstruct Dummy Objects for the Recognizer
            reconstructed_frames = [DummyHandLMS(frame_data) for frame_data in raw_frames]
            
            labels = []
            for hand_lms in reconstructed_frames:
                label = self.recognizer.recognize(hand_lms)
                if label != "unknown":
                    labels.append(label)
                    
            most_common_label = Counter(labels).most_common(1)[0][0] if labels else "none"
            
            final_payload = {
                "content": most_common_label,
                "descriptor": avg_descriptors
            }
            
            self.content_pub.publish(String(data=json.dumps(final_payload)))
            self.get_logger().info(f"Classified: {most_common_label}")
            
        except Exception as e:
            self.get_logger().error(f"Classification Error: {e}")
def main(args=None):
    rclpy.init(args=args)
    
    classification_node = GestureClassificationNode()
    
    
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