#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from realsense2_camera_msgs.msg import Metadata
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
#from scipy.ndimage import sobel
import time
#import json

class VideoQualityAssessment(Node):
    def __init__(self):
        super().__init__('video_quality_assessment')
        
        # Parameters
        self.declare_parameter('camera_name', 'D455')
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('color_meta_topic', '/camera/camera/color/metadata')
        self.declare_parameter('depth_meta_topic', '/camera/camera/depth/metadata')
        
        self.camera_name = self.get_parameter('camera_name').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_meta_topic = self.get_parameter('color_meta_topic').value
        self.depth_meta_topic = self.get_parameter('depth_meta_topic').value
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10)
        self.color_meta_sub = self.create_subscription(
            Metadata, self.color_meta_topic, self.color_meta_callback, 10)
        self.depth_meta_sub = self.create_subscription(
            Metadata, self.depth_meta_topic, self.depth_meta_callback, 10)
        
        # Publishers
        self.quality_pub = self.create_publisher(DiagnosticArray, '/video_quality_status', 10)
        self.quality_score_pub = self.create_publisher(Float64, '/video_quality_score', 10)
        
        self.bridge = CvBridge()
        self.metrics = {}
        self.rgb_metrics = {}
        self.depth_metrics = {}
        self.meta_metrics = {}
        
        self.color_frame_count = 0
        self.depth_frame_count = 0
        self.last_time = time.time()
        
        self.get_logger().info(f'Quality publisher ready for {self.camera_name}')
        self.get_logger().info(f'Subscribing to: {self.color_topic}, {self.depth_topic}')
        
    def color_callback(self, msg):
        self.color_frame_count += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.rgb_metrics = self.compute_rgb_metrics(gray)
        except Exception as e:
            self.get_logger().warn(f'RGB processing failed: {e}')
        
        self.publish_combined_metrics(msg.header)
    
    def depth_callback(self, msg):
        self.depth_frame_count += 1
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.depth_metrics = self.compute_depth_metrics(depth_image)
        except Exception as e:
            self.get_logger().warn(f'Depth processing failed: {e}')
        
        self.publish_combined_metrics(msg.header)
    
    def color_meta_callback(self, msg):
        try:
            self.meta_metrics['rgb_actual_exposure'] = msg.auto_exposure
            self.meta_metrics['rgb_fps'] = msg.actual_fps / 100.0
            self.meta_metrics['rgb_sensor_brightness'] = msg.brightness
            # I did not find anything that says rgb_gain or similar. Might have to come back later
            # self.meta_metrics['rgb_gain'] = msg.gain_field_if_exists
        except Exception as e:
            print(f"Error in color_meta_callback: {e}")
        """
        try:
            if msg.key == 'actual_exposure':
                self.meta_metrics['rgb_actual_exposure'] = float(msg.value)
            elif msg.key == 'gain':
                self.meta_metrics['rgb_gain'] = float(msg.value)
            elif msg.key == 'frame_rate':
                self.meta_metrics['rgb_fps'] = float(msg.value)
            elif msg.key == 'brightness':
                self.meta_metrics['rgb_sensor_brightness'] = int(msg.value)
        except:
            pass
        """
    
    def depth_meta_callback(self, msg):
        try:
            self.meta_metrics['depth_laser_power'] = msg.frame_laser_power
            # Again, could not find anything related to confidence and frame drops
        except Exception as e:
            print(f"Error in depth_meta_callback: {e}")
        """
        try:
            if msg.key == 'laser_power':
                self.meta_metrics['depth_laser_power'] = float(msg.value)
            elif msg.key == 'confidence':
                self.meta_metrics['depth_confidence'] = float(msg.value)
            elif msg.key == 'frame_drop_percentage':
                self.meta_metrics['frame_drop_pct'] = float(msg.value)
        except:
            pass
        """
    
    def compute_rgb_metrics(self, gray):
        # Sharpness (Laplacian variance)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = laplacian.var()
        
        # Brightness statistics
        mean_bright = np.mean(gray)
        std_bright = np.std(gray)
        
        # Contrast (Michelson)
        gray = gray.astype(np.float32)
        contrast = (np.max(gray) - np.min(gray)) / (np.max(gray) + np.min(gray) + 1e-6)
        
        # Noise level (high-frequency content)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        noise_level = np.sqrt(sobelx.var() + sobely.var())
        
        return {
            'sharpness': sharpness,
            'mean_brightness': mean_bright,
            'brightness_std': std_bright,
            'contrast': contrast,
            'noise_level': noise_level
        }
    
    def compute_depth_metrics(self, depth_image):
        valid_mask = np.isfinite(depth_image)
        valid_pixels = np.sum(valid_mask)
        total_pixels = depth_image.size
        
        # Fill rate
        fill_rate = (valid_pixels / total_pixels) * 100
        
        # Depth precision
        valid_depths = depth_image[valid_mask]
        depth_std = np.std(valid_depths) if valid_pixels > 0 else 0.0
        
        # Depth range
        depth_min = np.min(valid_depths) if valid_pixels > 0 else 0.0
        depth_max = np.max(valid_depths) if valid_pixels > 0 else 0.0
        
        # Spatial consistency
        grad_x = np.gradient(depth_image, axis=1)
        grad_y = np.gradient(depth_image, axis=0)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        consistency = np.mean(grad_mag[valid_mask]) if valid_pixels > 0 else 0.0
        
        return {
            'fill_rate': fill_rate,
            'depth_std': depth_std,
            'depth_range': depth_max - depth_min,
            'spatial_consistency': consistency
        }
    
    def compute_quality_score(self):
        """Combined quality score 0-100"""
        score = 0.0
        
        # RGB Image metrics (40%)
        if 'sharpness' in self.rgb_metrics:
            score += min(self.rgb_metrics['sharpness'] / 200, 1.0) * 20
        if 'brightness_std' in self.rgb_metrics:
            score += min(self.rgb_metrics['brightness_std'] / 80, 1.0) * 10
        if 'contrast' in self.rgb_metrics:
            score += min(self.rgb_metrics['contrast'] / 0.5, 1.0) * 10
        
        # Depth Image metrics (30%)
        if 'fill_rate' in self.depth_metrics:
            score += min(self.depth_metrics['fill_rate'] / 100, 1.0) * 20
        if 'depth_std' in self.depth_metrics:
            score += max(0, 1.0 - (self.depth_metrics['depth_std'] / 0.02)) * 10
        
        # Metadata metrics (30%)
        if 'rgb_actual_exposure' in self.meta_metrics:
            exp = self.meta_metrics['rgb_actual_exposure']
            score += 1.0 if 50 < exp < 1000 else 0.3 * 10
        if 'rgb_gain' in self.meta_metrics:
            gain = self.meta_metrics['rgb_gain']
            score += min(1.0 / (gain + 1), 1.0) * 10
        if 'depth_laser_power' in self.meta_metrics:
            laser = self.meta_metrics['depth_laser_power']
            score += min(laser / 360, 1.0) * 10
        
        return min(score, 100.0)
    
    def publish_combined_metrics(self, header):
        if not self.rgb_metrics or not self.depth_metrics:
            return
            
        diag_array = DiagnosticArray()
        diag_array.header = header
        
        # RGB Status
        rgb_status = DiagnosticStatus()
        rgb_status.name = 'RGB Quality Metrics'
        rgb_status.level = self.get_status_level(self.rgb_metrics)
        rgb_status.message = f'RGB Quality: {self.compute_quality_score():.1f}/100'
        
        for key, value in self.rgb_metrics.items():
            kv = KeyValue(key=f'rgb_{key}', value=str(value))
            rgb_status.values.append(kv)
        
        # Depth Status
        depth_status = DiagnosticStatus()
        depth_status.name = 'Depth Quality Metrics'
        depth_status.level = self.get_status_level(self.depth_metrics)
        depth_status.message = 'Depth Quality: OK'
        
        for key, value in self.depth_metrics.items():
            kv = KeyValue(key=f'depth_{key}', value=str(value))
            depth_status.values.append(kv)
        
        # Metadata Status
        meta_status = DiagnosticStatus()
        meta_status.name = 'Camera Metadata'
        meta_status.level = DiagnosticStatus.OK
        meta_status.message = 'Metadata available'
        
        for key, value in self.meta_metrics.items():
            kv = KeyValue(key=key, value=str(value))
            meta_status.values.append(kv)
        
        diag_array.status.extend([rgb_status, depth_status, meta_status])
        self.quality_pub.publish(diag_array)
        
        # Quality score
        score_msg = Float64()
        score_msg.data = self.compute_quality_score()
        self.quality_score_pub.publish(score_msg)
        
        # FPS logging
        now = time.time()
        if now - self.last_time > 1.0:
            fps_color = self.color_frame_count
            fps_depth = self.depth_frame_count
            self.get_logger().debug(f'RGB FPS: {fps_color}, Depth FPS: {fps_depth}, Quality: {score_msg.data:.1f}')
            self.color_frame_count = 0
            self.depth_frame_count = 0
            self.last_time = now
    
    def get_status_level(self, metrics):
        if 'sharpness' in metrics and metrics['sharpness'] < 50:
            return DiagnosticStatus.WARN
        if 'fill_rate' in metrics and metrics['fill_rate'] < 90:
            return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

def main(args=None):
    rclpy.init(args=args)
    node = VideoQualityAssessment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutdown called")
    except Exception as e:
        node.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
