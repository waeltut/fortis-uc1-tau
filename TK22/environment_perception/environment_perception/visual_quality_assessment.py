#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus
from realsense2_camera_msgs.msg import Metadata
from fortis_interfaces.msg import VideoQualityScore, VideoQualityStatus, VideoQualityStatusColorMetrics, VideoQualityStatusDepthMetrics

from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json


class VideoQualityAssessment(Node):

    def __init__(self):
        super().__init__('video_quality_assessment')

        # --------------------
        # Parameters
        # --------------------
        self.camera_name = 'D455'
        self.color_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/depth/image_rect_raw'
        self.color_meta_topic = '/camera/camera/color/metadata'
        self.depth_meta_topic = '/camera/camera/depth/metadata'

        # --------------------
        # Subscribers
        # --------------------
        self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(Metadata, self.color_meta_topic, self.color_meta_callback, 10)
        self.create_subscription(Metadata, self.depth_meta_topic, self.depth_meta_callback, 10)

        # --------------------
        # Publishers
        # --------------------
        self.status_pub = self.create_publisher(VideoQualityStatus, '/video_quality_status', 10)
        self.score_pub = self.create_publisher(VideoQualityScore, '/video_quality_score', 10)

        # --------------------
        # Internal state
        # --------------------
        self.bridge = CvBridge()

        self.rgb_metrics = {}
        self.depth_metrics = {}
        self.meta_metrics = {}

        self.rgb_ready = False
        self.depth_ready = False

        self.smoothed_score = 0.0

        self.color_frames = 0
        self.depth_frames = 0
        self.last_fps_time = time.time()

        self.get_logger().info(f'Video quality assessment started for {self.camera_name}')

    # ======================================================================
    # Callbacks
    # ======================================================================

    def color_callback(self, msg):
        self.color_frames += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.rgb_metrics = self.compute_rgb_metrics(gray)
            self.rgb_ready = True
        except Exception as e:
            self.get_logger().warn(f'RGB processing failed: {e}')

        self.try_publish(msg.header)

    def depth_callback(self, msg):
        self.depth_frames += 1
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            self.depth_metrics = self.compute_depth_metrics(depth)
            self.depth_ready = True
        except Exception as e:
            self.get_logger().warn(f'Depth processing failed: {e}')

        self.try_publish(msg.header)

    def color_meta_callback(self, msg):
        try:
            meta = json.loads(msg.json_data)
            self.meta_metrics['rgb_auto_exposure'] = meta.get('auto_exposure', False)
            self.meta_metrics['rgb_exposure_time'] = meta.get('actual_exposure', None)
            self.meta_metrics['rgb_gain'] = meta.get('gain', None)
        except Exception as e:
            self.get_logger().debug(f'RGB metadata error: {e}')

    def depth_meta_callback(self, msg):
        try:
            meta = json.loads(msg.json_data)
            self.meta_metrics['depth_laser_power'] = meta.get('frame_laser_power', None)
        except Exception as e:
            self.get_logger().debug(f'Depth metadata error: {e}')

    # ======================================================================
    # Metric computation
    # ======================================================================

    def compute_rgb_metrics(self, gray):
        lap = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = lap.var()

        mean_brightness = float(np.mean(gray))
        contrast = (float(np.max(gray)) - float(np.min(gray))) / (float(np.max(gray)) + float(np.min(gray)) + 1e-6)

        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        noise = np.sqrt(sobelx.var() + sobely.var())

        return {
            'sharpness': sharpness,
            'mean_brightness': mean_brightness,
            'contrast': contrast,
            'noise_level': noise
        }

    def compute_depth_metrics(self, depth):
        valid = np.isfinite(depth)
        valid_count = np.sum(valid)

        fill_rate = 100.0 * valid_count / depth.size if depth.size > 0 else 0.0
        valid_depths = depth[valid]

        depth_std = float(np.std(valid_depths)) if valid_count > 0 else 0.0

        grad_x = np.gradient(depth, axis=1)
        grad_y = np.gradient(depth, axis=0)
        grad_mag = np.sqrt(grad_x**2 + grad_y**2)
        consistency = float(np.mean(grad_mag[valid])) if valid_count > 0 else 0.0

        return {
            'fill_rate': fill_rate,
            'depth_std': depth_std,
            'spatial_consistency': consistency
        }

    # ======================================================================
    # Scoring helpers
    # ======================================================================

    def clamp01(self, x):
        return max(0.0, min(1.0, x))

    def smooth_score(self, value, good_min, good_max):
        return self.clamp01((value - good_min) / (good_max - good_min))

    def compute_quality_score(self):
        score = 0.0

        # RGB (35)
        sharp = self.rgb_metrics.get('sharpness', 0.0)
        score += self.smooth_score(sharp, 50, 300) * 15

        contrast = self.rgb_metrics.get('contrast', 0.0)
        score += self.smooth_score(contrast, 0.1, 0.5) * 10

        noise = self.rgb_metrics.get('noise_level', 0.0)
        score += (1.0 - self.smooth_score(noise, 5, 20)) * 10

        # Depth (30)
        fill = self.depth_metrics.get('fill_rate', 0.0)
        score += self.smooth_score(fill, 85, 98) * 20

        depth_std = self.depth_metrics.get('depth_std', 0.0)
        score += (1.0 - self.smooth_score(depth_std, 0.01, 0.05)) * 10

        # Metadata (15)
        if self.meta_metrics.get('rgb_auto_exposure', False):
            score += 5

        exp = self.meta_metrics.get('rgb_exposure_time')
        if exp is not None:
            score += self.smooth_score(exp, 100, 1000) * 5

        laser = self.meta_metrics.get('depth_laser_power')
        if laser is not None:
            score += self.smooth_score(laser, 150, 360) * 5

        return min(score, 100.0)

    # ======================================================================
    # Publishing
    # ======================================================================

    def try_publish(self, header):
        if not (self.rgb_ready and self.depth_ready):
            return

        raw_score = self.compute_quality_score()
        self.smoothed_score = 0.8 * self.smoothed_score + 0.2 * raw_score
        
        score_msg = VideoQualityScore()
        score_msg.header.stamp = self.get_clock().now().to_msg()
        score_msg.header.frame_id = 'camera_link'
        score_msg.score = self.smoothed_score
        self.score_pub.publish(score_msg)

        status_msg = VideoQualityStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.header.frame_id = 'camera_link'
        status_msg.color = VideoQualityStatusColorMetrics()
        status_msg.color.contrast = self.rgb_metrics['contrast']
        status_msg.color.mean_brightness = self.rgb_metrics['mean_brightness']
        status_msg.color.noise_level = self.rgb_metrics['noise_level']
        status_msg.color.sharpness = self.rgb_metrics['sharpness']
        status_msg.depth = VideoQualityStatusDepthMetrics()
        status_msg.depth.depth_std = self.depth_metrics['depth_std']
        status_msg.depth.fill_rate=self.depth_metrics['fill_rate']
        status_msg.depth.spatial_consistency=self.depth_metrics['spatial_consistency']
        self.status_pub.publish(status_msg)
        
        self.rgb_ready=False
        self.depth_ready=False



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