import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time
import math
import os
from math import floor
from ament_index_python.packages import get_package_share_directory
import numpy as np


def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    """
    Applies the Weber-Fechner Law to calculate perceived sensory intensity.
    Psi = k * ln(S / S0)
    """
    try:
        if is_log_scale:
            # dB is already logarithmic. Fechner's law on a log scale is just a linear difference.
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return k_factor * diff if diff > 0 else 0.00001
        else:
            # Prevent division by zero or log(0) errors
            strength_safe = np.maximum(physical_strength, 0.001)

            # The ratio S / S0
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold

            # Only apply intensity scaling if the ratio exceeds the baseline (1.0)
            return k_factor * np.log(ratio) if ratio > 1.0 else 0.00001

    except Exception as e:
        print(f"Intensity calculation error: {e}")
        return 0.0

def calculate_modality_weights_fechner(psi_light,psi_contrast,psi_blur, psi_voice,psi_noise):
    # 1. Calculate Perceived Intensities (Psi) using Fechner's Law
    # (Physical Strength, Optimal Threshold, K-Factor, is_log, inverse)
    """psi_light = calculate_fechner_intensity(visual_descriptors['light'], 100.0, 2.0, False, True)
    psi_contrast = calculate_fechner_intensity(visual_descriptors['contrast'], 30.0, 3.0, False, True)
    psi_blur = calculate_fechner_intensity(visual_descriptors['speed'], 0.05, 5.0, False, False)

    psi_voice = calculate_fechner_intensity(audio_descriptors['volume'], -20.0, 0.1, True, True)
    psi_noise = calculate_fechner_intensity(audio_descriptors['noise'], -50.0, 0.2, True, False)
"""
    # 2. Combine into an overall Modality Intensity Score
    intensity_visual = (psi_light * psi_contrast) / (0.000001 + psi_blur)
    intensity_audio = psi_voice / (0.000001 + psi_noise)

    # 3. Normalize into Bayesian-style Weights
    total_intensity = intensity_visual + intensity_audio

    # Avoid division by zero if both intensities are 0
    if total_intensity <= 0:
        return {"visual_content": 0.0, "audio_content": 0.0}

    return {
        "visual_content": intensity_visual ,#/ total_intensity,
        "audio_content": intensity_audio ,#/ total_intensity
    }
    

        

class Plotting_node(Node):
    def __init__(self):
        super().__init__('fusion_engine_node')
        
    
        self.create_subscription(Float32, '/cognition/visual/intensity_light_pub', self.vis_light_intensity_callback, 10)
        self.create_subscription(Float32, '/cognition/visual/intensity_speed_pub', self.vis_speed_intensity_callback, 10)
        self.create_subscription(Float32, '/cognition/visual/intensity_contrast_pub', self.vis_contrast_intensity_callback, 10)
        self.create_subscription(Float32, '/cognition/audio/intensity_noise_pub', self.aud_noise_intensity_callback, 10)
        self.create_subscription(Float32, '/cognition/audio/intensity_volume_pub', self.aud_volume_intensity_callback, 10)


        self.visual_pub = self.create_publisher(Float32, '/cognition/visual_weight', 10)
        self.audio_pub = self.create_publisher(Float32, '/cognition/audio_weight', 10)

        

    def vis_light_intensity_callback(self, msg):
        self.light_intensity = msg.data
        try:
            output =calculate_modality_weights_fechner(self.light_intensity,
                                                    self.contrast_intensity,
                                                    self.speed_intensity,
                                                    self.volume_intensity,
                                                    self.noise_intensity,
            )
            visual_weight = output["visual_content"]
            audio_weight = output["audio_content"]
            visual_weight_msg = Float32()
            visual_weight_msg.data = visual_weight
            self.visual_pub.publish(visual_weight_msg)

            audio_weight_msg = Float32()
            audio_weight_msg.data = audio_weight
            self.audio_pub.publish(audio_weight_msg)
        except Exception as er:
            self.get_logger().info(str(er))
            pass
            
    def vis_speed_intensity_callback(self, msg):
        self.speed_intensity = msg.data
    
    def vis_contrast_intensity_callback(self, msg):
        self.contrast_intensity = msg.data
    
    def aud_noise_intensity_callback(self, msg):
        self.noise_intensity = msg.data
    
    def aud_volume_intensity_callback(self, msg):
        self.volume_intensity = msg.data
        




def main(args=None):
    rclpy.init(args=args)
    node = Plotting_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()