#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from fortis_interfaces.msg import Mic as mic_msg    
import numpy as np

class AudioAnalyser(Node):
    def __init__(self):
        super().__init__('audio_analyser')
        
        # Audio params (match publisher)
        self.sample_rate = 16000
        self.block_size = 2048
        self.n_fft = 4096
        
        # 1/3 OCTAVE CENTERS (25Hz-12.5kHz, 28 bands) - Industry Standard
        self.third_octave_centers = np.array([
            25,   31.5, 40,   50,   63,   80,   100,  125,
            160,  200,  250,  315,  400,  500,  630,  800,
            1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000,
            6300, 8000, 10000,12500
        ])
        
        self.freqs = np.fft.rfftfreq(self.n_fft, 1/self.sample_rate)
        
        # SINGLE OUTPUT TOPIC
        self.pub = self.create_publisher(Float32MultiArray, 'mic/noise/third_octave', 10)
        
        # Subscribe to mic data
        self.mic_sub = self.create_subscription(mic_msg, 'mic/raw_audio', self.mic_sub_callback, 10)
        
        self.get_logger().info('1/3 Octave Noise Analyzer ready (28 bands)')
    
    def mic_sub_callback(self, msg):
        try:
            # Sanitize input
            audio = np.array(msg.data, dtype=np.float32)
            audio = np.nan_to_num(audio, nan=0.0, posinf=1.0, neginf=-1.0)
            audio = np.clip(audio, -1.0, 1.0)
            
            if len(audio) != self.block_size:
                return
            
            # FFT → Power spectrum → dB
            fft_data = np.fft.rfft(audio, n=self.n_fft)
            fft_power = np.abs(fft_data)**2
            fft_mag_db = 10 * np.log10(np.maximum(fft_power, 1e-12))
            
            # 1/3 OCTAVE BANDING
            spectrum_db = np.zeros(len(self.third_octave_centers))
            band_edges = np.zeros(len(self.third_octave_centers) + 1)
            band_edges[0] = 20.0
            
            for i, center in enumerate(self.third_octave_centers):
                # 1/3 octave bandwidth: center * 2^(1/6)
                band_edges[i+1] = center * 2**(1/6)
                
                # Find FFT bins in this band
                mask = (self.freqs >= band_edges[i]) & (self.freqs < band_edges[i+1])
                if np.any(mask):
                    spectrum_db[i] = np.mean(fft_mag_db[mask])
                else:
                    spectrum_db[i] = -100.0  # Silence floor
            
            # PACK MESSAGE: [freq1, dB1, freq2, dB2, ...] (56 floats total)
            spectrum_msg = Float32MultiArray()
            spectrum_data = []
            for freq, db in zip(self.third_octave_centers, spectrum_db):
                spectrum_data.extend([float(freq), float(db)])
            spectrum_msg.data = spectrum_data
            
            self.pub.publish(spectrum_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Analysis error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AudioAnalyser()
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
