#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sounddevice as sd
import numpy as np
from fortis_interfaces.msg import Mic as mic_msg


class MicPublisher(Node):
    def __init__(self):
        super().__init__('mic_publisher')
        
        # ✅ OPTIMIZED FOR SPEECH + LOW BANDWIDTH
        self.sample_rate = 16000   # 16kHz = STT standard
        self.channels = 1
        self.block_size = 2048     # ~8Hz publish rate
        
        self.pub = self.create_publisher(mic_msg, 'mic/raw_audio', 10)
        
        self.stream = sd.InputStream(
            channels=self.channels,
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            callback=self.audio_callback
        )
        self.stream.start()
        
        self.get_logger().info(f'Streaming 16kHz mic @ ~8Hz ({self.sample_rate}/{self.block_size})')
    
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(str(status))
        
        # 🛡️ FIX: Sanitize NaN/Inf values
        indata = np.nan_to_num(indata, nan=0.0, posinf=1.0, neginf=-1.0)
        indata = np.clip(indata, -10.0, 10.0)  # Hard limit extreme values
        
        msg = mic_msg()
        msg.data = indata.astype('float32').flatten().tolist()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
