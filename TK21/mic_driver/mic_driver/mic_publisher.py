#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sounddevice as sd
import numpy as np
from fortis_interfaces.msg import Mic as mic_msg
from fortis_interfaces.msg import Capability


class MicPublisher(Node):
    def __init__(self):
        super().__init__('mic_publisher')
        
        # ✅ OPTIMIZED FOR SPEECH + LOW BANDWIDTH
        self.sample_rate = 16000   # 16kHz = STT standard
        self.channels = 1
        self.block_size = 2048     # ~8Hz publish rate
        
        self.pub = self.create_publisher(mic_msg, 'mic/raw_audio', 10)
        self.capability_pub = self.create_publisher(Capability, 'capability/hearing', 10)
        self.capability_informed = False
        self.last_audio_time = self.get_clock().now()
        self.audio_timeout = 1_500_000_000  # nanoseconds  =1.5 seconds

        self.stream = sd.InputStream(
            channels=self.channels,
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            callback=self.audio_callback
        )
        self.stream.start()
        self.create_timer(5.0, self.health_check)
        
        self.get_logger().info(f'Streaming 16kHz mic @ ~8Hz ({self.sample_rate}/{self.block_size})')
    
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(str(status))
        
        # 🛡️ FIX: Sanitize NaN/Inf values
        self.last_audio_time = self.get_clock().now()
        indata = np.nan_to_num(indata, nan=0.0, posinf=1.0, neginf=-1.0)
        indata = np.clip(indata, -10.0, 10.0)  # Hard limit extreme values
        
        msg = mic_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = indata.astype('float32').flatten().tolist()
        self.pub.publish(msg)

    
    def health_check(self):
        if self.stream is None or not self.stream.active or self.get_clock().now().nanoseconds - self.last_audio_time.nanoseconds > self.audio_timeout:
            self.get_logger().warn("Mic stream inactive, reopening...")
            msg = Capability()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.capability_name = "Hearing"
            msg.capability_description = "Microphone is not streaming audio signal"
            msg.capability_devices = ["Microphone"]
            msg.capability_health = "NOT_STREAMING"
            self.capability_pub.publish(msg)

            try:
                if self.stream:
                    self.stream.close()
            except Exception:
                pass
            self.stream = None
            self.try_start_stream()
            self.capability_informed = False
        elif not self.capability_informed:
            msg = Capability()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.capability_name = "Hearing"
            msg.capability_description = "Microphone is streaming audio signal"
            msg.capability_devices = ["Microphone"]
            msg.capability_health = "STREAMING"
            self.capability_pub.publish(msg)
            self.capability_informed = True

    def try_start_stream(self):
        try:
            self.stream = sd.InputStream(
                channels=self.channels,
                samplerate=self.sample_rate,
                blocksize=self.block_size,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info("Mic stream restarted successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to restart mic stream: {e}")
            self.stream = None



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
