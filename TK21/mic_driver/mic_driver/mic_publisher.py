#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sounddevice as sd
import numpy as np
from fortis_interfaces.msg import Mic as mic_msg
#from fortis_interfaces.msg import Capability
import scipy.signal

class MicPublisher(Node):
    def __init__(self):
        super().__init__('mic_publisher')
        
        self.microphone_index = get_mic_device_index("Yeti GX")
        self.sample_rate = 48000      # 48 kHz = STT standard
        self.channels = 1
        self.block_size = 6144#2048     # ~8Hz publish rate, new: triple that size because of the tripled sample rate, yeah somehow this stop the input overflow, i guess it needs to publish less frequently, so all is good 
        
        self.pub = self.create_publisher(mic_msg, 'mic/raw_audio', 10)
        #self.capability_pub = self.create_publisher(Capability, 'capability/hearing', 10)
        #self.capability_informed = False
        self.last_audio_time = self.get_clock().now()
        self.audio_timeout = 1_500_000_000  # nanoseconds  =1.5 seconds

        self.stream = sd.InputStream(
            device=self.microphone_index,
            channels=self.channels,
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            callback=self.audio_callback
        )
        self.stream.start()
        #self.create_timer(5.0, self.health_check)
        
        self.get_logger().info(f'Streaming 48kHz mic @ ~8Hz ({self.sample_rate}/{self.block_size})')
    
    def audio_callback_2(self, indata, frames, time, status):
        # old version, in this, there is no downscaling sample rate
        if status:
            self.get_logger().warn(str(status))
        
        self.last_audio_time = self.get_clock().now()
        indata = np.nan_to_num(indata, nan=0.0, posinf=1.0, neginf=-1.0)
        indata = np.clip(indata, -10.0, 10.0)  # Hard limit extreme values
        
        msg = mic_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = indata.astype('float32').flatten().tolist()
        self.pub.publish(msg)
        
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(str(status))
        
        self.last_audio_time = self.get_clock().now()
        
        indata = np.nan_to_num(indata, nan=0.0, posinf=1.0, neginf=-1.0)
        indata = np.clip(indata, -10.0, 10.0)  # Hard limit extreme values
        
        #DOWNSAMPLE: 48000 Hz to 16000 Hz
        indata = scipy.signal.decimate(indata, 3, axis=0)
        
        msg = mic_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Convert to float32 list for ROS message
        msg.data = indata.astype('float32').flatten().tolist()
        self.pub.publish(msg)
    
    """def health_check(self):
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
"""
    def try_start_stream(self):
        try:
            self.stream = sd.InputStream(
                device=self.microphone_index,
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

def get_mic_device_index(name_hint="Yeti GX"):
    devices = sd.query_devices()
    for i, dev in enumerate(devices):
        if name_hint.lower() in dev['name'].lower() and dev['max_input_channels'] > 0:
            return i
    raise RuntimeError(f"Microphone '{name_hint}' not found")

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
