import rclpy
#from rclpy.duration import Duration
#from rclpy.time import Time
from rclpy.node import Node
from rclpy import qos
#from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from fortis_interfaces.msg import Hearing as hrn
from fortis_interfaces.msg import Mic as mic_msg
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from vosk import Model, KaldiRecognizer
import numpy as np
#import pyaudio
import json
import os
#import sounddevice as sd



class Hearing(Node):
    def __init__(self, keywords = ["steven", "stephen", "spin"]):
        # Initialization
        super().__init__('hearing')
        self.get_logger().info("Initializing Hearing Node...")
        self.keywords = [keyword.lower() for keyword in keywords]
        self.social_message = Float64()
        self.social_message.data = self.get_clock().now().nanoseconds * 1e-9

        # Model
        pkg_path = get_package_share_directory('sensing_controller')
        model_path = os.path.join(pkg_path, 'models', 'vosk-model-small-en-us-0.15')
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)

        # Open microphone stream
        # self.stream = sd.InputStream(
        #         samplerate=16000, 
        #         channels=1, 
        #         blocksize=4000, 
        #         dtype='int16', 
        #         callback=self.audio_callback)
        # self.stream.start()

        # Ros
        self.get_logger().info("Node started. Listening to data from mic...")
        self.hearing_pub = self.create_publisher(hrn, "/steven/sensing/hearing", 1)
        self.social_pub = self.create_publisher(Float64, "/steven/sensing/time_since_interaction", 1)
        self.mic_sub = self.create_subscription(mic_msg, "mic/raw_audio", self.mic_sub_callback, 1)

    def mic_sub_callback(self, msg):
        # Convert the incoming audio data to bytes
        audio_data = msg.data
        audio_bytes = (np.array(audio_data) * 32767).astype('int16').tobytes()
        # Read audio
        #data_bytes = audio_bytes.tobytes()
        if self.recognizer.AcceptWaveform(audio_bytes):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").lower()
            if text:
                self.get_logger().info(f"Recognized: {text}")
                self.check_keywords(text)
        self.social_pub.publish(self.social_message)

    # def audio_callback(self, indata, frames, time, status):
    #     # Did something go wrong
    #     if status:
    #         self.get_logger().warn(str(status))
        
    #     # Read audio
    #     data_bytes = indata.tobytes()
    #     if self.recognizer.AcceptWaveform(data_bytes):
    #         result = json.loads(self.recognizer.Result())
    #         text = result.get("text", "").lower()
    #         if text:
    #             self.get_logger().info(f"Recognized: {text}")
    #             self.check_keywords(text)
    #     self.social_pub.publish(self.social_message)
        
    # Mostly for reducing clutter
    def check_keywords(self, text):
        text = text.split()
        found_keywords = [word for word in text if word in self.keywords]
        if found_keywords:
            # If interaction (replace later)
            if "steven" in found_keywords or "stephen" in found_keywords:
                self.social_message.data = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"Keywords detected: {found_keywords}")
            message = hrn()
            message.data = found_keywords
            self.hearing_pub.publish(message)



def main(args=None):
    rclpy.init(args=args)
    keywords = ["steven", "stephen", "spin", "turn", "follow", "forward", "back", "go", "begin", "stop"]
    hearing = Hearing(keywords)
    try:
        rclpy.spin(hearing)
    except KeyboardInterrupt:
        hearing.get_logger().warn("Shutdown called")
    except Exception as e:
        hearing.get_logger().error(f"Unexpected error occurred: {e}")
    finally:
        hearing.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()