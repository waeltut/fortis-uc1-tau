import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from fortis_interfaces.msg import Hearing as hrn
from fortis_interfaces.msg import Mic as mic_msg
from std_msgs.msg import Float64, Float32, String
import numpy as np
import json
import os
import math
import whisper
import noisereduce
import librosa
import threading
import queue
from collections import deque
from abc import ABC, abstractmethod


# --- CORE UTILITIES ---
def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    try:
        if is_log_scale:
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return float(k_factor * diff if diff > 0 else np.finfo(float).eps)
        else:
            strength_safe = np.maximum(physical_strength, 0.001)
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold
            return float(k_factor * np.log(ratio) if ratio > 1.0 else np.finfo(float).eps)
    except Exception as e:
        return np.finfo(float).eps


# --- AUDIO DESCRIPTORS ---
class AudioDescriptor(ABC):
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def process(self, audio_float, removed_noise_float, rate):
        pass


class NoiseDescriptor(AudioDescriptor):
    def __init__(self, history_len=20):
        super().__init__("audio_noise")
        self.noise_history = deque(maxlen=history_len)

    def process(self, audio_float, removed_noise_float, rate):
        rms_original = np.sqrt(np.mean(audio_float ** 2))
        cleaned_float = audio_float - removed_noise_float
        rms_cleaned = np.sqrt(np.mean(cleaned_float ** 2))
        power_noise_current = max(0.0, (rms_original ** 2) - (rms_cleaned ** 2))
        rms_noise_current = np.sqrt(power_noise_current)

        self.noise_history.append(rms_noise_current)
        ambient_noise_floor = min(self.noise_history)
        volume_level_noise_db = 20 * np.log10(ambient_noise_floor + 1e-10)
        return float(volume_level_noise_db)


class VolumeDescriptor(AudioDescriptor):
    def __init__(self):
        super().__init__("audio_volume")

    def process(self, audio_float, removed_noise_float, rate):
        rms_float = np.sqrt(np.mean(audio_float ** 2))
        volume_level_db = 20 * math.log10(rms_float) if rms_float > 0 else -100.0
        return float(volume_level_db)


# --- MAIN UNIFIED NODE ---
class HearingNode(Node):
    def __init__(self,
                 keywords=["steven", "stephen", "spin", "turn", "follow", "forward", "back", "go", "begin", "stop", "yes", "done"]):
        super().__init__('hearing')

        # Keyword & Social Setup
        self.keywords = [keyword.lower() for keyword in keywords]
        self.social_message = Float64()
        self.social_message.data = self.get_clock().now().nanoseconds * 1e-9

        # Parameters
        self.declare_parameter('model_filename', 'base.en.pt')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('whisper_rate', 16000)

        self.declare_parameter('base_S_noise', -40.0)
        self.declare_parameter('k_noise', 0.2)
        self.declare_parameter('base_S_volume', -50.0)
        self.declare_parameter('k_volume', 0.1)

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.whisper_rate = self.get_parameter('whisper_rate').get_parameter_value().integer_value
        self.base_S_noise = self.get_parameter('base_S_noise').get_parameter_value().double_value
        self.k_noise = self.get_parameter('k_noise').get_parameter_value().double_value
        self.base_S_volume = self.get_parameter('base_S_volume').get_parameter_value().double_value
        self.k_volume = self.get_parameter('k_volume').get_parameter_value().double_value

        # VAD & Descriptor State
        self.descriptors = [NoiseDescriptor(), VolumeDescriptor()]
        self.state = "SILENCE"
        self.silent_chunks_count = 0
        self.current_speech_buffer = []
        self.desc_during_recording = []

        # ROS 2 Subscriptions
        self.create_subscription(mic_msg, '/mic/raw_audio', self.audio_callback, 100)

        # ROS 2 Publishers
        self.hearing_pub = self.create_publisher(hrn, "/steven/sensing/hearing", 1)
        self.social_pub = self.create_publisher(Float64, "/steven/sensing/time_since_interaction", 1)

        self.content_pub = self.create_publisher(String, '/cognition/audio/content', 10)
        self.desc_pub = self.create_publisher(String, '/cognition/audio/descriptors', 10)

        self.noise_pub = self.create_publisher(Float32, '/cognition/audio/noise_pub', 10)
        self.intensity_noise_pub = self.create_publisher(Float32, '/cognition/audio/intensity_noise_pub', 10)
        self.volume_pub = self.create_publisher(Float32, '/cognition/audio/volume_pub', 10)
        self.intensity_volume_pub = self.create_publisher(Float32, '/cognition/audio/intensity_volume_pub', 10)

        # Load Whisper
        model_filename = self.get_parameter('model_filename').get_parameter_value().string_value
        pkg_path = get_package_share_directory('human_robot_interaction')
        model_path = os.path.join(pkg_path, 'stt_model', model_filename)

        self.get_logger().info(f"Loading Whisper model from {model_path}...")
        self.model = whisper.load_model(model_path)
        self.get_logger().info("Hearing Node initialized successfully.")

        # Transcription Worker Thread
        self.transcription_queue = queue.Queue()
        self.transcription_thread = threading.Thread(target=self.transcription_worker, daemon=True)
        self.transcription_thread.start()

    def audio_callback(self, msg):
        # 1. Ingest Data
        audio_data_float = np.array(msg.data, dtype=np.float32)

        # 2. Noise Reduction & Voice Activity Detection (VAD)
        cleaned_float = noisereduce.reduce_noise(y=audio_data_float, sr=self.sample_rate, stationary=False)
        rms_cleaned = np.sqrt(np.mean(cleaned_float ** 2))
        raw_db = 20 * np.log10(rms_cleaned + 1e-10) 
        #self.get_logger().info(f"Transcribed: {raw_db}")
        is_speech = raw_db > -30

        removed_noise_float = audio_data_float - cleaned_float

        # 3. Continuous Descriptors Computation
        descriptor_data = {}
        for desc in self.descriptors:
            res = desc.process(audio_data_float, removed_noise_float, self.sample_rate)
            descriptor_data[desc.name] = res[0] if isinstance(res, tuple) else res

        descriptor_data["state"] = self.state
        self.desc_pub.publish(String(data=json.dumps(descriptor_data)))

        # Publish Independent Descriptor Streams
        self.noise_pub.publish(Float32(data=descriptor_data["audio_noise"]))
        intensity_noise = calculate_fechner_intensity(descriptor_data["audio_noise"], self.base_S_noise, self.k_noise,
                                                      True, True)
        self.intensity_noise_pub.publish(Float32(data=intensity_noise))

        self.volume_pub.publish(Float32(data=descriptor_data["audio_volume"]))
        intensity_volume = calculate_fechner_intensity(descriptor_data["audio_volume"], self.base_S_volume,
                                                       self.k_volume, True, False)
        self.intensity_volume_pub.publish(Float32(data=intensity_volume))

        # 4. VAD Buffering Logic
        if self.state == "SILENCE" and is_speech:
            self.state = "SPEAKING"
            self.current_speech_buffer = [cleaned_float]
            self.desc_during_recording = [descriptor_data]
            self.silent_chunks_count = 0

        elif self.state == "SPEAKING":
            self.current_speech_buffer.append(cleaned_float)
            self.desc_during_recording.append(descriptor_data)
            self.silent_chunks_count = 0 if is_speech else self.silent_chunks_count + 1

            if self.silent_chunks_count > 10:
                self.state = "SILENCE"

                # Average accumulated descriptors
                avg_descriptor_data = {}
                for desc in ["audio_noise", "audio_volume"]:
                    avg_descriptor_data[desc] = float(np.average([inst[desc] for inst in self.desc_during_recording]))

                # Consolidate float buffer
                full_audio_buffer = np.concatenate(self.current_speech_buffer)
                
                # Push to background thread for inference
                self.transcription_queue.put({
                    "audio_buffer": full_audio_buffer,
                    "descriptor": avg_descriptor_data
                })

    def transcription_worker(self):
        while rclpy.ok():
            try:
                task = self.transcription_queue.get(timeout=1.0)
                audio_float32 = task["audio_buffer"]
                avg_descriptors = task["descriptor"]

                # Resample to Whisper rate if required
                if self.sample_rate != self.whisper_rate:
                    audio_float32 = librosa.resample(audio_float32, orig_sr=self.sample_rate,
                                                     target_sr=self.whisper_rate)

                # Execute Whisper Inference
                result = self.model.transcribe(audio_float32, fp16=False,
                                               condition_on_previous_text=False, # Prevents looping hallucinations
                                                no_speech_threshold=0.6,          # Rejects audio if it thinks it's just noise
                                                logprob_threshold=-1.0)
                text = result.get('text', '').strip().lower()

                if text:
                    self.get_logger().info(f"Transcribed: {text}")

                    # Publish the fused cognition content
                    payload = {"content": text, "descriptor": avg_descriptors}
                    self.content_pub.publish(String(data=json.dumps(payload)))

                    # Process Keywords
                    self.check_keywords(text)

                self.transcription_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Transcription Worker Error: {e}")

    def check_keywords(self, text):
        words = text.split()
        words = [word.strip(",.") for word in words]
        found_keywords = [word for word in words if word in self.keywords]

        if found_keywords:
            if "steven" in found_keywords or "stephen" in found_keywords:
                self.social_message.data = self.get_clock().now().nanoseconds * 1e-9

            self.get_logger().info(f"Keywords detected: {found_keywords}")

            message = hrn()
            message.data = found_keywords
            self.hearing_pub.publish(message)

        self.social_pub.publish(self.social_message)


def main(args=None):
    rclpy.init(args=args)
    node = HearingNode()
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