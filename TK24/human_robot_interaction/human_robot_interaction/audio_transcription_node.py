import mediapipe as mp
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
#import sounddevice as sd
from scipy.io.wavfile import write
import time
import subprocess, sys, os, json
import pprint
from datetime import timedelta, datetime
import ffmpeg
import wave
import keyboard
import pyaudio
import audioop
from math import log10, floor
from scipy.fft import fft, ifft
from collections import deque, defaultdict
import librosa  # For pitch
import traceback
import multiprocessing
import queue  # Used for the queue.Empty exception
import pprint
from abc import ABC, abstractmethod
import json
import noisereduce
import torch
import torchvision
import math
import traceback
import sys
import multiprocessing
import queue
import time
import json
import math
import os
from collections import deque
import whisper

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from fortis_interfaces.msg import Mic as mic_msg
from cv_bridge import CvBridge
import queue
import soundfile as sf

from std_msgs.msg import String, Float32
import numpy as np
import json
import librosa
from collections import deque
import noisereduce
import wave
import signal
from ament_index_python.packages import get_package_share_directory
import yaml
import base64
# -----------------------------------------------------------------
# UTILITY FUNCTIONS
# -----------------------------------------------------------------+
#os.chdir("src/human_robot_interaction/human_robot_interaction/stt_model/")
def calculate_fechner_intensity(physical_strength, optimal_threshold, k_factor, is_log_scale=False, inverse=False):
    """
    Applies the Weber-Fechner Law to calculate perceived sensory intensity.
    Psi = k * ln(S / S0)
    """
    try:
        if is_log_scale:
            # dB is already logarithmic. Fechner's law on a log scale is just a linear difference.
            diff = optimal_threshold - physical_strength if inverse else physical_strength - optimal_threshold
            return k_factor * diff if diff > 0 else 0.0
        else:
            # Prevent division by zero or log(0) errors
            strength_safe = np.maximum(physical_strength, 0.001)

            # The ratio S / S0
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold

            # Only apply intensity scaling if the ratio exceeds the baseline (1.0)
            return k_factor * np.log(ratio) if ratio > 1.0 else 0.0

    except Exception as e:
        print(f"Intensity calculation error: {e}")
        return 0.0
class AudioDescriptor(ABC):
    """
    Abstract base class for any audio descriptor.
    This defines the "plugin" interface.
    """

    def __init__(self, name):
        self.name = name  # The key for the output dictionary (e.g., "pitch")

    @abstractmethod
    def process(self, audio_float, audio_int, removed_noise_float, rate):
        """
        Processes a single chunk of audio data.

        Args:
            audio_float (np.array): Audio data as float32 (-1.0 to 1.0)
            audio_int (np.array): Audio data as int16
            removed_noise_float (np.array): The isolated noise as float32
            rate (int): The sample rate (e.g., 44100)

        Returns:
            float or str: The calculated descriptor value.
        """
        pass


# -----------------------------------------------------------------
# CONCRETE DESCRIPTOR CLASSES
# -----------------------------------------------------------------

class NoiseDescriptor(AudioDescriptor):
    """Calculates the RMS volume of the *removed noise* signal in dB."""

    def __init__(self):
        super().__init__("audio_noise")

    def process(self, audio_float, audio_int, removed_noise_float, rate):
        rms_noise_float = np.sqrt(np.mean(removed_noise_float ** 2))
        volume_level_noise_db = 10 * log10(rms_noise_float) if rms_noise_float > 0 else -100
        return volume_level_noise_db


class PredictiveVolumeDescriptor(AudioDescriptor):
    """
    Calculates RMS volume and applies Predictive Coding.
    If the actual volume deviates significantly from the predicted (average) volume,
    it generates a 'surprise' multiplier to boost attention.
    """

    def __init__(self):
        super().__init__("audio_volume")
        # Predictive State
        self.history = deque(maxlen=10)  # Short-term memory
        self.attention_multiplier = 1.0

        # Hyperparameters
        self.DECAY_RATE = 0.95  # How fast attention fades (0.0-1.0)
        self.SURPRISE_THRESHOLD = 10.0  # dB difference to trigger surprise
        # self.MAX_MULTIPLIER = 3.0  # Max boost to weight
        self.MAX_MULTIPLIER = 1
        self.BASE_MULTIPLIER = 1.0  # Normal state

    def predict_next_volume(self):
        """Predicts volume based on simple moving average."""
        if not self.history:
            return -50.0  # Default silence assumption
        return np.mean(self.history)

    def process(self, audio_float, audio_int, removed_noise_float, rate):
        # 1. Calculate Actual Volume (Observation)
        rms_float = np.sqrt(np.mean(audio_float ** 2))
        current_db = 10 * log10(rms_float) if rms_float > 0 else -100

        # 2. Generate Prediction (Top-Down Expectation)
        predicted_db = self.predict_next_volume()

        # 3. Calculate Prediction Error (Surprise)
        prediction_error = abs(current_db - predicted_db)

        # 4. Update Attention Multiplier (The "Decaying Multiplier")
        if prediction_error > self.SURPRISE_THRESHOLD:
            # Significant change detected! Boost attention.
            # The bigger the error, the bigger the boost (clamped)
            boost = 1.0 + (prediction_error / 20.0)
            self.attention_multiplier = min(self.attention_multiplier + boost, self.MAX_MULTIPLIER)
        else:
            # No surprise, decay back to baseline
            self.attention_multiplier = max(self.BASE_MULTIPLIER,
                                            self.attention_multiplier * self.DECAY_RATE)

        # 5. Update History (Learning)
        self.history.append(current_db)

        # Return tuple: (Actual Value, Attention Multiplier)
        # Note: We return a tuple here, so we must handle it in AudioProcessor
        return (current_db, self.attention_multiplier)


class PitchDescriptor(AudioDescriptor):
    """Calculates the fundamental frequency (pitch) using YIN."""

    def __init__(self):
        super().__init__("audio_pitch")
        self.fmin = librosa.note_to_hz('C2')
        self.fmax = librosa.note_to_hz('C7')

    def process(self, audio_float, audio_int, removed_noise_float, rate):
        f0 = librosa.yin(
            audio_float,
            fmin=self.fmin,
            fmax=self.fmax,
            sr=rate
        )
        voiced_f0 = f0[f0 > 0]
        current_pitch = np.mean(voiced_f0) if len(voiced_f0) > 0 else 0
        return current_pitch


class ToneClassifier(AudioDescriptor):
    """
    (Example Placeholder)
    A future model that could classify the speaker's tone.
    """

    def __init__(self):
        super().__init__("tone")
        # In a real app, you would load a ML model here
        # self.model = load_model("tone_classifier.h5")

    def process(self, audio_float, audio_int, removed_noise_float, rate):
        # --- Placeholder Logic ---
        f0 = librosa.yin(audio_float, fmin=50, fmax=500, sr=rate)
        pitch = np.mean(f0[f0 > 0]) if len(f0[f0 > 0]) > 0 else 0

        if pitch > 200:
            return "excited"
        elif pitch < 100 and pitch > 0:
            return "calm"
        else:
            return "neutral"


def calculate_level(current_value: float, min_value: float, max_value: float) -> int:
    """
    Scales a value from a given range [min_value, max_value] to an integer level from 1 to 10.
    """
    # 1. Validate that the min/max range is valid
    if min_value >= max_value:
        raise ValueError("Error: 'max_value' must be strictly greater than 'min_value'.")
    # 2. Handle edge cases

    if current_value <= min_value: return 1
    if current_value >= max_value: return 10
    # 3. Calculate interval
    total_range = max_value - min_value
    level_interval = total_range / 10.0
    # 4. Determine level
    level = floor((current_value - min_value) / level_interval) + 1
    return int(level)


def scale_reliability_weights(data):
    # Find the highest reliability_weight in the dictionary
    max_weight = max(item['reliability_weight'] for item in data.values())

    # Check to prevent division by zero in case all weights are 0
    if max_weight > 0:
        for item in data.values():
            # Divide each weight by the max_weight to scale it
            item['reliability_weight'] /= max_weight

    return data


def is_float(input_number):
    try:
        k = float(input_number)
        return True
    except:
        return False

# -----------------------------------------------------------------
# PROCESS 2: AUDIO PROCESSOR CLASS
# -----------------------------------------------------------------



class AudioProcessorNode(Node):
    def __init__(self):
        super().__init__('audio_transcription_node')
        self.declare_parameter('model_filename', 'base.en.pt')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('whisper_rate', 16000)
        self.declare_parameter('buffer_seconds', 2.5)
        self.declare_parameter('noise_reduction_factor', 1.3)
        self.declare_parameter('channels', 1)
        self.declare_parameter('sample_width', 2)

        model_filename = self.get_parameter('model_filename').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.RATE = self.sample_rate 
        self.WHISPER_RATE = self.get_parameter('whisper_rate').get_parameter_value().integer_value
        self.BUFFER_SECONDS = self.get_parameter('buffer_seconds').get_parameter_value().double_value
        self.NOISE_REDUCTION_FACTOR = self.get_parameter('noise_reduction_factor').get_parameter_value().double_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.sample_width = self.get_parameter('sample_width').get_parameter_value().integer_value
        
        
        # Subscriptions
        self.create_subscription(String, '/cognition/audio/descriptor_segment', self.audio_callback_base64, 10)
        
        # Publishers
        self.content_pub = self.create_publisher(String, '/cognition/audio/content', 10)
        # Audio State
        #self.RATE = 16000 # Match your publisher
        #self.audio_buffer = deque(maxlen=self.RATE * 5) # 5 second rolling buffer

        #self.BUFFER_SECONDS = 2.0
        #self.NOISE_REDUCTION_FACTOR = 1.3
        self.count = 0
        # Whisper requires 16000 Hz
        #self.WHISPER_RATE = 16000
        self.state = "SILENCE"
        self.speech_chunks = []
        self.silent_chunks_count = 0
        self.desc_during_recording = []
        #self.model_path = "base.en.pt"
        #self.channels = 1
        #self.sample_width = 2      # bytes (16-bit audio)
        #self.sample_rate = 16000   # Hz
        package_share_directory = get_package_share_directory('human_robot_interaction')
        self.model_path = os.path.join(package_share_directory, 'stt_model', model_filename)
        self.audio_file_path = os.path.join(package_share_directory, "recorded_audio.wav")
        # Processing Timer (Runs at ~10Hz to unblock the subscriber)
        #self.create_timer(0.1, self.process_audio_loop)
        try:
            # Load Whisper Model (can be "tiny", "base", "small", "medium")
            # Appending ".en" optimizes speed if you only expect English
            self.get_logger().info("Loading Whisper model...")
            self.model = whisper.load_model(self.model_path)
            self.get_logger().info("Model loaded")
        except Exception as e:
            self.get_logger().info(f"Failed to load Whisper model: {e}")
            self.stop_event.set()
            
            return
        self.get_logger().info("Audio Processor Node started.")
        #self.get_logger().info(f"test_data {self.BUFFER_SECONDS}")
    def init_wav_file(self):
        self.wav_file = wave.open(self.audio_file_path, 'wb')
        self.wav_file.setnchannels(self.channels)
        self.wav_file.setsampwidth(self.sample_width)
        self.wav_file.setframerate(self.sample_rate)
        
    def process_and_transcribe(self):
        """Converts the buffered audio and passes it to Whisper."""
        # Convert raw bytes to a 16-bit integer NumPy array
        #audio_int16 = np.frombuffer(self.audio_buffer, np.int16)
        
        # Convert to 32-bit float and normalize for Whisper
        #audio_float32 = np.array(self.complete_wav_file,dtype=np.float32) * 32768.0
        #speech_16k = librosa.resample(audio_float32, orig_sr=self.RATE,target_sr=self.WHISPER_RATE)
        #sf.write(f"audio_seg_{self.count}.wav",audio_float32, self.RATE)
        # Pass to Whisper
        audio, sample_rate = sf.read(self.audio_file_path)
        audio = audio.astype(np.float32)
        speech_16k = librosa.resample(audio, orig_sr=48000,target_sr=16000)
        result = self.model.transcribe(speech_16k, fp16=False)
        text = result['text'].strip()
        
        # Publish the result if actual text was generated
        if text:
            return text
        else:
            self.get_logger().info('Buffer transcribed, but no words were recognized (likely noise).')

    
    def audio_callback(self, msg):
        # Simply ingest data quickly into a thread-safe queue
        audio_data = json.loads(msg.data)
        avg_descriptor_data = audio_data["descriptor"]
        try:
            result_text = self.process_and_transcribe()
            
            
            payload = {"content": result_text, "descriptor": avg_descriptor_data}

            self.get_logger().info(result_text)

            self.content_pub.publish(String(data=json.dumps(payload)))
            
        except:
            pass
        #audio_int16 = np.frombuffer(self.audio_buffer, np.int16)
        #audio_float32 = audio_int16.astype(np.float32) 
        
        #self.audio_buffer.clear()
        #self.wav_file.clear()
        self.init_wav_file()
        
    def process_and_transcribe_base64(self, audio_b64, sample_rate, sample_width):
        """Decodes Base64 audio into memory and passes it to Whisper."""
        try:
            # 1. Decode Base64 string back to bytes
            audio_bytes = base64.b64decode(audio_b64)
            
            # 2. Convert bytes to 16-bit integer NumPy array
            if sample_width == 2:
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
            else:
                self.get_logger().error(f"Unsupported sample width: {sample_width}")
                return ""
            
            # 3. Convert to 32-bit float and normalize [-1.0, 1.0] for Whisper
            audio_float32 = audio_int16.astype(np.float32) / 32768.0
            
            # 4. Resample if the input rate doesn't match Whisper's required 16kHz
            if sample_rate != self.WHISPER_RATE:
                audio_float32 = librosa.resample(audio_float32, orig_sr=sample_rate, target_sr=self.WHISPER_RATE)
            
            # 5. Pass directly to Whisper from memory
            result = self.model.transcribe(audio_float32, fp16=False)
            text = result['text'].strip()
            
            if text:
                return text
            else:
                self.get_logger().info('Buffer transcribed, but no words were recognized (likely noise).')
                return ""
                
        except Exception as e:
            self.get_logger().error(f"Error during transcription processing: {e}")
            return ""

    def audio_callback_base64(self, msg):
        try:
            audio_data = json.loads(msg.data)
            avg_descriptor_data = audio_data.get("descriptor", {})
            audio_b64 = audio_data.get("audio_b64", "")
            sample_rate = audio_data.get("sample_rate", 16000)
            sample_width = audio_data.get("sample_width", 2)
            
            # Only process if audio data is present
            if audio_b64:
                result_text = self.process_and_transcribe_base64(audio_b64, sample_rate, sample_width)
                
                if result_text:
                    # Forward the final text and descriptor to the next node
                    payload = {"content": result_text, "descriptor": avg_descriptor_data}
                    self.get_logger().info(f"Transcribed: {result_text}")
                    self.content_pub.publish(String(data=json.dumps(payload)))
                    
        except Exception as e:
            self.get_logger().error(f"Callback error: {traceback.format_exc()}")
def main(args=None):
    rclpy.init(args=args)

    recorder = AudioProcessorNode()

    def signal_handler(sig, frame):
        recorder.close()
        rclpy.shutdown()
        sys.exit(0)

    #signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(recorder)


if __name__ == '__main__':
    main()