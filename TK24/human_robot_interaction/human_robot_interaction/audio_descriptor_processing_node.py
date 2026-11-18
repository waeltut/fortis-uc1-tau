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
import queue
import soundfile as sf

from rclpy.node import Node
from fortis_interfaces.msg import Mic as mic_msg
from std_msgs.msg import String, Float32
import numpy as np
import json
import librosa
from collections import deque
import noisereduce
import wave
import signal
from ament_index_python.packages import get_package_share_directory
import base64
from scipy.fft import fft, ifft
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
            return k_factor * diff if diff > 0 else np.finfo(float).eps
        else:
            # Prevent division by zero or log(0) errors
            strength_safe = np.maximum(physical_strength, 0.001)

            # The ratio S / S0
            ratio = optimal_threshold / strength_safe if inverse else strength_safe / optimal_threshold

            # Only apply intensity scaling if the ratio exceeds the baseline (1.0)
            return k_factor * np.log(ratio) if ratio > 1.0 else np.finfo(float).eps

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

class VolumeDescriptor(AudioDescriptor):
    """Calculates the RMS volume of the *original* signal in dB."""

    def __init__(self):
        super().__init__("audio_volume")

    def process(self, audio_float, audio_int, removed_noise_float, rate):
        rms_float = np.sqrt(np.mean(audio_float ** 2))
        volume_level_db = 20 * log10(rms_float) if rms_float > 0 else -100
        return volume_level_db
    
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
        super().__init__('audio_descriptor_processing_node')
        
        # Subscriptions
        self.create_subscription(mic_msg, '/mic/raw_audio', self.audio_callback_nr, 100)
        
        # Publishers
        self.content_pub = self.create_publisher(String, '/cognition/audio/descriptor_segment', 10)
        self.desc_pub = self.create_publisher(String, '/cognition/audio/descriptors', 10)
        
        
        self.noise_pub = self.create_publisher(Float32, '/cognition/audio/noise_pub', 10)
        self.volume_pub = self.create_publisher(Float32, '/cognition/audio/volume_pub', 10)
        
        # Audio State
        #self.RATE = 16000 # Match your publisher
        #self.audio_buffer = deque(maxlen=self.RATE * 5) # 5 second rolling buffer
        self.descriptors = [NoiseDescriptor(), VolumeDescriptor()]
        #self.BUFFER_SECONDS = 2.0
        self.count = 0
        # Whisper requires 16000 Hz
        self.state = "SILENCE"
        self.speech_chunks = []
        self.silent_chunks_count = 0
        self.desc_during_recording = []
        self.channels = 1
        self.sample_width = 2      # bytes (16-bit audio)
        #self.sample_rate = 16000   # Hz
        self.intensity_noise_pub = self.create_publisher(Float32, '/cognition/audio/intensity_noise_pub', 10)
        self.intensity_volume_pub = self.create_publisher(Float32, '/cognition/audio/intensity_volume_pub', 10)
        package_share_directory = get_package_share_directory('human_robot_interaction')
        self.audio_file_path = os.path.join(package_share_directory, "recorded_audio.wav")
        
        
        # Processing Timer (Runs at ~10Hz to unblock the subscriber)
        #self.create_timer(0.1, self.process_audio_loop)
        
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('buffer_seconds', 2.5)
        #self.declare_parameter('noise_reduction_factor', 1.3)
        self.declare_parameter('channels', 1)
        self.declare_parameter('sample_width', 2)
        
        self.declare_parameter('base_S_noise', -50.0)
        self.declare_parameter('k_noise', 0.2)
        self.declare_parameter('base_S_volume', -50.0)
        self.declare_parameter('k_volume', 0.1)

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.RATE = self.sample_rate 
        self.BUFFER_SECONDS = self.get_parameter('buffer_seconds').get_parameter_value().double_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.sample_width = self.get_parameter('sample_width').get_parameter_value().integer_value
        
        self.base_S_noise = self.get_parameter('base_S_noise').get_parameter_value().double_value
        self.k_noise = self.get_parameter('k_noise').get_parameter_value().double_value
        self.base_S_volume = self.get_parameter('base_S_volume').get_parameter_value().double_value
        self.k_volume = self.get_parameter('k_volume').get_parameter_value().double_value
        self.get_logger().info(str(self.k_volume))
        self.get_logger().info(str(self.base_S_volume))
        self.init_wav_file()
        #no wav file for now, i use base 64 to encode
        self.noise_profile = None
        
    def init_wav_file(self):
        
        self.wav_file = wave.open(self.audio_file_path, 'wb')
        self.wav_file.setnchannels(self.channels)
        self.wav_file.setsampwidth(self.sample_width)
        self.wav_file.setframerate(self.sample_rate)
        
    def audio_callback_ff(self, msg):
            # --- 0. Safe Data Ingestion ---
            # Parse floats and handle NaNs/Infs BEFORE scaling to prevent overflow
            raw_floats = np.array(msg.data, dtype=np.float32)
            clean_floats = np.nan_to_num(raw_floats, nan=0.0, posinf=1.0, neginf=-1.0)
            
            # Remove DC Offset to prevent transient clicking
            clean_floats = clean_floats - np.mean(clean_floats)

            # --- 1. Noise Reduction via Fourier Transform Spectral Gating ---
            # Transform to frequency domain
            fft_signal = fft(clean_floats)
            magnitude = np.abs(fft_signal)
            
            # Estimate noise threshold dynamically 
            # (Increase the 1.5 multiplier if too much noise still gets through)
            #noise_threshold = np.mean(magnitude) * 1
            if self.noise_profile is None:
                self.noise_profile = magnitude.copy()
            else:
                # Create a boolean mask where current magnitude is lower than the profile
                mask_lower = magnitude < self.noise_profile
                
                # Apply Asymmetric EMA
                # alpha_fast = 0.90 (adapts quickly to a quieter room)
                # alpha_slow = 0.995 (resists being skewed upward by human speech)
                self.noise_profile[mask_lower] = 0.90 * self.noise_profile[mask_lower] + 0.10 * magnitude[mask_lower]
                self.noise_profile[~mask_lower] = 0.995 * self.noise_profile[~mask_lower] + 0.005 * magnitude[~mask_lower]
            
            # Calculate dynamic threshold per frequency bin
            # (A multiplier of 2.0 to 2.5 is mathematically standard for oversubtraction)
            noise_threshold = self.noise_profile * 2.0 
            
            # Apply Hard Gating
            fft_signal_cleaned = np.where(magnitude > noise_threshold, fft_signal, 0.0)
            
            # Transform back to time domain
            cleaned_float = np.real(ifft(fft_signal_cleaned)).astype(np.float32)
                
            # Calculate removed noise for the descriptor
            removed_noise_float = clean_floats - cleaned_float

            # --- 2. Safe Conversion to Int16 ---
            # Scale and STRICTLY clip to prevent integer wrap-around (the cause of loud static)
            audio_data_int_scaled = np.clip(clean_floats * 32767.0, -32768, 32767)
            audio_data_int = audio_data_int_scaled.astype(np.int16)
            
            cleaned_int_scaled = np.clip(cleaned_float * 32767.0, -32768, 32767)
            cleaned = cleaned_int_scaled.astype(np.int16)

            # Buffer appends
            audio_bytes = audio_data_int.tobytes()

            # VAD (Voice Activity Detection) Calculation
            rms = np.sqrt(np.mean(cleaned.astype(np.float64) ** 2))
            raw_db = 20 * np.log10(rms) if rms > 0 else -100
            is_speech = raw_db > 60

            # --- 3. Descriptors ---
            descriptor_data = {}
            for desc in self.descriptors:
                res = desc.process(cleaned, audio_data_int, removed_noise_float, self.RATE)
                raw_val = res[0] if isinstance(res, tuple) else res
                # Explicitly cast to native Python float to satisfy ROS 2 assertions
                descriptor_data[desc.name] = float(raw_val)
                
            descriptor_data["state"] = self.state
            
            # Publishing Descriptors
            self.desc_pub.publish(String(data=json.dumps(descriptor_data)))
            
            noise_msg = Float32()
            noise_msg.data = descriptor_data["audio_noise"]
            self.noise_pub.publish(noise_msg)

            intensity_noise = float(calculate_fechner_intensity(descriptor_data["audio_noise"], self.base_S_noise , self.k_noise, True, True))
            intensity_noise_msg = Float32()
            intensity_noise_msg.data = intensity_noise
            self.intensity_noise_pub.publish(intensity_noise_msg)
            
            volume_msg = Float32()
            volume_msg.data = descriptor_data["audio_volume"]
            self.volume_pub.publish(volume_msg)
            
            intensity_volume = float(calculate_fechner_intensity(descriptor_data["audio_volume"], self.base_S_volume , self.k_volume, True, False))
            intensity_volume_msg = Float32()
            intensity_volume_msg.data = intensity_volume
            self.intensity_volume_pub.publish(intensity_volume_msg)

            self.wav_file.writeframes(cleaned)

            # --- 4. Accumulation & Whisper Logic ---
            if self.state == "SILENCE" and is_speech:
                self.state = "SPEAKING"
                self.current_speech_buffer = bytearray()
                self.current_speech_buffer.extend(audio_bytes)
                self.desc_during_recording = [descriptor_data]
                self.silent_chunks_count = 0
                
            elif self.state == "SPEAKING":
                self.current_speech_buffer.extend(audio_bytes)
                self.desc_during_recording.append(descriptor_data)
                self.silent_chunks_count = 0 if is_speech else self.silent_chunks_count + 1
                
                if self.silent_chunks_count > 10: # Silence frames
                    self.state = "SILENCE"
                    
                    avg_descriptor_data={}
                    for desc in list(self.desc_during_recording[0].keys()):
                        if desc not in ["state","attention_multiplier"]:
                            avg_descriptor_data[desc] = np.average([inst[desc] for inst in self.desc_during_recording])
                            
                    encoded_audio = base64.b64encode(self.current_speech_buffer).decode('utf-8')
                    payload = {
                        "content": "", 
                        "descriptor": avg_descriptor_data,
                        "audio_b64": encoded_audio,
                        "sample_rate": self.RATE,
                        "sample_width": self.sample_width
                    }

                    self.content_pub.publish(String(data=json.dumps(payload)))
                    self.count += 1
    def audio_callback_nr(self, msg):
        # --- 0. Safe Data Ingestion ---
        # Parse floats and handle NaNs/Infs BEFORE scaling
        try:
            # 1. Cast the ROS message data directly to a Python byte string
            raw_bytes = bytes(msg.data)
            
            # 2. Decode the raw bytes directly into 16-bit integers (Little Endian)
            audio_data_int = np.frombuffer(raw_bytes, dtype=np.int16)
            
            # 3. Convert to float32 and mathematically normalize to [-1.0, 1.0]
            clean_floats = audio_data_int.astype(np.float32) / 32768.0
            
            # 4. Remove DC Offset
            clean_floats = clean_floats - np.mean(clean_floats)
            
        except Exception as e:
            self.get_logger().error(f"Data Ingestion Error: {e}")
            return

        # --- 1. Noise Reduction via noisereduce ---
        cleaned_float = noisereduce.reduce_noise(y=clean_floats, sr=48000, stationary=True)
        
        # Calculate removed noise for the descriptor
        removed_noise_float = clean_floats - cleaned_float

        # --- 2. Safe Conversion to Int16 ---
        audio_data_int_scaled = np.clip(clean_floats * 32767.0, -32768, 32767)
        audio_data_int = audio_data_int_scaled.astype(np.int16)
        
        cleaned_int_scaled = np.clip(cleaned_float * 32767.0, -32768, 32767)
        cleaned = cleaned_int_scaled.astype(np.int16)
        # Buffer appends
        audio_bytes = audio_data_int.tobytes()

        # VAD (Voice Activity Detection) Calculation
        rms = np.sqrt(np.mean(cleaned.astype(np.float64) ** 2))
        raw_db = 20 * np.log10(rms) if rms > 0 else -100
        is_speech = raw_db > 60

        # --- 3. Descriptors ---
        descriptor_data = {}
        for desc in self.descriptors:
            res = desc.process(cleaned, audio_data_int, removed_noise_float, self.RATE)
            raw_val = res[0] if isinstance(res, tuple) else res
            # Explicit cast to native Python float for ROS 2 compatibility
            descriptor_data[desc.name] = float(raw_val)
            
        descriptor_data["state"] = self.state
        
        # Publishing Descriptors
        self.desc_pub.publish(String(data=json.dumps(descriptor_data)))
        
        noise_msg = Float32()
        noise_msg.data = float(descriptor_data["audio_noise"])
        self.noise_pub.publish(noise_msg)

        intensity_noise = float(calculate_fechner_intensity(descriptor_data["audio_noise"], self.base_S_noise , self.k_noise, True, True))
        intensity_noise_msg = Float32()
        intensity_noise_msg.data = intensity_noise
        self.intensity_noise_pub.publish(intensity_noise_msg)
        
        volume_msg = Float32()
        volume_msg.data = float(descriptor_data["audio_volume"])
        self.volume_pub.publish(volume_msg)
        
        intensity_volume = float(calculate_fechner_intensity(descriptor_data["audio_volume"], self.base_S_volume , self.k_volume, True, False))
        intensity_volume_msg = Float32()
        intensity_volume_msg.data = intensity_volume
        self.intensity_volume_pub.publish(intensity_volume_msg)

        self.wav_file.writeframes(cleaned_float)

        # --- 4. Accumulation & Whisper Logic ---
        if self.state == "SILENCE" and is_speech:
            self.state = "SPEAKING"
            self.current_speech_buffer = bytearray()
            self.current_speech_buffer.extend(audio_bytes)
            self.desc_during_recording = [descriptor_data]
            self.silent_chunks_count = 0
            
        elif self.state == "SPEAKING":
            self.current_speech_buffer.extend(audio_bytes)
            self.desc_during_recording.append(descriptor_data)
            self.silent_chunks_count = 0 if is_speech else self.silent_chunks_count + 1
            
            if self.silent_chunks_count > 10: # Silence frames
                self.state = "SILENCE"
                
                avg_descriptor_data={}
                for desc in list(self.desc_during_recording[0].keys()):
                    if desc not in ["state","attention_multiplier"]:
                        avg_descriptor_data[desc] = float(np.average([inst[desc] for inst in self.desc_during_recording]))
                        
                encoded_audio = base64.b64encode(self.current_speech_buffer).decode('utf-8')
                payload = {
                    "content": "", 
                    "descriptor": avg_descriptor_data,
                    "audio_b64": encoded_audio,
                    "sample_rate": self.RATE,
                    "sample_width": self.sample_width
                }

                self.content_pub.publish(String(data=json.dumps(payload)))
                self.count += 1
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