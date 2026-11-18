#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fortis_interfaces.msg import Mic
import numpy as np
import wave


class AudioRecorder(Node):

    def __init__(self):
        super().__init__('audio_recorder')

        # Must match publisher settings
        self.sample_rate = 48000
        self.channels = 1

        self.audio_buffer = []

        self.subscription = self.create_subscription(
            Mic,
            'mic/raw_audio',
            self.audio_callback,
            10
        )

        self.get_logger().info("Listening on /mic/raw_audio")

    def audio_callback(self, msg):
        audio_chunk = np.array(msg.data, dtype=np.float32)
        self.audio_buffer.append(audio_chunk)

    def save_wav(self, filename="recorded_audio.wav"):
        if not self.audio_buffer:
            self.get_logger().warn("No audio received.")
            return

        audio = np.concatenate(self.audio_buffer)

        # Convert float32 [-1.0,1.0] -> int16
        audio = np.clip(audio, -1.0, 1.0)
        audio_int16 = (audio * 32767).astype(np.int16)

        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)  # int16 = 2 bytes
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())

        duration = len(audio) / self.sample_rate

        self.get_logger().info(
            f"Saved {filename} ({duration:.2f} seconds)"
        )


def main(args=None):
    rclpy.init(args=args)

    node = AudioRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.save_wav("recorded_audio.wav")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()