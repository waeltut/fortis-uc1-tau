import os
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String
from text_to_speech.audio import AudioSink, SpeakerSink, TopicSink
from text_to_speech.backend import PiperBackend, TtsBackend
from text_to_speech.config import OUTPUTS, load_config, load_env_file
from text_to_speech.message import RobotMessage, parse

from fortis_tk24_tts.interfaces import load_registry, resolve_qos

# Topics and config file can be set through TK24_TTS_* environment variables
# (deployment, or a .env file loaded in main()); the tts.yaml values are
# overridden in load_config().
DEFAULT_INPUT_TOPIC = "/fortis/tk23/robot_to_human_message"
DEFAULT_OUTPUT_TOPIC = "/fortis/tk24/tts_audio"
ENV_INPUT_TOPIC = "TK24_TTS_INPUT_TOPIC"
ENV_OUTPUT_TOPIC = "TK24_TTS_OUTPUT_TOPIC"
ENV_CONFIG_FILE = "TK24_TTS_CONFIG_FILE"
QUEUE_SIZE = 10
WARMUP_TEXT = "Warm up."
WARMUP_RUNS = 2


class TtsNode(Node):
    def __init__(self) -> None:
        super().__init__("tts_node")

        # Precedence: ROS params > TK24_TTS_* env vars > tts.yaml
        default_config = os.environ.get(ENV_CONFIG_FILE) or (
            get_package_share_directory("fortis_tk24_tts") + "/config/tts.yaml"
        )
        self.declare_parameter("config_path", default_config)
        self.declare_parameter("output", "")  # empty = use the env/YAML value
        self.declare_parameter(
            "input_topic", os.environ.get(ENV_INPUT_TOPIC) or DEFAULT_INPUT_TOPIC
        )
        self.declare_parameter(
            "output_topic", os.environ.get(ENV_OUTPUT_TOPIC) or DEFAULT_OUTPUT_TOPIC
        )
        self._registry = load_registry()

        config_path = self.get_parameter("config_path").value
        self._cfg = load_config(config_path, env=os.environ)  # .env loaded in main()
        output = self.get_parameter("output").value or self._cfg.output
        if output not in OUTPUTS:
            raise ValueError(f"'output' must be one of {OUTPUTS}, got '{output}'")
        self.get_logger().info(
            f"Config {config_path}: language={self._cfg.language}, output={output}"
        )

        self._backend: TtsBackend = PiperBackend()
        self._backend.load(self._cfg.voice)
        self._warm_up()
        self._sink = self._create_sink(output)

        self._queue: deque[RobotMessage] = deque(maxlen=QUEUE_SIZE)
        self._cond = threading.Condition()
        self._interrupt = threading.Event()
        self._shutdown = False
        self._worker = threading.Thread(target=self._run_worker, daemon=True)
        self._worker.start()

        input_topic = self.get_parameter("input_topic").value
        qos = resolve_qos(self._registry, input_topic, "std_msgs/msg/String")
        self.create_subscription(String, input_topic, self._on_message, qos)
        self.get_logger().info(f"Ready, listening on {input_topic}")

    def _warm_up(self) -> None:
        for _ in range(WARMUP_RUNS):
            for _chunk in self._backend.stream(WARMUP_TEXT):
                pass

    def _create_sink(self, output: str) -> AudioSink:
        if output == "speaker":
            return SpeakerSink(self._backend.sample_rate, self._cfg.audio_device)

        topic = self.get_parameter("output_topic").value
        msg_type = (
            Float32MultiArray
            if self._cfg.topic_format == "float32"
            else Int16MultiArray
        )
        qos = resolve_qos(self._registry, topic, f"std_msgs/msg/{msg_type.__name__}")
        publisher = self.create_publisher(msg_type, topic, qos)

        def publish(samples: np.ndarray) -> None:
            msg = msg_type()
            msg.data = samples.tolist()
            publisher.publish(msg)

        self.get_logger().info(
            f"Publishing {self._cfg.topic_format} audio on {topic} "
            f"(sample rate {self._backend.sample_rate} Hz)"
        )
        return TopicSink(publish, self._cfg.topic_format)

    def _on_message(self, msg: String) -> None:
        try:
            message = parse(msg.data)
        except ValueError as e:  # JSONDecodeError is a ValueError
            self.get_logger().error(f"Ignoring invalid message ({e}): {msg.data!r}")
            return

        with self._cond:
            if message.urgent:
                self._queue.clear()
                self._interrupt.set()
            elif len(self._queue) == self._queue.maxlen:
                self.get_logger().warning("Queue full, dropping oldest message")
            self._queue.append(message)
            self._cond.notify()

    def _run_worker(self) -> None:
        while True:
            with self._cond:
                self._cond.wait_for(lambda: self._queue or self._shutdown)
                if self._shutdown:
                    return
                message = self._queue.popleft()
                self._interrupt.clear()
            try:
                self._speak(message)
            except Exception as e:
                self.get_logger().error(f"Synthesis failed: {e!r}")

    def _speak(self, message: RobotMessage) -> None:
        start = time.perf_counter()
        first_audio: float | None = None
        synth_time = 0.0
        audio_bytes = 0
        interrupted = False

        chunks = iter(self._backend.stream(message.content))
        while True:
            if self._interrupt.is_set():
                interrupted = True
                break
            t0 = time.perf_counter()
            chunk = next(chunks, None)
            synth_time += time.perf_counter() - t0
            if chunk is None:
                break
            if first_audio is None:
                first_audio = time.perf_counter() - start
            self._sink.write(chunk, self._interrupt.is_set)
            audio_bytes += len(chunk)

        if interrupted:
            chunks.close()
            self._sink.stop()

        duration = audio_bytes / 2 / self._backend.sample_rate
        rtf = synth_time / duration if duration else float("nan")
        first = f"{first_audio:.3f}s" if first_audio is not None else "n/a"
        self.get_logger().info(
            f"language={self._cfg.language} text={message.content!r} "
            f"first_audio={first} synth={synth_time:.3f}s "
            f"audio={duration:.3f}s rtf={rtf:.3f}"
            f"{' INTERRUPTED' if interrupted else ''}"
        )

    def shutdown(self) -> None:
        with self._cond:
            self._shutdown = True
            self._queue.clear()
            self._cond.notify()
        self._interrupt.set()
        self._worker.join(timeout=2.0)
        close = getattr(self._sink, "close", None)
        if close:
            close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        load_env_file()
        node = TtsNode()
    except Exception as e:
        # Fail clearly and non-zero, so a deployment container does not run idle
        rclpy.logging.get_logger("tts_node").fatal(f"Initialization failed: {e}")
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
