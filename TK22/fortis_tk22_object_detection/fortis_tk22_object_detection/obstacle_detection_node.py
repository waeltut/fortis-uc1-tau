"""ROS 2 node for TK2.2 obstacle detection (BEKO pilot).

Subscribes to the raw RealSense color (rgb8/bgr8) and depth (16UC1) `Image` topics,
for now replayed from the BEKO rosbags with `ros2 bag play`, pairs them by header
stamp (the config's `sync` block, same rule as the offline `BagSource`), runs
`ObstaclePipeline` on the newest pair and publishes the path result. The pipeline
is loaded and warmed up once, before subscribing. It runs in a worker thread, so the
subscriptions keep pairing while a frame is processed; pairs that arrive in the
meantime are dropped (latest frame wins).
"""

import json
import threading
from pathlib import Path
from time import perf_counter
from typing import TextIO

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from loguru import logger
from object_detection.obstacle.output import draw_overlay
from object_detection.obstacle.pipeline import (
    FrameResult,
    ObstaclePipeline,
    TrackingStats,
)
from object_detection.obstacle.sync import (
    NS_PER_S,
    PairedFrame,
    StreamPairer,
    load_sync_config,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from fortis_interfaces.msg import ObstacleDetection
from std_msgs.msg import Header

from fortis_tk22_object_detection.images import (
    image_to_bgr,
    image_to_depth,
    to_compressed,
)
from fortis_tk22_object_detection.interfaces import (
    DEFAULT_ANNOTATED_TOPIC,
    DEFAULT_COLOR_TOPIC,
    DEFAULT_DEPTH_TOPIC,
    DEFAULT_OUTPUT_TOPIC,
    load_registry,
    resolve_qos,
)

CONFIG_DIR = (
    Path(get_package_share_directory("fortis_tk22_object_detection")) / "config"
)
ANNOTATED_JPEG_QUALITY = 90
STATS_EVERY = 100  # processed frames between stats log lines


def stamp_ns(header: Header) -> int:
    return header.stamp.sec * NS_PER_S + header.stamp.nanosec


def calibration_size(camera_config: str) -> tuple[int, int]:
    """Color (width, height) of the camera config, the expected frame size."""
    with open(camera_config) as f:
        color = yaml.safe_load(f)["color"]
    return int(color["width"]), int(color["height"])


BUCKETS = {
    "dynamic": ObstacleDetection.BUCKET_DYNAMIC,
    "static": ObstacleDetection.BUCKET_STATIC,
}


def record_to_msgs(record: dict, header: Header) -> list[ObstacleDetection]:
    """Map one pipeline record to the output messages.

    The only place that knows the output message shape. Filled from the record's
    `path` block (the closest in-path object); `header` is the color frame's.
    """
    msg = ObstacleDetection()
    msg.header = header
    msg.track_id = -1
    path = record["path"]
    if path is None:
        msg.depth_status = ObstacleDetection.DEPTH_MISSING
        return [msg]
    msg.depth_status = ObstacleDetection.DEPTH_OK
    msg.path_blocked = path["path_blocked"]
    closest = path["closest"]
    if closest is not None:
        msg.track_id = closest["track_id"]
        msg.object_class = closest["class"]
        msg.bucket = BUCKETS[closest["bucket"]]
        msg.distance_m = closest["distance_m"]
        msg.lateral_m = closest["lateral_m"]
    return [msg]


class ObstacleDetectionNode(Node):
    """Pairs color and depth, runs the obstacle pipeline, publishes the path."""

    def __init__(self) -> None:
        super().__init__("obstacle_detection")
        logger.info("Initializing ObstacleDetectionNode...")
        self.declare_parameter("config", str(CONFIG_DIR / "beko.yaml"))
        self.declare_parameter("camera_config", str(CONFIG_DIR / "fast_camera.yaml"))
        self.declare_parameter("tracker", "")  # empty: the config's tracker
        self.declare_parameter("use_engine", False)    # This should default to True, but I just want to get this working
        self.declare_parameter("color_topic", DEFAULT_COLOR_TOPIC)
        self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("publish_annotated", False)
        self.declare_parameter("annotated_topic", DEFAULT_ANNOTATED_TOPIC)
        self.declare_parameter("jsonl_path", "")  # empty: no JSONL

        config = self._param("config")
        camera_config = self._param("camera_config")
        color_topic = self._param("color_topic")
        depth_topic = self._param("depth_topic")
        output_topic = self._param("output_topic")
        annotated_topic = self._param("annotated_topic")
        publish_annotated = self._param("publish_annotated")
        jsonl_path = self._param("jsonl_path")

        # Validate topics and configs before the (slow) model loading.
        registry = load_registry()
        color_qos = resolve_qos(registry, color_topic, Image, required=False)
        depth_qos = resolve_qos(registry, depth_topic, Image, required=False)
        output_qos = resolve_qos(registry, output_topic, ObstacleDetection)
        self._pairer = StreamPairer(load_sync_config(config))
        self._frame_size = calibration_size(camera_config)

        construct_start = perf_counter()
        self._pipeline = ObstaclePipeline(
            config,
            self._frame_size,
            self._param("tracker") or None,
            camera_config,
            use_engine=self._param("use_engine"),
        )
        logger.info(f"Pipeline constructed in {perf_counter() - construct_start:.2f} s")
        self._pipeline.warmup()

        self._tracking_stats = TrackingStats()
        self._processed = 0
        self._first_stamp_ns: int | None = None
        self._resets_seen = 0
        self._run_start = 0.0
        self._jsonl: TextIO | None = None
        if jsonl_path:
            Path(jsonl_path).parent.mkdir(parents=True, exist_ok=True)
            self._jsonl = open(jsonl_path, "w", buffering=1)
            logger.info(f"Writing per-frame records to {jsonl_path}")

        self._publisher = self.create_publisher(
            ObstacleDetection, output_topic, output_qos
        )
        self._annotated_publisher = None
        if publish_annotated:
            annotated_qos = resolve_qos(
                registry, annotated_topic, CompressedImage, required=False
            )
            self._annotated_publisher = self.create_publisher(
                CompressedImage, annotated_topic, annotated_qos
            )

        # Guards the pairer; the worker waits on it for a paired frame.
        self._condition = threading.Condition()
        self._stopping = False
        self._worker = threading.Thread(target=self._run, name="obstacle_pipeline")
        self._worker.start()

        self.create_subscription(Image, color_topic, self._color_callback, color_qos)
        self.create_subscription(Image, depth_topic, self._depth_callback, depth_qos)
        logger.info(
            f"ObstacleDetectionNode ready. Subscribed to '{color_topic}' and "
            f"'{depth_topic}', publishing on '{output_topic}'"
            + (f" and '{annotated_topic}'." if publish_annotated else ".")
        )

    def _param(self, name: str) -> str | bool:
        return self.get_parameter(name).value

    def _color_callback(self, msg: Image) -> None:
        with self._condition:
            self._pairer.add_color(stamp_ns(msg.header), msg)
            self._condition.notify()

    def _depth_callback(self, msg: Image) -> None:
        with self._condition:
            self._pairer.add_depth(stamp_ns(msg.header), msg)
            self._condition.notify()

    def _run(self) -> None:
        try:
            while True:
                with self._condition:
                    frame = self._pairer.pop()
                    while frame is None and not self._stopping:
                        self._condition.wait()
                        frame = self._pairer.pop()
                    if self._stopping:
                        return
                    resets = self._pairer.resets
                self._process(frame, resets)
        except Exception:
            logger.exception("Obstacle pipeline failed, shutting down.")
            rclpy.try_shutdown()

    def _process(self, frame: PairedFrame, resets: int) -> None:
        try:
            color = image_to_bgr(frame.color)
            depth = None if frame.depth is None else image_to_depth(frame.depth)
        except ValueError as e:
            logger.warning(f"Skipping frame: {e}")
            return
        height, width = color.shape[:2]
        if (width, height) != self._frame_size:
            raise ValueError(
                f"Color frame {width}x{height} does not match the camera config "
                f"{self._frame_size[0]}x{self._frame_size[1]}."
            )

        # Record timestamps start at 0 like the offline runs, and restart with the
        # pairing, e.g. when a bag loops.
        if self._first_stamp_ns is None or resets != self._resets_seen:
            self._first_stamp_ns = frame.stamp_ns
            self._resets_seen = resets
        timestamp = (frame.stamp_ns - self._first_stamp_ns) / NS_PER_S

        process_start = perf_counter()
        if self._processed == 0:
            self._run_start = process_start
        result = self._pipeline.process(timestamp, color, depth)
        if self._processed == 0:
            first_ms = 1000 * (perf_counter() - process_start)
            logger.info(f"First frame processed in {first_ms:.1f} ms")

        for msg in record_to_msgs(result.record, frame.color.header):
            self._publisher.publish(msg)
        if self._annotated_publisher is not None:
            self._publish_annotated(result, color, frame.color.header)
        if self._jsonl is not None:
            self._jsonl.write(json.dumps(result.record) + "\n")

        self._tracking_stats.update(result.detections)
        self._processed += 1
        if self._processed % STATS_EVERY == 0:
            self._log_stats()

    def _publish_annotated(
        self, result: FrameResult, color: np.ndarray, header: Header
    ) -> None:
        state = self._pipeline.state
        assert state is not None and result.z_map is not None
        assert result.assessments is not None and result.path is not None
        overlay = draw_overlay(
            result.frame_index,
            color,
            result.z_map,
            result.detections,
            result.assessments,
            result.path,
            state.corridor,
            state.calibration.color,
        )
        self._annotated_publisher.publish(
            to_compressed(
                overlay,
                header,
                "jpeg",
                [cv2.IMWRITE_JPEG_QUALITY, ANNOTATED_JPEG_QUALITY],
            )
        )

    def _log_stats(self) -> None:
        processed = self._processed
        elapsed = perf_counter() - self._run_start
        with self._condition:
            sync = self._pairer.stats()
        stage_ms = {
            stage: 1000 * seconds / processed
            for stage, seconds in self._pipeline.timer.totals.items()
        }
        logger.info(
            f"Processed {processed} frames, {processed / elapsed:.2f} FPS; "
            "stage ms/frame: "
            + ", ".join(f"{k}={v:.1f}" for k, v in stage_ms.items())
            + f"; sync: {sync}"
        )

    def stop(self) -> None:
        """Stop the worker after its current frame and log the run summary."""
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        if self._worker.is_alive():
            self._worker.join()
        if self._jsonl is not None:
            self._jsonl.close()
            self._jsonl = None
        if self._processed:
            self._log_stats()
            logger.info(f"Tracking stats: {self._tracking_stats.summary()}")


def main(args: list[str] | None = None) -> None:
    """Entrypoint for the obstacle detection node."""
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("Shutting down...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
