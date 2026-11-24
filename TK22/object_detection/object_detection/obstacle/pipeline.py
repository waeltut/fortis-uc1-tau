"""Obstacle pipeline: `ObstaclePipeline`, ROS-free and loaded once.

The ROS 2 node (`fortis_tk22_object_detection`) constructs it once, warms it up and
calls `process()` per paired color/depth frame. With depth assessment (a camera
config), each object gets a distance, lateral offset and in-path flag, and every
frame a path result.
"""

import statistics
from collections import defaultdict
from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from time import perf_counter

import numpy as np
from loguru import logger

from object_detection.obstacle.assess import (
    AssessmentConfig,
    ObjectAssessment,
    PathResult,
    TrackDebouncer,
    assess_object,
    load_assessment,
    load_buckets,
    load_debouncer,
    summarize_path,
)
from object_detection.obstacle.calibration import CameraCalibration, load_calibration
from object_detection.obstacle.detector import (
    DEVICE,
    MODEL_NAME,
    Detector,
    FrameDetections,
    load_config,
    load_model,
)
from object_detection.obstacle.engine import load_engine, load_engine_config
from object_detection.obstacle.geometry import (
    Corridor,
    ZMapBuilder,
    load_block,
    load_corridor,
)
from object_detection.obstacle.output import build_record

BUILTIN_TRACKERS = [
    "bytetrack",
    "botsort",
    "ocsort",
    "deepocsort",
    "fasttrack",
    "tracktrack",
]
WARMUP_KEYS = ("frames",)


def resolve_tracker(tracker: str) -> str:
    """Map a bare name like 'botsort' to the ultralytics default YAML."""
    if tracker not in BUILTIN_TRACKERS:
        return tracker
    logger.warning(
        f"Using the ultralytics default config for tracker '{tracker}', not a "
        "tuned YAML."
    )
    return f"{tracker}.yaml"


class TrackingStats:
    """Accumulates per-track length and class history over a run."""

    def __init__(self) -> None:
        self.lengths: dict[int, int] = {}
        # Per ID, classes in order of first appearance.
        self.classes: dict[int, list[str]] = {}

    def update(self, detections: FrameDetections) -> None:
        if detections.ids is None:
            return
        for track_id, cls in zip(detections.ids, detections.classes):
            track_id = int(track_id)
            self.lengths[track_id] = self.lengths.get(track_id, 0) + 1
            seen = self.classes.setdefault(track_id, [])
            if cls not in seen:
                seen.append(cls)

    def summary(self) -> dict:
        flips = {tid: names for tid, names in self.classes.items() if len(names) > 1}
        lengths = list(self.lengths.values())
        return {
            "unique_ids": len(lengths),
            "track_length_min": min(lengths, default=0),
            "track_length_median": statistics.median(lengths) if lengths else 0,
            "track_length_max": max(lengths, default=0),
            "ids_with_class_flips": len(flips),
            "class_flip_examples": [
                f"id {tid}: {'->'.join(names)}"
                for tid, names in list(flips.items())[:5]
            ],
        }


class StageTimer:
    """Accumulates wall time per named pipeline stage."""

    def __init__(self) -> None:
        self.totals: defaultdict[str, float] = defaultdict(float)

    def add(self, stage: str, seconds: float) -> None:
        self.totals[stage] += seconds

    @contextmanager
    def measure(self, stage: str) -> Generator[None]:
        start = perf_counter()
        try:
            yield
        finally:
            self.add(stage, perf_counter() - start)


def assess_frame(
    detections: FrameDetections,
    z_map: np.ndarray,
    calibration: CameraCalibration,
    corridor: Corridor,
    config: AssessmentConfig,
    debouncer: TrackDebouncer,
) -> list[ObjectAssessment]:
    """Assess every detection, then debounce in_path by track ID.

    The debouncer is updated on every frame so absent tracks age out. Detections
    without a track ID are not debounced and are never in path.
    """
    masks = [] if detections.masks is None else detections.masks
    assessments = [
        assess_object(z_map, mask, box, calibration.color, corridor, config)
        for mask, box in zip(masks, detections.boxes)
    ]
    raw = {}
    if detections.ids is not None:
        raw = {int(i): a.in_path_raw for i, a in zip(detections.ids, assessments)}
    in_path = debouncer.update(raw)
    if detections.ids is not None:
        for i, a in zip(detections.ids, assessments):
            a.in_path = in_path[int(i)]
    return assessments


@dataclass
class AssessmentState:
    """Everything needed to assess detections against depth, built once."""

    calibration: CameraCalibration
    corridor: Corridor
    config: AssessmentConfig
    debouncer: TrackDebouncer
    buckets: dict[str, str]
    build_z_map: ZMapBuilder


@dataclass
class FrameResult:
    """Result of `ObstaclePipeline.process`; the depth fields are None without depth
    assessment (no camera config).
    """

    frame_index: int
    timestamp: float
    detections: FrameDetections
    assessments: list[ObjectAssessment] | None
    z_map: np.ndarray | None
    path: PathResult | None
    record: dict  # JSONL record


class ObstaclePipeline:
    """Detector, tracker and depth assessment for one stream; ROS-free.

    Everything heavy is loaded in the constructor. Call `warmup()` once before the
    first frame, then `process()` per frame. Depth is assessed when `camera_config`
    is given. Every frame gets the config's `jpeg_quality` round trip (null = off).
    `use_engine` runs a cached TensorRT engine (exported on first use, see the
    config's `engine` block) instead of the `.pt` model.
    """

    def __init__(
        self,
        config_path: str,
        frame_size: tuple[int, int],
        tracker: str | None = None,
        camera_config: str | None = None,
        use_engine: bool = False,
    ) -> None:
        self.config = load_config(config_path)
        tracker = tracker or self.config["tracker"]
        if not tracker:
            raise ValueError(
                "No tracker YAML: set 'tracker' in the config or pass a tracker."
            )
        self.tracker = resolve_tracker(tracker)
        self.frame_size = frame_size
        self.warmup_frames = int(
            load_block(config_path, "warmup", WARMUP_KEYS)["frames"]
        )
        self.warmup_time_s = 0.0
        self.timer = StageTimer()
        self._frame_index = 0

        jpeg_quality = self.config["jpeg_quality"]
        logger.info(
            f"Model: {MODEL_NAME} ({'TensorRT engine' if use_engine else 'PyTorch'}), "
            f"conf: {self.config['conf']}, imgsz: {self.config['imgsz']}, "
            f"tracker: {self.tracker}, JPEG re-encode quality: {jpeg_quality}"
        )
        if use_engine:
            model = load_engine(
                self.config["objects"],
                self.config["imgsz"],
                load_engine_config(config_path),
            )
        else:
            model = load_model(self.config["objects"], self.config["imgsz"])
        self.detector = Detector(
            model,
            self.tracker,
            self.config["conf"],
            self.config["imgsz"],
            jpeg_quality,
        )

        self.state: AssessmentState | None = None
        if camera_config is not None:
            calibration = load_calibration(camera_config, frame_size)
            logger.info(
                f"Calibration: depth_aligned={calibration.depth_aligned}, "
                f"depth_scale={calibration.depth_scale}"
            )
            corridor = load_corridor(config_path)
            assessment = load_assessment(config_path)
            debouncer = load_debouncer(config_path)
            buckets = load_buckets(config_path, self.config["objects"])
            logger.info(f"Corridor: {corridor}, assessment: {assessment}")
            logger.info(
                f"Debouncer: k={debouncer.k}, n={debouncer.n}, buckets: {buckets}"
            )
            self.state = AssessmentState(
                calibration,
                corridor,
                assessment,
                debouncer,
                buckets,
                ZMapBuilder(calibration, DEVICE),
            )

    def warmup(self) -> None:
        """Pay the first-call costs (CUDA, model, Z-map kernels) on blank frames.

        Leaves no state behind: no tracker exists yet and the debouncer is empty.
        """
        start = perf_counter()
        width, height = self.frame_size
        self.detector.warmup(
            np.zeros((height, width, 3), dtype=np.uint8), self.warmup_frames
        )
        if self.state is not None:
            depth = self.state.calibration.depth
            for _ in range(self.warmup_frames):
                self.state.build_z_map(np.zeros((depth.height, depth.width), np.uint16))
        self.warmup_time_s = perf_counter() - start
        logger.info(
            f"Warm-up: {self.warmup_frames} frames in {self.warmup_time_s:.2f} s"
        )

    def process(
        self, timestamp: float, color: np.ndarray, depth: np.ndarray | None
    ) -> FrameResult:
        """Track and (with depth assessment on) assess one BGR frame. `depth` is
        None when no depth message was paired to this frame.
        """
        with self.timer.measure("track"):
            detections = self.detector.track(color)

        assessments = None
        z_map = None
        path = None
        state = self.state
        if state is not None:
            with self.timer.measure("z_map"):
                if depth is None:
                    z_map = np.zeros(color.shape[:2], dtype=np.float32)
                else:
                    z_map = state.build_z_map(depth)
            with self.timer.measure("assess"):
                assessments = assess_frame(
                    detections,
                    z_map,
                    state.calibration,
                    state.corridor,
                    state.config,
                    state.debouncer,
                )
                path = summarize_path(detections, assessments, state.buckets)

        with self.timer.measure("record"):
            record = build_record(
                self._frame_index,
                timestamp,
                detections,
                assessments,
                path,
                None if state is None else state.buckets,
            )
        result = FrameResult(
            self._frame_index, timestamp, detections, assessments, z_map, path, record
        )
        self._frame_index += 1
        return result
