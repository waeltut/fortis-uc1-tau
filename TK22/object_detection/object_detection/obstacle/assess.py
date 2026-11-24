"""Per-object distance and in-path assessment, and per-track debouncing."""

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from object_detection.obstacle.calibration import Intrinsics
from object_detection.obstacle.detector import FrameDetections
from object_detection.obstacle.geometry import (
    Corridor,
    in_corridor,
    load_block,
    mask_points,
)

ASSESSMENT_KEYS = ("distance_percentile", "min_points", "min_in_path_frac")
DEBOUNCER_KEYS = ("k", "n")
BUCKETS = ("dynamic", "static")


@dataclass(frozen=True)
class AssessmentConfig:
    distance_percentile: float
    min_points: int
    min_in_path_frac: float


def load_assessment(path: str | Path) -> AssessmentConfig:
    """Read and validate the `assessment` block of the pipeline config."""
    block = load_block(path, "assessment", ASSESSMENT_KEYS)
    config = AssessmentConfig(
        float(block["distance_percentile"]),
        int(block["min_points"]),
        float(block["min_in_path_frac"]),
    )
    if not 0 <= config.distance_percentile <= 100:
        raise ValueError("assessment.distance_percentile must be in [0, 100].")
    if config.min_points < 1:
        raise ValueError("assessment.min_points must be >= 1.")
    if not 0 <= config.min_in_path_frac <= 1:
        raise ValueError("assessment.min_in_path_frac must be in [0, 1].")
    return config


@dataclass
class ObjectAssessment:
    distance_m: float | None  # None when the mask has no depth points
    lateral_m: float | None  # median X, None when the mask has no depth points
    n_points: int
    in_corridor_frac: float
    in_path_raw: bool
    distance_valid: bool
    in_path: bool = False  # debounced, set after `TrackDebouncer.update`


def assess_object(
    z_map: np.ndarray,
    mask: np.ndarray,
    box: np.ndarray,
    color: Intrinsics,
    corridor: Corridor,
    config: AssessmentConfig,
) -> ObjectAssessment:
    """Distance, lateral offset and raw in-path flag from one object's mask."""
    x, z = mask_points(z_map, mask, box, color)
    n_points = len(z)
    if n_points == 0:
        return ObjectAssessment(None, None, 0, 0.0, False, False)

    distance = float(np.percentile(z, config.distance_percentile))
    in_corridor_frac = float(np.mean(in_corridor(x, z, corridor)))
    enough_points = n_points >= config.min_points
    return ObjectAssessment(
        distance_m=distance,
        lateral_m=float(np.median(x)),
        n_points=n_points,
        in_corridor_frac=in_corridor_frac,
        in_path_raw=enough_points and in_corridor_frac >= config.min_in_path_frac,
        distance_valid=enough_points and distance <= corridor.max_range,
    )


class TrackDebouncer:
    """Reports a track as in path when at least `k` of its last `n` frames were."""

    def __init__(self, k: int, n: int) -> None:
        if not 1 <= k <= n:
            raise ValueError("debouncer needs 1 <= k <= n.")
        self.k = k
        self.n = n
        self._history: dict[int, deque[bool]] = {}

    def update(self, in_path_raw: dict[int, bool]) -> dict[int, bool]:
        """Add one frame's raw flags by track ID; returns the debounced flags.

        Tracks missing from this frame count as False, and a history is dropped
        once its last `n` entries are all False.
        """
        for track_id, raw in in_path_raw.items():
            self._history.setdefault(track_id, deque(maxlen=self.n)).append(raw)
        for track_id, history in list(self._history.items()):
            if track_id not in in_path_raw:
                history.append(False)
            if len(history) == self.n and not any(history):
                del self._history[track_id]

        return {
            track_id: track_id in self._history
            and sum(self._history[track_id]) >= self.k
            for track_id in in_path_raw
        }


def load_debouncer(path: str | Path) -> TrackDebouncer:
    """Build the debouncer from the `debouncer` block of the pipeline config."""
    block = load_block(path, "debouncer", DEBOUNCER_KEYS)
    return TrackDebouncer(int(block["k"]), int(block["n"]))


@dataclass(frozen=True)
class ClosestObstacle:
    """The nearest in-path object."""

    track_id: int
    object_class: str
    bucket: str  # dynamic | static
    distance_m: float
    lateral_m: float


@dataclass(frozen=True)
class PathResult:
    path_blocked: bool
    closest: ClosestObstacle | None  # None when the path is clear


def summarize_path(
    detections: FrameDetections,
    assessments: list[ObjectAssessment],
    buckets: dict[str, str],
) -> PathResult:
    """Path state of one frame: the nearest object that is in path with a valid
    distance, or a clear path.
    """
    closest = None
    for i, (cls, a) in enumerate(zip(detections.classes, assessments)):
        if not (a.in_path and a.distance_valid):
            continue
        assert detections.ids is not None  # in_path needs a track ID
        assert a.distance_m is not None and a.lateral_m is not None
        if closest is None or a.distance_m < closest.distance_m:
            closest = ClosestObstacle(
                int(detections.ids[i]), cls, buckets[cls], a.distance_m, a.lateral_m
            )
    return PathResult(closest is not None, closest)


def load_buckets(path: str | Path, objects: list[dict]) -> dict[str, str]:
    """Read the `buckets` map (prompt name or label -> dynamic|static); every prompt
    in `objects` must be mapped.
    """
    with open(path) as f:
        buckets = yaml.safe_load(f).get("buckets")
    if not isinstance(buckets, dict):
        raise ValueError(f"Config '{path}' must have a 'buckets' map.")
    invalid = {name: b for name, b in buckets.items() if b not in BUCKETS}
    if invalid:
        raise ValueError(f"buckets values must be one of {BUCKETS}, got {invalid}.")
    names = [obj["name"] if obj["type"] == "text" else obj["label"] for obj in objects]
    unmapped = [name for name in names if name not in buckets]
    if unmapped:
        raise ValueError(f"buckets has no entry for prompts {unmapped}.")
    return {name: str(bucket) for name, bucket in buckets.items()}
