"""Pairing of streamed color frames with depth by header stamp, latest frame wins.

Same rule as the offline `BagSource`: each color frame gets the nearest buffered
depth within `sync.max_sync_dt`, and is resolved once a later message (color or
depth) shows that no better depth can still arrive. Resolved frames wait in a
one-slot buffer: the consumer always takes the newest one and older ones are
dropped, since the camera publishes faster than the pipeline runs.

Payloads are opaque (e.g. undecoded ROS messages), so a dropped frame is never
decoded. `StreamPairer` is not thread-safe; callers serialize access.
"""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from loguru import logger

from object_detection.obstacle.geometry import load_block

NS_PER_S = 1_000_000_000
NS_PER_MS = 1e6
SYNC_KEYS = ("max_sync_dt", "depth_buffer")
# A stamp this far behind the latest one restarts pairing, e.g. a looped bag.
RESET_JUMP_NS = NS_PER_S
DT_HISTORY = 1000  # recent pairing gaps kept for the median


@dataclass(frozen=True)
class SyncConfig:
    max_sync_dt: float  # s
    depth_buffer: int  # recent depth messages kept for pairing


def load_sync_config(path: str | Path) -> SyncConfig:
    """Read and validate the `sync` block of the pipeline config."""
    block = load_block(path, "sync", SYNC_KEYS)
    config = SyncConfig(float(block["max_sync_dt"]), int(block["depth_buffer"]))
    if config.max_sync_dt <= 0 or config.depth_buffer < 1:
        raise ValueError("sync.max_sync_dt must be > 0 and sync.depth_buffer >= 1.")
    return config


@dataclass(frozen=True)
class PairedFrame:
    stamp_ns: int  # color header stamp
    color: Any
    depth: Any | None  # None when no depth was within max_sync_dt


class StreamPairer:
    """Pairs color with the nearest depth and keeps only the newest paired frame."""

    def __init__(self, config: SyncConfig) -> None:
        self._tolerance_ns = round(config.max_sync_dt * NS_PER_S)
        self._depths: deque[tuple[int, Any]] = deque(maxlen=config.depth_buffer)
        self._pending: deque[tuple[int, Any]] = deque()
        self._latest_ns: int | None = None
        self._ready: PairedFrame | None = None
        self._dt_ms: deque[float] = deque(maxlen=DT_HISTORY)
        self._dt_ms_max: float | None = None
        self.paired = 0
        self.unpaired = 0
        self.dropped = 0  # paired frames replaced before the consumer took them
        self.resets = 0

    def add_color(self, stamp_ns: int, payload: Any) -> None:
        self._add(stamp_ns, payload, self._pending)

    def add_depth(self, stamp_ns: int, payload: Any) -> None:
        self._add(stamp_ns, payload, self._depths)

    def pop(self) -> PairedFrame | None:
        """Take the newest paired frame, or None if there is none yet."""
        frame, self._ready = self._ready, None
        return frame

    def stats(self) -> dict:
        return {
            "frames": self.paired + self.unpaired,
            "paired": self.paired,
            "unpaired": self.unpaired,
            "dropped": self.dropped,
            "resets": self.resets,
            "dt_ms_median": float(np.median(self._dt_ms)) if self._dt_ms else None,
            "dt_ms_max": self._dt_ms_max,
        }

    def _add(self, stamp_ns: int, payload: Any, queue: deque) -> None:
        if self._latest_ns is not None and stamp_ns < self._latest_ns - RESET_JUMP_NS:
            logger.warning(
                f"Stamp jumped back by {(self._latest_ns - stamp_ns) / NS_PER_S:.1f} "
                "s; restarting color/depth pairing."
            )
            self._depths.clear()
            self._pending.clear()
            self._latest_ns = None
            self.resets += 1
        self._latest_ns = (
            stamp_ns if self._latest_ns is None else max(self._latest_ns, stamp_ns)
        )
        queue.append((stamp_ns, payload))
        while self._pending and self._pending[0][0] + self._tolerance_ns < (
            self._latest_ns
        ):
            self._resolve()

    def _resolve(self) -> None:
        stamp, color = self._pending.popleft()
        depth = None
        if self._depths:
            near_stamp, near = min(self._depths, key=lambda d: abs(d[0] - stamp))
            dt_ns = abs(near_stamp - stamp)
            if dt_ns <= self._tolerance_ns:
                depth = near
                dt_ms = dt_ns / NS_PER_MS
                self._dt_ms.append(dt_ms)
                self._dt_ms_max = max(self._dt_ms_max or 0.0, dt_ms)
        if depth is None:
            self.unpaired += 1
        else:
            self.paired += 1
        if self._ready is not None:
            self.dropped += 1
        self._ready = PairedFrame(stamp, color, depth)
