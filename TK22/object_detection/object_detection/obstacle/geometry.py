"""Depth geometry for the obstacle pipeline: Z map, corridor test and floor overlay.

Camera frame is the optical frame: X right, Y down, Z forward, in meters.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import yaml

from object_detection.obstacle.calibration import CameraCalibration, Intrinsics

CORRIDOR_KEYS = (
    "robot_width",
    "side_margin",
    "x_cam",
    "min_range",
    "max_range",
)


def load_block(path: str | Path, name: str, keys: tuple[str, ...]) -> dict:
    """Read one block of the pipeline config; every key in `keys` is required."""
    with open(path) as f:
        config = yaml.safe_load(f)
    block = config.get(name)
    if not isinstance(block, dict):
        raise ValueError(f"Config '{path}' must have a '{name}' block.")
    missing = [key for key in keys if key not in block]
    if missing:
        raise ValueError(f"Config '{path}': '{name}' is missing keys {missing}.")
    return block


@dataclass(frozen=True)
class Corridor:
    """Driving corridor in the camera frame, all values in meters."""

    robot_width: float
    side_margin: float
    x_cam: float  # lateral center of the corridor in camera X
    min_range: float
    max_range: float

    @property
    def width(self) -> float:
        return self.robot_width + 2 * self.side_margin


def load_corridor(path: str | Path) -> Corridor:
    """Read and validate the `corridor` block of the pipeline config."""
    block = load_block(path, "corridor", CORRIDOR_KEYS)
    corridor = Corridor(**{key: float(block[key]) for key in CORRIDOR_KEYS})
    if corridor.robot_width <= 0 or corridor.side_margin < 0:
        raise ValueError("corridor.robot_width must be > 0 and side_margin >= 0.")
    if not 0 < corridor.min_range < corridor.max_range:
        raise ValueError("corridor needs 0 < min_range < max_range.")
    return corridor


class ZMapBuilder:
    """Builds color-resolution Z maps (meters, 0 = no depth) from uint16 depth images.

    For unaligned depth, a depth pixel with depth z lands at p_color = z * (R @ ray)
    + t, where the per-pixel `R @ ray` is computed once here. The work runs on
    `device` (torch); the returned map is a numpy array.
    """

    def __init__(
        self, calibration: CameraCalibration, device: torch.device | str | int
    ) -> None:
        self._calibration = calibration
        self._device = torch.device(device)
        if calibration.depth_aligned:
            return
        d = calibration.depth
        v, u = np.mgrid[0 : d.height, 0 : d.width]
        rays = np.stack(
            [(u - d.cx) / d.fx, (v - d.cy) / d.fy, np.ones(u.shape)]
        ).reshape(3, -1)
        extrinsics = calibration.depth_to_color
        direction = extrinsics.rotation @ rays
        self._direction = torch.tensor(
            direction, dtype=torch.float32, device=self._device
        )
        self._translation = torch.tensor(
            extrinsics.translation[:, None], dtype=torch.float32, device=self._device
        )

    def __call__(self, depth: np.ndarray) -> np.ndarray:
        calibration = self._calibration
        color = calibration.color
        scale = calibration.depth_scale
        if calibration.depth_aligned:
            if depth.shape != (color.height, color.width):
                raise ValueError(f"Aligned depth {depth.shape} != color size.")
            return depth.astype(np.float32) * np.float32(scale)
        if depth.shape != (calibration.depth.height, calibration.depth.width):
            raise ValueError(f"Depth {depth.shape} does not match the calibration.")

        # Dense and sync-free: pixels without a valid target go to a sink slot.
        raw = torch.from_numpy(depth.astype(np.float32)).to(self._device).reshape(-1)
        points = raw * scale * self._direction + self._translation
        zc = points[2]
        u = torch.round(points[0] * color.fx / zc + color.cx)
        v = torch.round(points[1] * color.fy / zc + color.cy)
        inside = (
            (raw > 0)
            & (zc > 0)
            & (u >= 0)
            & (u < color.width)
            & (v >= 0)
            & (v < color.height)
        )
        size = color.height * color.width
        index = torch.where(inside, v * color.width + u, size).long()

        # Nearest Z per pixel.
        flat = torch.full((size + 1,), torch.inf, device=self._device)
        flat.scatter_reduce_(
            0, index, torch.where(inside, zc, torch.inf), reduce="amin"
        )
        z_map = flat[:size].reshape(color.height, color.width)
        return torch.where(torch.isinf(z_map), 0.0, z_map).cpu().numpy()


def mask_points(
    z_map: np.ndarray, mask: np.ndarray, box: np.ndarray, color: Intrinsics
) -> tuple[np.ndarray, np.ndarray]:
    """Lateral X and depth Z of the mask pixels that have depth.

    Only the window of the xyxy `box` is scanned; the mask lies inside its box.
    """
    height, width = mask.shape
    x1 = max(int(np.floor(box[0])), 0)
    y1 = max(int(np.floor(box[1])), 0)
    x2 = min(int(np.ceil(box[2])), width)
    y2 = min(int(np.ceil(box[3])), height)
    z_win = z_map[y1:y2, x1:x2]
    v, u = np.nonzero(mask[y1:y2, x1:x2] & (z_win > 0))
    z = z_win[v, u]
    return (u + x1 - color.cx) * z / color.fx, z


def in_corridor(x: np.ndarray, z: np.ndarray, corridor: Corridor) -> np.ndarray:
    """Boolean per point: laterally inside the corridor and within range."""
    return (
        (np.abs(x - corridor.x_cam) <= corridor.width / 2)
        & (z >= corridor.min_range)
        & (z <= corridor.max_range)
    )


def corridor_pixel_mask(
    z_map: np.ndarray, corridor: Corridor, color: Intrinsics
) -> np.ndarray:
    """Boolean map of the pixels with depth that pass the corridor test."""
    u = np.arange(z_map.shape[1], dtype=np.float32)[None, :]
    x = (u - color.cx) * z_map / color.fx
    return (z_map > 0) & in_corridor(x, z_map, corridor)
