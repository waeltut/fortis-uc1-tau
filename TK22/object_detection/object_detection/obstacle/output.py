"""Output of the obstacle pipeline: the per-frame record and the annotated overlay."""

import cv2
import numpy as np

from object_detection.obstacle.assess import ObjectAssessment, PathResult
from object_detection.obstacle.calibration import Intrinsics
from object_detection.obstacle.detector import FrameDetections
from object_detection.obstacle.geometry import Corridor, corridor_pixel_mask

IN_PATH_COLOR = (0, 0, 255)
OUT_OF_PATH_COLOR = (0, 160, 0)
CORRIDOR_TINT_COLOR = (0, 255, 255)  # yellow, distinct from the outline colors
CORRIDOR_TINT_ALPHA = 0.35
RAW_DETECTION_COLOR = (255, 0, 255)  # magenta
LABEL_COLOR = (255, 255, 255)
BANNER_HEIGHT = 32
BANNER_TEXT_COLOR = (255, 255, 255)


def build_record(
    frame_index: int,
    timestamp: float,
    detections: FrameDetections,
    assessments: list[ObjectAssessment] | None,
    path: PathResult | None,
    buckets: dict[str, str] | None,
) -> dict:
    """One JSONL record. Depth fields, `bucket` and `path` are null without depth."""
    objects = []
    for i, cls in enumerate(detections.classes):
        track_id = None if detections.ids is None else int(detections.ids[i])
        obj = {
            "track_id": track_id,
            "class": cls,
            "bucket": None,
            "confidence": float(detections.confidences[i]),
            "bbox": [float(v) for v in detections.boxes[i]],
            "distance_m": None,
            "distance_valid": None,
            "lateral_m": None,
            "n_points": None,
            "in_corridor_frac": None,
            "in_path_raw": None,
            "in_path": None,
        }
        if assessments is not None:
            assert buckets is not None
            a = assessments[i]
            obj["bucket"] = buckets[cls]
            obj["distance_m"] = a.distance_m
            obj["distance_valid"] = a.distance_valid
            obj["lateral_m"] = a.lateral_m
            obj["n_points"] = a.n_points
            obj["in_corridor_frac"] = a.in_corridor_frac
            obj["in_path_raw"] = a.in_path_raw
            obj["in_path"] = a.in_path
        objects.append(obj)

    path_record = None
    if path is not None:
        closest = path.closest
        path_record = {
            "path_blocked": path.path_blocked,
            "closest": None
            if closest is None
            else {
                "track_id": closest.track_id,
                "class": closest.object_class,
                "bucket": closest.bucket,
                "distance_m": closest.distance_m,
                "lateral_m": closest.lateral_m,
            },
        }
    return {
        "timestamp": timestamp,
        "frame_index": frame_index,
        "path": path_record,
        "objects": objects,
    }


def draw_tag(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int],
    below: bool = False,
) -> None:
    """Draw white text on a filled `color` background, above (default) or below `y`."""
    width, height = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    height += 4
    x = max(0, min(x, image.shape[1] - width))
    top = y if below else max(y - height, 0)
    cv2.rectangle(image, (x, top), (x + width, top + height), color, -1)
    cv2.putText(
        image,
        text,
        (x, top + height - 3),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        LABEL_COLOR,
        1,
        cv2.LINE_AA,
    )


def draw_raw_detections(image: np.ndarray, detections: FrameDetections) -> None:
    """Draw the pre-tracking detections in place, labelled below each box."""
    for cls, conf, box in zip(
        detections.raw_classes, detections.raw_confidences, detections.raw_boxes
    ):
        x1, y1, x2, y2 = (int(v) for v in box)
        cv2.rectangle(image, (x1, y1), (x2, y2), RAW_DETECTION_COLOR, 2)
        draw_tag(
            image, f"raw {cls} {conf:.2f}", x1, y2, RAW_DETECTION_COLOR, below=True
        )


def draw_overlay(
    frame_index: int,
    color: np.ndarray,
    z_map: np.ndarray,
    detections: FrameDetections,
    assessments: list[ObjectAssessment],
    path: PathResult,
    corridor: Corridor,
    intrinsics: Intrinsics,
) -> np.ndarray:
    """Color with the pixels checked against the corridor tinted, mask outlines with
    distance and in-path state, and the path banner.
    """
    tint = np.full_like(color, CORRIDOR_TINT_COLOR)
    tinted = cv2.addWeighted(
        color, 1 - CORRIDOR_TINT_ALPHA, tint, CORRIDOR_TINT_ALPHA, 0
    )
    inside = corridor_pixel_mask(z_map, corridor, intrinsics)
    image = np.where(inside[..., None], tinted, color)

    for i, cls in enumerate(detections.classes):
        a = assessments[i]
        track_id = None if detections.ids is None else int(detections.ids[i])
        outline = IN_PATH_COLOR if a.in_path else OUT_OF_PATH_COLOR
        assert detections.masks is not None
        contours, _ = cv2.findContours(
            detections.masks[i].astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(image, contours, -1, outline, 2)

        distance = "n/a" if a.distance_m is None else f"{a.distance_m:.2f}m"
        lateral = "n/a" if a.lateral_m is None else f"{a.lateral_m:+.2f}m"
        label = f"{cls} #{track_id} d={distance} x={lateral}"
        if a.in_path:
            label += " IN PATH"
        x1, y1 = (int(v) for v in detections.boxes[i][:2])
        draw_tag(image, label, x1, y1, outline)

    draw_raw_detections(image, detections)

    cv2.putText(
        image,
        f"frame {frame_index}",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        LABEL_COLOR,
        2,
    )

    closest = path.closest
    if closest is None:
        banner, banner_color = "PATH CLEAR", (0, 128, 0)
    else:
        banner = f"PATH BLOCKED: {closest.object_class}, {closest.distance_m:.2f} m"
        banner_color = (0, 0, 200)
    top = image.shape[0] - BANNER_HEIGHT
    cv2.rectangle(image, (0, top), (image.shape[1], image.shape[0]), banner_color, -1)
    cv2.putText(
        image,
        banner,
        (8, image.shape[0] - 9),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        BANNER_TEXT_COLOR,
        2,
    )
    return image
