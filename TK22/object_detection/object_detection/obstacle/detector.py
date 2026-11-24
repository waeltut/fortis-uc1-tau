"""YOLOE model loading and tracking for BEKO obstacle detection.

Holds the prompt/embedding helpers shared with `video_infer.py` and the
`Detector`, which wraps `model.track` and returns a small per-frame structure.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from loguru import logger
from ultralytics import YOLOE
from ultralytics.engine.model import Model
from ultralytics.engine.results import Results
from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor

DEVICE = 0 if torch.cuda.is_available() else "cpu"
MODELS_DIR = Path("/srv/models/yolo")
MODEL_NAME = "yoloe-26x-seg"
DEFAULT_CONF = 0.5
DEFAULT_IMGSZ = 640


def _resolve(path: str, base: Path) -> str:
    """Resolve a relative config path against the config's directory."""
    return str(base / path) if not Path(path).is_absolute() else path


def load_config(config_path: str) -> dict:
    """Load and validate the YAML config: conf/imgsz (with defaults), objects and
    the optional tracker YAML path.

    Relative tracker YAML and visual-prompt image paths are resolved against the
    config's directory, so the config works from any working directory.
    """
    with open(config_path) as f:
        config = yaml.safe_load(f)

    objects = config.get("objects")
    if not objects:
        raise ValueError(
            f"Config '{config_path}' must have a non-empty 'objects' list."
        )
    base = Path(config_path).parent
    for obj in objects:
        for prompt in obj.get("prompts", []):
            prompt["image"] = _resolve(prompt["image"], base)

    # A bare name (e.g. 'tracktrack') is an ultralytics default, not a file.
    tracker = config.get("tracker")
    if tracker and Path(tracker).suffix in (".yaml", ".yml"):
        tracker = _resolve(tracker, base)

    return {
        "conf": config.get("conf", DEFAULT_CONF),
        "imgsz": config.get("imgsz", DEFAULT_IMGSZ),
        "objects": objects,
        "tracker": tracker,
        "jpeg_quality": config.get("jpeg_quality"),
    }


def get_image_embeddings(
    model: YOLOE,
    image_path: str,
    bboxes: list[list[float]],
    class_names: list[str],
    imgsz: int,
) -> torch.Tensor:
    """Run one reference image through YOLOE's visual-prompt predictor, one embedding
    per class.
    """
    class_to_idx = {name: idx for idx, name in enumerate(dict.fromkeys(class_names))}
    visual_prompts = {
        "bboxes": np.array(bboxes, dtype=np.float32),
        "cls": np.array([class_to_idx[name] for name in class_names], dtype=np.int64),
    }

    # A fresh predictor is required per reference image; it holds per-image state.
    predictor = YOLOEVPSegPredictor(
        overrides={
            "task": model.model.task,
            "mode": "predict",
            "save": False,
            "verbose": False,
            "batch": 1,
            "device": DEVICE,
            "imgsz": imgsz,
        },
        _callbacks=model.callbacks,
    )
    predictor.set_prompts(visual_prompts.copy())
    predictor.setup_model(model=model.model, verbose=False)
    return predictor.get_vpe(image_path)


def get_visual_embeddings(
    model: YOLOE,
    image_paths: list[str],
    bboxes: list[list[float]],
    class_names: list[str],
    imgsz: int,
) -> tuple[list[str], torch.Tensor]:
    """Compute per-class visual-prompt embeddings, averaging across images sharing a
    class name.

    This lets the same class (e.g. "box") be defined by boxes on multiple reference
    images, e.g. one where it's lying horizontally and one where it's vertical.
    """
    # Group prompts by image, since one predictor call handles all boxes on that image.
    entries_by_image: dict[str, list[tuple[list[float], str]]] = {}
    for image_path, bbox, class_name in zip(image_paths, bboxes, class_names):
        entries_by_image.setdefault(image_path, []).append((bbox, class_name))

    class_order: list[str] = []
    class_embeddings: dict[str, list[torch.Tensor]] = {}
    for image_path, entries in entries_by_image.items():
        image_bboxes = [bbox for bbox, _ in entries]
        image_classes = [class_name for _, class_name in entries]
        vpe = get_image_embeddings(
            model, image_path, image_bboxes, image_classes, imgsz
        )
        # vpe columns are ordered by each class's first occurrence on this image.
        for idx, class_name in enumerate(dict.fromkeys(image_classes)):
            class_embeddings.setdefault(class_name, []).append(vpe[:, idx : idx + 1])
            if class_name not in class_order:
                class_order.append(class_name)

    logger.info(
        "YOLOE visual prompt classes: "
        f"{ {name: len(class_embeddings[name]) for name in class_order} }"
    )
    combined = torch.cat(
        [
            # A mean of unit vectors is not unit-norm; the engine export fuses it as is.
            torch.nn.functional.normalize(
                torch.cat(class_embeddings[name], dim=0).mean(dim=0, keepdim=True),
                dim=-1,
            )
            for name in class_order
        ],
        dim=1,
    )
    return class_order, combined


def load_model(objects: list[dict], imgsz: int) -> YOLOE:
    logger.info(f"Using device: {DEVICE}")

    # Auto-downloads to this path if the weights are missing.
    model_path = MODELS_DIR / f"{MODEL_NAME}.pt"
    model = YOLOE(str(model_path))

    class_names: list[str] = []
    embeddings: list[torch.Tensor] = []

    text_names = [obj["name"] for obj in objects if obj["type"] == "text"]
    if text_names:
        class_names.extend(text_names)
        embeddings.append(model.get_text_pe(text_names))
    logger.info(f"YOLOE text prompt classes: {text_names}")

    visual_objects = [obj for obj in objects if obj["type"] == "visual"]
    if visual_objects:
        image_paths = [
            prompt["image"] for obj in visual_objects for prompt in obj["prompts"]
        ]
        bboxes = [prompt["bbox"] for obj in visual_objects for prompt in obj["prompts"]]
        visual_names = [obj["label"] for obj in visual_objects for _ in obj["prompts"]]
        visual_class_order, visual_pe = get_visual_embeddings(
            model, image_paths, bboxes, visual_names, imgsz
        )
        embeddings.append(visual_pe)
        class_names.extend(visual_class_order)

    if not embeddings:
        raise ValueError("No valid 'text' or 'visual' objects found in config.")

    # get_text_pe and get_vpe don't guarantee matching devices, so align before cat.
    embeddings = [embedding.to(embeddings[0].device) for embedding in embeddings]
    model.set_classes(class_names, torch.cat(embeddings, dim=1))
    return model


@dataclass
class FrameDetections:
    """Detections for one frame, as numpy arrays at full image resolution."""

    ids: np.ndarray | None  # (N,) track IDs, None if the tracker has no IDs yet
    classes: list[str]  # (N,) class names
    confidences: np.ndarray  # (N,)
    boxes: np.ndarray  # (N, 4) xyxy in pixels
    masks: np.ndarray | None  # (N, H, W) bool, None if there are no detections
    raw_classes: list[str]  # (M,) detections before tracking, M >= N
    raw_confidences: np.ndarray  # (M,)
    raw_boxes: np.ndarray  # (M, 4) xyxy in pixels
    result: Results  # tracked ultralytics result, e.g. for `plot()`


class Detector:
    """YOLOE with a tracking model. One instance handles one stream (no reset)."""

    def __init__(
        self,
        model: Model,
        tracker: str,
        conf: float,
        imgsz: int,
        jpeg_quality: int | None = None,
    ) -> None:
        self.model = model
        self.tracker = tracker
        self.conf = conf
        self.imgsz = imgsz
        self.jpeg_quality = jpeg_quality
        self._raw: Results | None = None
        # Summed ultralytics preprocess/inference/postprocess ms over `track` calls;
        # the tracker update runs after these are measured, so it is not included.
        self.speed_ms: dict[str, float] = {}
        # Must be registered before `track` adds the tracker callback, which
        # replaces the results with the tracked ones; callbacks run in order.
        model.add_callback("on_predict_postprocess_end", self._capture_raw)

    def _capture_raw(self, predictor: object) -> None:
        self._raw = predictor.results[0]  # type: ignore[attr-defined]

    def _prepare(self, frame: np.ndarray) -> np.ndarray:
        if self.jpeg_quality is None:
            return frame
        # Raw sensor noise lowers YOLOE confidences; a JPEG round trip smooths it.
        _, buffer = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        return cv2.imdecode(buffer, cv2.IMREAD_COLOR)

    def _inference_args(self) -> dict:
        # retina_masks gives full-resolution masks, needed to read depth per pixel.
        return {
            "conf": self.conf,
            "imgsz": self.imgsz,
            "device": DEVICE,
            "retina_masks": True,
            "verbose": False,
        }

    def warmup(self, frame: np.ndarray, num_frames: int) -> None:
        """Run `predict` (not `track`) on `frame`, so no tracker is created and the
        first real frame starts with fresh track IDs.
        """
        logger.info(f"Detector classes: {list(self.model.names.values())}")
        for _ in range(num_frames):
            self.model.predict(source=self._prepare(frame), **self._inference_args())

    def track(self, frame: np.ndarray) -> FrameDetections:
        results: list[Results] = self.model.track(
            source=self._prepare(frame),
            persist=True,
            tracker=self.tracker,
            **self._inference_args(),
        )
        assert len(results) == 1, "Expected batch size of 1."
        result = results[0]
        raw = self._raw
        assert raw is not None, "Raw detections were not captured."
        for stage, ms in result.speed.items():
            self.speed_ms[stage] = self.speed_ms.get(stage, 0.0) + ms

        boxes = result.boxes
        ids = None
        if boxes.id is not None:
            ids = boxes.id.int().cpu().numpy()
        masks = None
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy().astype(bool)

        return FrameDetections(
            ids=ids,
            classes=[result.names[int(c)] for c in boxes.cls.cpu().numpy()],
            confidences=boxes.conf.cpu().numpy(),
            boxes=boxes.xyxy.cpu().numpy(),
            masks=masks,
            raw_classes=[raw.names[int(c)] for c in raw.boxes.cls.cpu().numpy()],
            raw_confidences=raw.boxes.conf.cpu().numpy(),
            raw_boxes=raw.boxes.xyxy.cpu().numpy(),
            result=result,
        )
