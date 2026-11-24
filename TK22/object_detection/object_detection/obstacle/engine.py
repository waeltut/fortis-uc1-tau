"""TensorRT engine export and loading for the obstacle detector.

The engine is never copied between machines: it is built here on first run, on the
machine that uses it, and cached in the configured directory (a mounted volume in
Docker). Its filename carries the TensorRT version and a hash of the model, `imgsz`,
precision and prompts, so changing any of them builds a new engine; tuning parameters
do not affect it. The version keeps engines from different TensorRT installs (e.g.
host and dev container) apart when they share a directory.
"""

import hashlib
import json
import shutil
from dataclasses import dataclass
from pathlib import Path

from loguru import logger
from ultralytics import YOLO

from object_detection.obstacle.detector import DEVICE, MODEL_NAME, load_model
from object_detection.obstacle.geometry import load_block

ENGINE_KEYS = ("dir", "half")


@dataclass(frozen=True)
class EngineConfig:
    dir: Path  # where engines are cached
    half: bool  # FP16 export


def load_engine_config(path: str | Path) -> EngineConfig:
    """Read and validate the `engine` block of the pipeline config."""
    block = load_block(path, "engine", ENGINE_KEYS)
    return EngineConfig(dir=Path(block["dir"]), half=bool(block["half"]))


def prompts_hash(objects: list[dict], imgsz: int, half: bool) -> str:
    """Hash of everything baked into the engine, including the bytes of the
    visual-prompt images, so editing an image invalidates the engine.
    """
    digest = hashlib.sha256()
    digest.update(json.dumps(objects, sort_keys=True).encode())
    digest.update(f"{MODEL_NAME}|{imgsz}|{half}".encode())
    for obj in objects:
        if obj["type"] != "visual":
            continue
        for prompt in obj["prompts"]:
            digest.update(Path(prompt["image"]).read_bytes())
    return digest.hexdigest()[:12]


def engine_path(objects: list[dict], imgsz: int, config: EngineConfig) -> Path:
    # Imported here: only engine runs need TensorRT.
    import tensorrt

    digest = prompts_hash(objects, imgsz, config.half)
    name = f"{MODEL_NAME}_{imgsz}_trt{tensorrt.__version__}_{digest}.engine"
    return config.dir / name


def export_engine(
    objects: list[dict], imgsz: int, config: EngineConfig, target: Path
) -> None:
    """Export the model with its prompts baked in (`set_classes`) to `target`."""
    logger.info(f"Exporting TensorRT engine to {target} (this takes a few minutes)")

    model = load_model(objects, imgsz)
    exported = model.export(
        format="engine",
        quantize=16 if config.half else 32,
        imgsz=imgsz,
        device=DEVICE,
        verbose=False,
    )

    target.parent.mkdir(parents=True, exist_ok=True)
    # Move under a temp name first so a half-written file is never taken as ready.
    temp = target.with_suffix(".tmp")
    shutil.move(exported, temp)
    temp.replace(target)
    logger.info(f"Saved TensorRT engine: {target}")


def load_engine(objects: list[dict], imgsz: int, config: EngineConfig) -> YOLO:
    """Load the cached engine, exporting it first if missing. When it exists the
    `.pt` weights and prompt embeddings are not touched; class names come from the
    engine metadata.
    """
    target = engine_path(objects, imgsz, config)
    if target.exists():
        logger.info(f"Using cached TensorRT engine: {target}")
    else:
        export_engine(objects, imgsz, config, target)
    return YOLO(str(target), task="segment")
