"""Conversions between images and `sensor_msgs` messages, without cv_bridge."""

import array

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

COLOR_ENCODINGS = ("rgb8", "bgr8")
DEPTH_ENCODING = "16UC1"


def image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a `sensor_msgs/Image` (rgb8 or bgr8) to a BGR array."""
    if msg.encoding not in COLOR_ENCODINGS:
        raise ValueError(f"Unsupported color encoding '{msg.encoding}'.")
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    image = rows[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def image_to_depth(msg: Image) -> np.ndarray:
    """Convert a `sensor_msgs/Image` (16UC1) to a uint16 array."""
    if msg.encoding != DEPTH_ENCODING or msg.is_bigendian:
        raise ValueError(
            f"Unsupported depth format '{msg.encoding}' (bigendian={msg.is_bigendian})."
        )
    rows = np.frombuffer(msg.data, np.uint8).view(np.uint16)
    return rows.reshape(msg.height, msg.step // 2)[:, : msg.width].copy()


def to_compressed(
    image: np.ndarray, header: Header, image_format: str, params: list[int]
) -> CompressedImage:
    """Encode `image` as `image_format` ('jpeg' or 'png') into a `CompressedImage`."""
    ok, buffer = cv2.imencode(f".{image_format}", image, params)
    if not ok:
        raise ValueError(f"Cannot encode image as {image_format}.")
    msg = CompressedImage()
    msg.header = header
    msg.format = image_format
    # array.array skips the per-byte validation the message setter runs on bytes.
    msg.data = array.array("B", buffer.tobytes())
    return msg
