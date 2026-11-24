"""FORTIS interface registry lookup.

The FORTIS interface registry (`interfaces.yaml` in `fortis_interfaces`) lists every
topic available in a deployment, together with its message type and QoS (services and
actions will follow):

    schema_version: 1
    topics:
      /fortis/tk21/video_chunk/image_compressed:
        type: sensor_msgs/msg/CompressedImage.msg
        description: TK2.1 ingestion system - video chunk (RGB image)
        qos: {reliability: reliable, durability: volatile, depth: 10}

The registry is looked up in `TK22_OD_INTERFACES_FILE` or, if unset, as
`interfaces.yaml` in the installed `fortis_interfaces` share directory. When a registry
is found, the selected topics must be listed in it with a matching type, and their QoS
is taken from it. Topics resolved with `required=False` (the camera inputs, which are
the RealSense topics of the BEKO rosbags until the TK2.1 interface is agreed, and debug
outputs) may be missing from it and then get the default QoS. Without a registry, any
topic name is accepted and the default QoS is used.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from loguru import logger
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# RealSense topics of the BEKO rosbags (`ros2 bag play`), raw `sensor_msgs/Image`; not
# registered. To be replaced by the TK2.1 interface once it is agreed.
DEFAULT_COLOR_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEFAULT_OUTPUT_TOPIC = "/fortis/tk22/obstacle_detection"
# Debug only, never registered; `/compressed` so that rqt_image_view lists it.
DEFAULT_ANNOTATED_TOPIC = "/fortis/tk22/obstacle_detection/annotated/compressed"

SUPPORTED_SCHEMA_VERSION = 1
DEFAULT_QOS_DEPTH = 10

_RELIABILITY = {
    "reliable": ReliabilityPolicy.RELIABLE,
    "best_effort": ReliabilityPolicy.BEST_EFFORT,
}
_DURABILITY = {
    "volatile": DurabilityPolicy.VOLATILE,
    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
}


def _registry_path() -> Path | None:
    if path := os.environ.get("TK22_OD_INTERFACES_FILE"):
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f"TK22_OD_INTERFACES_FILE={path} does not exist")
        return path
    try:
        share_dir = Path(get_package_share_directory("fortis_interfaces"))
        path = share_dir / "interfaces.yaml"
    except PackageNotFoundError:
        return None
    return path if path.is_file() else None


def load_registry() -> dict[str, dict] | None:
    """Load the registered topics, or return None if no registry is available."""
    path = _registry_path()
    if path is None:
        logger.warning(
            "No FORTIS interface registry found; topic names are not validated and "
            "default QoS (depth {}) is used.",
            DEFAULT_QOS_DEPTH,
        )
        return None
    with path.open() as f:
        registry = yaml.safe_load(f)
    version = registry.get("schema_version")
    if version != SUPPORTED_SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported interface registry schema_version {version} in {path}, "
            f"expected {SUPPORTED_SCHEMA_VERSION}"
        )
    logger.info("Loaded FORTIS interface registry from {}", path)
    return registry.get("topics") or {}


def _type_name(msg_type: type) -> str:
    """Return the `<pkg>/msg/<Name>` name of a ROS message class."""
    return f"{msg_type.__module__.split('.')[0]}/msg/{msg_type.__name__}"


def resolve_qos(
    registry: dict[str, dict] | None,
    topic: str,
    msg_type: type,
    required: bool = True,
) -> QoSProfile:
    """Validate *topic* against the registry and return its QoS profile.

    With `required=False`, a topic missing from the registry is accepted with the
    default QoS; a registered one is still type-checked.
    """
    if registry is None:
        return QoSProfile(depth=DEFAULT_QOS_DEPTH)
    if topic not in registry:
        if not required:
            logger.warning(
                f"Topic '{topic}' is not in the FORTIS interface registry; using "
                f"the default QoS (depth {DEFAULT_QOS_DEPTH})."
            )
            return QoSProfile(depth=DEFAULT_QOS_DEPTH)
        raise ValueError(
            f"Topic '{topic}' is not in the FORTIS interface registry. Available "
            f"topics: {', '.join(sorted(registry))}"
        )
    entry = registry[topic]
    registered_type = entry["type"].removesuffix(".msg")
    expected_type = _type_name(msg_type)
    if registered_type != expected_type:
        raise ValueError(
            f"Topic '{topic}' has type '{registered_type}' in the FORTIS interface "
            f"registry, but this node requires '{expected_type}'"
        )
    qos = entry.get("qos", {})
    return QoSProfile(
        depth=qos.get("depth", DEFAULT_QOS_DEPTH),
        reliability=_RELIABILITY[qos.get("reliability", "reliable")],
        durability=_DURABILITY[qos.get("durability", "volatile")],
    )
