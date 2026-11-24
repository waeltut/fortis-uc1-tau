"""FORTIS interface registry lookup.

The registry (`interfaces.yaml` in `fortis_interfaces`) lists every topic of a
deployment with its message type and QoS:

    schema_version: 1
    topics:
      /fortis/tk23/robot_to_human_message:
        type: std_msgs/msg/String.msg
        qos: {reliability: reliable, durability: volatile, depth: 10}

It is read from `TK24_TTS_INTERFACES_FILE` or, if unset, from the installed
`fortis_interfaces` share directory. When a registry is found, the node's topics
must be listed in it with a matching type, and their QoS is taken from it. Without
a registry, any topic name is accepted and the default QoS (reliable, volatile,
depth 10) is used.
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

SUPPORTED_SCHEMA_VERSION = 1
DEFAULT_QOS_DEPTH = 10
ENV_INTERFACES_FILE = "TK24_TTS_INTERFACES_FILE"

_RELIABILITY = {
    "reliable": ReliabilityPolicy.RELIABLE,
    "best_effort": ReliabilityPolicy.BEST_EFFORT,
}
_DURABILITY = {
    "volatile": DurabilityPolicy.VOLATILE,
    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
}


def _registry_path() -> Path | None:
    if env_path := os.environ.get(ENV_INTERFACES_FILE):
        path = Path(env_path)
        if not path.is_file():
            raise FileNotFoundError(f"{ENV_INTERFACES_FILE}={path} does not exist")
        return path
    try:
        path = Path(get_package_share_directory("fortis_interfaces"))
    except PackageNotFoundError:
        return None
    path = path / "interfaces.yaml"
    return path if path.is_file() else None


def load_registry() -> dict[str, dict] | None:
    """Load the registered topics, or return None if no registry is available."""
    path = _registry_path()
    if path is None:
        logger.warning(
            "No FORTIS interface registry found; topic names are not validated and "
            "the default QoS is used."
        )
        return None
    registry = yaml.safe_load(path.read_text(encoding="utf-8"))
    version = registry.get("schema_version")
    if version != SUPPORTED_SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported interface registry schema_version {version} in {path}, "
            f"expected {SUPPORTED_SCHEMA_VERSION}"
        )
    logger.info(f"Loaded FORTIS interface registry from {path}")
    return registry.get("topics") or {}


def resolve_qos(
    registry: dict[str, dict] | None, topic: str, msg_type: str
) -> QoSProfile:
    """Validate `topic` (of type `<pkg>/msg/<Name>`) against the registry and
    return its QoS profile."""
    if registry is None:
        return QoSProfile(depth=DEFAULT_QOS_DEPTH)
    if topic not in registry:
        raise ValueError(
            f"Topic '{topic}' is not in the FORTIS interface registry. Available "
            f"topics: {', '.join(sorted(registry))}"
        )
    registered_type = registry[topic]["type"].removesuffix(".msg")
    if registered_type != msg_type:
        raise ValueError(
            f"Topic '{topic}' has type '{registered_type}' in the FORTIS interface "
            f"registry, but this node requires '{msg_type}'"
        )
    qos = registry[topic].get("qos") or {}
    return QoSProfile(
        depth=qos.get("depth", DEFAULT_QOS_DEPTH),
        reliability=_RELIABILITY[qos.get("reliability", "reliable")],
        durability=_DURABILITY[qos.get("durability", "volatile")],
    )
