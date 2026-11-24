import json
from dataclasses import dataclass


@dataclass
class RobotMessage:
    content: str
    tone: str | None = None  # Parsed but unused: Piper has no tone control
    urgent: bool = False


def parse(json_str: str) -> RobotMessage:
    """Parse the JSON payload of /fortis/tk23/robot_to_human_message.

    Only place that knows the payload format.
    """
    data = json.loads(json_str)
    if not isinstance(data, dict) or not isinstance(data.get("content"), str):
        raise ValueError("Payload must be a JSON object with a string 'content'")
    return RobotMessage(
        content=data["content"],
        tone=data.get("tone"),
        urgent=bool(data.get("urgent", False)),
    )
