import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml
from dotenv import find_dotenv, load_dotenv
from loguru import logger

ROOT = Path(__file__).resolve().parents[2]    # Used to be 3
# Repo-relative default for plain-Python scripts; the ROS node passes the installed
# share-directory path instead.
# DEFAULT_CONFIG_PATH = ROOT / "src" / "fortis_tk24_tts" / "config" / "tts.yaml"
DEFAULT_CONFIG_PATH = ROOT / "fortis_tk24_tts" / "config" / "tts.yaml"

# Deployment overrides of tts.yaml values (unset or empty = keep the YAML value).
ENV_PREFIX = "TK24_TTS_"
ENV_KEYS = ("language", "output", "topic_format", "audio_device")
ENV_FILE = ENV_PREFIX + "ENV_FILE"

OUTPUTS = ("speaker", "topic")
TOPIC_FORMATS = ("int16", "float32")


@dataclass
class VoiceConfig:
    model: Path
    length_scale: float = 1.0


@dataclass
class TtsConfig:
    language: str
    voice: VoiceConfig
    output: str = "speaker"
    topic_format: str = "float32"
    audio_device: str | int | None = None


def _resolve(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else ROOT / path


def _voice_config(raw: dict, language: str) -> VoiceConfig:
    voices = raw.get("voices") or {}
    if language not in voices:
        raise ValueError(f"No voice configured for language '{language}'")
    voice_raw = voices[language]
    return VoiceConfig(
        model=_resolve(voice_raw["model"]),
        length_scale=float(voice_raw.get("length_scale", 1.0)),
    )


def _audio_device(value: str) -> str | int | None:
    if value.lower() in ("default", "none", "null"):
        return None
    return int(value) if value.isdigit() else value


def load_env_file(path: str | Path | None = None) -> Path | None:
    """Load a .env file into os.environ (python-dotenv) and return its path.

    The file is `path`, else TK24_TTS_ENV_FILE, else the nearest `.env` from the
    current directory upwards. Variables already set in the environment (e.g. by
    docker compose `env_file`) take precedence over the file. Returns None if no
    file was found.
    """
    if path is None:
        path = os.environ.get(ENV_FILE) or find_dotenv(usecwd=True)
        if not path:
            return None
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Environment file {path} does not exist")
    load_dotenv(path, override=False)
    logger.info(f"Loaded environment file {path}")
    return path


def apply_env(raw: dict, env: Mapping[str, str] = os.environ) -> dict:
    """Override tts.yaml values with TK24_TTS_<KEY> environment variables.

    TK24_TTS_VOICE_MODEL and TK24_TTS_LENGTH_SCALE apply to the voice of the
    selected language. Unset or empty variables keep the YAML value.
    """
    raw = dict(raw)
    for key in ENV_KEYS:
        if value := env.get(ENV_PREFIX + key.upper()):
            raw[key] = _audio_device(value) if key == "audio_device" else value

    model = env.get(ENV_PREFIX + "VOICE_MODEL")
    length_scale = env.get(ENV_PREFIX + "LENGTH_SCALE")
    if model or length_scale:
        voices = dict(raw.get("voices") or {})
        voice = dict(voices.get(raw.get("language")) or {})
        if model:
            voice["model"] = model
        if length_scale:
            voice["length_scale"] = length_scale
        voices[raw.get("language")] = voice
        raw["voices"] = voices
    return raw


def load_voice_config(
    language: str, path: str | Path = DEFAULT_CONFIG_PATH
) -> VoiceConfig:
    return _voice_config(
        yaml.safe_load(Path(path).read_text(encoding="utf-8")), language
    )


def load_config(
    path: str | Path = DEFAULT_CONFIG_PATH, env: Mapping[str, str] | None = None
) -> TtsConfig:
    """Load tts.yaml, with TK24_TTS_* environment overrides (see apply_env).

    Without an explicit `env`, a .env file is loaded first (see load_env_file) and
    os.environ is used.
    """
    if env is None:
        load_env_file()
        env = os.environ
    raw = apply_env(yaml.safe_load(Path(path).read_text(encoding="utf-8")), env)

    language = raw.get("language")
    if not language:
        raise ValueError("Config must specify 'language'")

    output = raw.get("output", "speaker")
    if output not in OUTPUTS:
        raise ValueError(f"'output' must be one of {OUTPUTS}, got '{output}'")
    topic_format = raw.get("topic_format", "float32")
    if topic_format not in TOPIC_FORMATS:
        raise ValueError(
            f"'topic_format' must be one of {TOPIC_FORMATS}, got '{topic_format}'"
        )

    return TtsConfig(
        language=language,
        voice=_voice_config(raw, language),
        output=output,
        topic_format=topic_format,
        audio_device=raw.get("audio_device"),
    )
