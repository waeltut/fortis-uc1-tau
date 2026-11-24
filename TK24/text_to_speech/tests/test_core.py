import os
from pathlib import Path

import numpy as np
import pytest
import yaml
from text_to_speech.audio import resample, to_float32
from text_to_speech.config import load_config, load_env_file
from text_to_speech.message import parse

RAW_CONFIG = {
    "language": "en",
    "output": "speaker",
    "audio_device": "H650e",
    "voices": {
        "en": {"model": "/m/en.onnx", "length_scale": 1.0},
        "es": {"model": "/m/es.onnx"},
    },
}


@pytest.fixture
def config_file(tmp_path: Path) -> Path:
    path = tmp_path / "tts.yaml"
    path.write_text(yaml.safe_dump(RAW_CONFIG), encoding="utf-8")
    return path


def test_load_config_without_env(config_file: Path) -> None:
    cfg = load_config(config_file, env={})
    assert (cfg.language, cfg.output, cfg.audio_device) == ("en", "speaker", "H650e")
    assert cfg.voice.model == Path("/m/en.onnx")


def test_load_config_env_overrides(config_file: Path) -> None:
    env = {
        "TK24_TTS_LANGUAGE": "es",
        "TK24_TTS_OUTPUT": "topic",
        "TK24_TTS_TOPIC_FORMAT": "float32",
        "TK24_TTS_AUDIO_DEVICE": "3",
        "TK24_TTS_LENGTH_SCALE": "1.2",
    }
    cfg = load_config(config_file, env=env)
    assert (cfg.language, cfg.output, cfg.topic_format) == ("es", "topic", "float32")
    assert cfg.audio_device == 3
    assert cfg.voice.model == Path("/m/es.onnx")
    assert cfg.voice.length_scale == 1.2


def test_load_config_env_empty_and_default(config_file: Path) -> None:
    env = {"TK24_TTS_LANGUAGE": "", "TK24_TTS_AUDIO_DEVICE": "default"}
    cfg = load_config(config_file, env=env)
    assert cfg.language == "en"
    assert cfg.audio_device is None


def test_load_config_env_voice_model(config_file: Path) -> None:
    cfg = load_config(config_file, env={"TK24_TTS_VOICE_MODEL": "/other.onnx"})
    assert cfg.voice.model == Path("/other.onnx")
    assert cfg.voice.length_scale == 1.0


def test_load_config_env_invalid_output(config_file: Path) -> None:
    with pytest.raises(ValueError):
        load_config(config_file, env={"TK24_TTS_OUTPUT": "radio"})


def test_load_env_file(
    config_file: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    env_file = tmp_path / ".env"
    env_file.write_text(
        "TK24_TTS_LANGUAGE=es\nTK24_TTS_OUTPUT=topic\n", encoding="utf-8"
    )
    # load_dotenv writes to os.environ; use a copy so nothing leaks into other tests
    monkeypatch.setattr(os, "environ", dict(os.environ))
    monkeypatch.delenv("TK24_TTS_LANGUAGE", raising=False)
    monkeypatch.setenv("TK24_TTS_OUTPUT", "speaker")  # set env wins over the file
    monkeypatch.setenv("TK24_TTS_ENV_FILE", str(env_file))
    cfg = load_config(config_file)
    assert (cfg.language, cfg.output) == ("es", "speaker")


def test_load_env_file_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("TK24_TTS_ENV_FILE", "/nonexistent/.env")
    with pytest.raises(FileNotFoundError):
        load_env_file()


def test_parse_full_payload() -> None:
    msg = parse('{"content": "Hello", "tone": "calm", "urgent": true}')
    assert (msg.content, msg.tone, msg.urgent) == ("Hello", "calm", True)


def test_parse_defaults() -> None:
    msg = parse('{"content": "Hello"}')
    assert msg.tone is None
    assert msg.urgent is False


@pytest.mark.parametrize("payload", ["[]", '{"tone": "x"}', '{"content": 1}'])
def test_parse_rejects_invalid(payload: str) -> None:
    with pytest.raises(ValueError):
        parse(payload)


def test_to_float32_range() -> None:
    pcm = np.array([-32768, 0, 16384], dtype=np.int16).tobytes()
    assert to_float32(pcm).tolist() == [-1.0, 0.0, 0.5]


def test_resample_doubles_length_and_keeps_dtype() -> None:
    x = np.array([0, 1000, 2000, 3000], dtype=np.int16)
    y = resample(x, 22050, 44100)
    assert y.dtype == np.int16
    assert len(y) == 8
    assert y[:3].tolist() == [0, 500, 1000]
    assert resample(x, 22050, 22050) is x
