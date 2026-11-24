from collections.abc import Iterator
from typing import Protocol

from piper import PiperVoice, SynthesisConfig

from text_to_speech.config import VoiceConfig


class TtsBackend(Protocol):
    sample_rate: int

    def load(self, voice_cfg: VoiceConfig) -> None: ...

    def stream(self, text: str) -> Iterator[bytes]:
        """Yield int16 mono PCM chunks, one per sentence."""
        ...


class PiperBackend:
    def __init__(self) -> None:
        self._voice: PiperVoice | None = None
        self._syn_config: SynthesisConfig | None = None
        self.sample_rate = 0

    def load(self, voice_cfg: VoiceConfig) -> None:
        self._voice = PiperVoice.load(voice_cfg.model)
        self._syn_config = SynthesisConfig(length_scale=voice_cfg.length_scale)
        self.sample_rate = self._voice.config.sample_rate

    def stream(self, text: str) -> Iterator[bytes]:
        if self._voice is None:
            raise RuntimeError("Backend not loaded; call load() first")
        for chunk in self._voice.synthesize(text, self._syn_config):
            yield chunk.audio_int16_bytes
