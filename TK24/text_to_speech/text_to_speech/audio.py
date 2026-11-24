from collections.abc import Callable
from typing import Protocol

import numpy as np

WRITE_SLICE_S = 0.05


class AudioSink(Protocol):
    def write(
        self, chunk: bytes, should_stop: Callable[[], bool] | None = None
    ) -> None:
        """Play int16 PCM; sinks that block may poll should_stop to cut short."""
        ...

    def stop(self) -> None:
        """Abort audio that is currently playing/pending."""
        ...


def to_float32(chunk: bytes) -> np.ndarray:
    """Convert int16 PCM bytes to float32 samples in [-1, 1]."""
    return np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0


def resample(samples: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """Linearly resample int16 samples; adequate for speech."""
    if src_rate == dst_rate or samples.size == 0:
        return samples
    n_out = int(round(samples.size * dst_rate / src_rate))
    positions = np.arange(n_out) * (src_rate / dst_rate)
    out = np.interp(positions, np.arange(samples.size), samples.astype(np.float32))
    return np.clip(np.rint(out), -32768, 32767).astype(np.int16)


class SpeakerSink:
    """Plays int16 mono PCM on a stream that stays open between messages.

    The stream runs at the device's default rate, since many devices reject the
    voice's native rate; audio is resampled from sample_rate.
    """

    def __init__(self, sample_rate: int, device: str | int | None = None) -> None:
        import sounddevice as sd

        self._sample_rate = sample_rate
        self._output_rate = 0
        try:
            self._output_rate = int(
                sd.query_devices(device, "output")["default_samplerate"]
            )
            self._stream = sd.RawOutputStream(
                samplerate=self._output_rate,
                channels=1,
                dtype="int16",
                device=device,
            )
            self._stream.start()
        except (sd.PortAudioError, ValueError) as e:
            outputs = [
                f"  {i}: {d['name']}"
                for i, d in enumerate(sd.query_devices())
                if d["max_output_channels"] > 0
            ]
            raise RuntimeError(
                f"Cannot open audio device {device!r}: {e}. "
                "Set 'audio_device' in tts.yaml (name substring or index) to a "
                "connected output, or use output: topic. "
                "Output devices:\n" + "\n".join(outputs)
            ) from e

    def write(
        self, chunk: bytes, should_stop: Callable[[], bool] | None = None
    ) -> None:
        samples = np.frombuffer(chunk, dtype=np.int16)
        samples = resample(samples, self._sample_rate, self._output_rate)
        # stream.write() blocks while the buffer is full, so short slices let an
        # interrupt cut a long sentence instead of waiting for it to finish.
        step = int(self._output_rate * WRITE_SLICE_S)
        for i in range(0, samples.size, step):
            if should_stop is not None and should_stop():
                return
            self._stream.write(samples[i : i + step].tobytes())

    def stop(self) -> None:
        # abort() drops buffered audio; the stream is restarted so it stays open
        self._stream.abort()
        self._stream.start()

    def close(self) -> None:
        self._stream.close()


class TopicSink:
    """Publishes audio through a callback; the node supplies the ROS publisher.

    topic_format selects "int16" (PCM as-is) or "float32" ([-1, 1]).
    """

    def __init__(
        self, publish: Callable[[np.ndarray], None], topic_format: str
    ) -> None:
        self._publish = publish
        self._topic_format = topic_format

    def write(
        self, chunk: bytes, should_stop: Callable[[], bool] | None = None
    ) -> None:
        if self._topic_format == "float32":
            samples = to_float32(chunk)
        else:
            samples = np.frombuffer(chunk, dtype=np.int16)
        self._publish(samples)

    def stop(self) -> None:
        pass
