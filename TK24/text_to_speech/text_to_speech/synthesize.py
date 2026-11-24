import json
import re
import string
import wave
from pathlib import Path
from time import perf_counter

from loguru import logger
from piper import PiperVoice

from text_to_speech.config import load_voice_config

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"
OUTPUT_DIR = ROOT / "outputs"

_PUNCT_TABLE = str.maketrans("", "", string.punctuation)


def to_filename(text: str) -> str:
    cleaned = text.lower().translate(_PUNCT_TABLE)
    return re.sub(r"\s+", " ", cleaned).strip()


def synthesize_language(samples_file: Path) -> None:
    language = samples_file.stem.removeprefix("samples_")
    voice = PiperVoice.load(load_voice_config(language).model)
    out_dir = OUTPUT_DIR / language
    out_dir.mkdir(parents=True, exist_ok=True)

    items = json.loads(samples_file.read_text(encoding="utf-8"))["items"]
    timings = {}
    for item in items:
        out_path = out_dir / f"{to_filename(item['english'])}.wav"
        start = perf_counter()
        with wave.open(str(out_path), "wb") as wav_file:
            voice.synthesize_wav(item["source"], wav_file)
        timings[out_path.name] = perf_counter() - start

    report = {
        "average_seconds": sum(timings.values()) / len(timings) if timings else 0.0,
        "files": timings,
    }
    (out_dir / "timings.json").write_text(
        json.dumps(report, indent=4), encoding="utf-8"
    )
    logger.info(f"[{language}] wrote {len(items)} files to {out_dir}")


def main() -> None:
    for samples_file in sorted(DATA_DIR.glob("samples_*.json")):
        synthesize_language(samples_file)


if __name__ == "__main__":
    main()
