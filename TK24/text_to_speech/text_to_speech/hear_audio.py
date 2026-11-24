import sys

from loguru import logger

from text_to_speech.backend import PiperBackend
from text_to_speech.config import load_config

cfg = load_config()
b = PiperBackend()
b.load(cfg.voice)
logger.info("sample_rate {}", b.sample_rate)
for chunk in b.stream(sys.argv[1]):
    sys.stdout.buffer.write(chunk)
    sys.stdout.buffer.flush()
