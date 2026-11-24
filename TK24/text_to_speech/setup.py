from setuptools import setup

# Installed in editable mode via `pip install -e ./src/text_to_speech` (see
# requirements.raw). A setup.py is used instead of pyproject.toml because the
# container's setuptools is too old for PEP 660 editable installs.
setup(
    name="text_to_speech",
    version="0.0.0",
    description="ROS-free Piper TTS core used by the fortis_tk24_tts ROS 2 wrapper",
    packages=["text_to_speech"],
)
