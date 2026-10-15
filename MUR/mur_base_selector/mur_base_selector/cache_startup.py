"""Cache location and actionable startup diagnostics for the online selector."""
from pathlib import Path
import shlex
from zipfile import BadZipFile
from .volume import Volume, IncompleteVolumeError

DEFAULT_VOLUME_FILE = '~/ros2_ws/src/MUR/mur_base_selector/config/dexterous_volume.npz'


class CacheStartupError(RuntimeError):
    def __init__(self,message,exit_code=0):
        super().__init__(message)
        self.exit_code=exit_code


def load_selector_volume(path):
    path=Path(path).expanduser().absolute()
    command='ros2 run mur_base_selector build_volume --output '+shlex.quote(str(path))+' --workers 2'
    try:
        return Volume(path)
    except FileNotFoundError as error:
        raise CacheStartupError(
            f'Volume selector not started: cache missing at {path}. '
            f'Generate it with: {command} . If it already exists elsewhere, move it here '
            'or set volume_file in volume_selector.yaml. Restart the selector after generation. '
            'Exiting cleanly; the rest of the robot launch can continue.') from error
    except IncompleteVolumeError as error:
        raise CacheStartupError(
            f'Volume selector not started: cache generation is incomplete at {path}. '
            f'Resume with: {command} --resume . Keep the original generation settings and robot model. '
            'Restart the selector after generation. Exiting cleanly.') from error
    except (OSError,ValueError,KeyError,BadZipFile) as error:
        raise CacheStartupError(
            f'Cannot load volume cache at {path}: {error}. Check the file, permissions and volume_file setting. '
            'The file was not modified. Exiting without starting selection.',exit_code=1) from error
