import copy
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual_arm_commander.config import validate, load_presets, joints

DEFAULTS = {'velocity_scaling': 0.1, 'acceleration_scaling': 0.1}


def pose(mode='left'):
    return {'mode': mode, 'target_type': 'pose', 'left_target': {
        'frame_id': 'mur', 'position': [0.6, 0.15, 1.2], 'orientation': [0, 0, 0, 1]}}


def test_case_and_defaults():
    result = validate(pose('LEFT'), DEFAULTS)
    assert result['mode'] == 'left'
    assert result['velocity_scaling'] == 0.1


@pytest.mark.parametrize('key,value', [('mode', 'simultaneous'), ('target_type', 'position'),
    ('velocity_scaling', -0.1), ('velocity_scaling', 1.1), ('acceleration_scaling', math.nan),
    ('sequential_order', 'random')])
def test_bad_values(key, value):
    data = pose(); data[key] = value
    with pytest.raises(ValueError): validate(data, DEFAULTS)


@pytest.mark.parametrize('mode', ['both', 'sequential'])
def test_both_requires_both_targets(mode):
    with pytest.raises(ValueError): validate(pose(mode), DEFAULTS)


@pytest.mark.parametrize('q', [[0, 0, 0, 0], [0, 0, 0, 2], [0, 0, math.inf, 1]])
def test_bad_quaternion(q):
    data = pose(); data['left_target']['orientation'] = q
    with pytest.raises(ValueError): validate(data, DEFAULTS)


def test_named_joints_exact():
    data = {'mode': 'left', 'target_type': 'joints', 'left_joints': dict.fromkeys(joints('left'), 0)}
    assert validate(data, DEFAULTS)['left_joints'] == data['left_joints']
    data['left_joints']['right_elbow_joint'] = 0
    with pytest.raises(ValueError): validate(data, DEFAULTS)


def test_preset_file():
    presets = load_presets(Path(__file__).resolve().parents[1]/'config/presets.yaml', DEFAULTS)
    assert set(presets) == {'home', 'home_joints', 'ready_joints'}
    assert presets['home']['left_target']['position'] == [0.6, 0.15, 1.2]


def test_duplicate_yaml_rejected(tmp_path):
    file = tmp_path/'bad.yaml'
    file.write_text('presets:\n  home: {}\n  home: {}\n')
    with pytest.raises(ValueError, match='Duplicate'): load_presets(file, DEFAULTS)


def test_unknown_field_rejected():
    data = pose(); data['velcity_scaling'] = 0.2
    with pytest.raises(ValueError): validate(data, DEFAULTS)


def test_input_not_mutated():
    data = pose(); original = copy.deepcopy(data)
    validate(data, DEFAULTS)
    assert data == original
