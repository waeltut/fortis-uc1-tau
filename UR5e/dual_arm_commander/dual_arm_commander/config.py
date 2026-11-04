"""ROS-independent, strict validation for named and explicit motion requests."""
import copy
import math
import yaml

SIDES = ('left', 'right')
SUFFIXES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def joints(side):
    return tuple(f'{side}_{suffix}' for suffix in SUFFIXES)


def choice(value, allowed, field):
    if not isinstance(value, str) or value.strip().lower() not in allowed:
        raise ValueError(f'{field} must be one of {", ".join(allowed)}')
    return value.strip().lower()


def number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{field} must be a finite number')
    return float(value)


def scaling(value, default, field):
    value = number(value, field)
    if value == 0:
        value = number(default, field)
    if not 0 < value <= 1:
        raise ValueError(f'{field} must be > 0 and <= 1 (0 selects default)')
    return value


def validate(data, defaults):
    if not isinstance(data, dict):
        raise ValueError('Motion must be a mapping')
    data = copy.deepcopy(data)
    allowed = {'mode', 'target_type', 'sequential_order', 'left_target', 'right_target',
               'left_joints', 'right_joints', 'velocity_scaling', 'acceleration_scaling'}
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f'Unknown motion fields: {sorted(unknown)}')
    data['mode'] = choice(data.get('mode'), ('left', 'right', 'both', 'sequential'), 'mode')
    data['target_type'] = choice(data.get('target_type'), ('pose', 'joints'), 'target_type')
    data['sequential_order'] = choice(data.get('sequential_order', 'left_first'),
                                     ('left_first', 'right_first'), 'sequential_order')
    for key in ('velocity_scaling', 'acceleration_scaling'):
        data[key] = scaling(data.get(key, 0), defaults[key], key)
    sides = SIDES if data['mode'] in ('both', 'sequential') else (data['mode'],)
    for side in sides:
        if data['target_type'] == 'joints':
            target = data.get(f'{side}_joints')
            if not isinstance(target, dict) or set(target) != set(joints(side)):
                raise ValueError(f'{side}_joints must name exactly: {", ".join(joints(side))}')
            data[f'{side}_joints'] = {k: number(v, k) for k, v in target.items()}
        else:
            target = data.get(f'{side}_target')
            if not isinstance(target, dict) or set(target) != {'frame_id', 'position', 'orientation'}:
                raise ValueError(f'{side}_target requires frame_id, position, orientation')
            if not isinstance(target['frame_id'], str) or not target['frame_id'].strip():
                raise ValueError(f'{side} frame_id is empty')
            for key, length in (('position', 3), ('orientation', 4)):
                values = target[key]
                if not isinstance(values, (tuple, list)) or len(values) != length:
                    raise ValueError(f'{side} {key} requires {length} numbers')
                target[key] = [number(v, key) for v in values]
            norm = math.sqrt(sum(x*x for x in target['orientation']))
            if abs(norm - 1) > 0.01:
                raise ValueError(f'{side} orientation must be a unit quaternion [x,y,z,w]')
            target['orientation'] = [x / norm for x in target['orientation']]
    return data


class UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'Duplicate YAML key: {key}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def load_presets(path, defaults):
    with open(path, encoding='utf-8') as stream:
        root = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(root, dict) or set(root) != {'presets'} or not isinstance(root['presets'], dict):
        raise ValueError('Preset file must contain one presets mapping')
    result = {}
    for name, data in root['presets'].items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError('Preset names must be nonempty strings')
        try:
            result[name] = validate(data, defaults)
        except ValueError as exc:
            raise ValueError(f'Preset {name!r}: {exc}') from exc
    return result
