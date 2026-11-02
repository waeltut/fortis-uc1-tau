"""Robot identity comes from offline MoveIt context, never guessed from link names."""
import re


def arm_names(metadata):
    names = metadata.get('context', {}).get('arm_names', ['left', 'right'])
    if (not isinstance(names, list) or not names or len(set(names)) != len(names)
            or any(not isinstance(n, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', n)
                   or n in ('either', 'both', 'intersection') for n in names)):
        raise ValueError('Invalid cache arm names')
    return names


def monitored_joints(metadata):
    context = metadata['context']
    result = []
    for name in arm_names(metadata):
        joints = context['arm_joints'][name]
        if not joints or any(not isinstance(j, str) or not j for j in joints):
            raise ValueError('Missing cached joints for ' + name)
        result.extend(joints)
    return list(dict.fromkeys(result))
