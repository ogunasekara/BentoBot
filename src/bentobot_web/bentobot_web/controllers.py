"""Per-controller teleop axis profiles, stored in <data_dir>/controllers.json.

Profiles are keyed by the joystick's `uniq` (the Bluetooth MAC for BT pads)
or vendor:product:name, see system.joystick_info().
"""
import json
import os
import threading

PARAM_TYPES = {
    'axis_linear': int,
    'axis_angular': int,
    'scale_linear': float,
    'scale_angular': float,
    'invert_linear': bool,
    'invert_angular': bool,
    'deadzone': float,
}


def clean_params(params):
    out = {}
    for name, typ in PARAM_TYPES.items():
        if name not in params:
            continue
        v = params[name]
        if typ is bool:
            if not isinstance(v, bool):
                raise ValueError('%s must be true/false' % name)
        elif typ is int:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or int(v) != v or v < 0 or v > 63:
                raise ValueError('%s must be an axis index 0-63' % name)
            v = int(v)
        else:
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError('%s must be a number' % name)
            v = float(v)
        out[name] = v
    return out


class ControllerProfiles:
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()

    def all(self):
        try:
            with open(self.path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def get(self, key):
        return self.all().get(key)

    def save(self, key, name, params):
        with self.lock:
            data = self.all()
            data[key] = {'name': name, 'params': clean_params(params)}
            self._write(data)
            return data[key]

    def delete(self, key):
        with self.lock:
            data = self.all()
            data.pop(key, None)
            self._write(data)

    def _write(self, data):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp, self.path)
