"""Jetson health and joystick device info, read from /proc and /sys.

The container is privileged with the host's /dev, and /proc/stat,
/proc/meminfo and /sys/class/thermal show host-wide values.
"""
import glob
import os
import shutil

import psutil

JS_DEV = '/dev/input/js0'


def _read(path, default=None):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


def temperatures():
    temps = {}
    for zone in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        t = _read(os.path.join(zone, 'temp'))
        if t is None:
            continue
        try:
            c = int(t) / 1000.0
        except ValueError:
            continue
        # the Nano's PMIC zone reports a fixed 100 C placeholder; skip obvious junk
        if c <= -40 or c >= 150:
            continue
        name = _read(os.path.join(zone, 'type'), os.path.basename(zone))
        temps[name] = round(c, 1)
    return temps


class SystemStats:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        psutil.cpu_percent(percpu=True)  # prime the counters

    def snapshot(self):
        per_cpu = psutil.cpu_percent(percpu=True)
        mem = psutil.virtual_memory()
        try:
            disk = shutil.disk_usage(self.data_dir)
            disk = {'free': disk.free, 'total': disk.total, 'path': self.data_dir}
        except OSError:
            disk = None
        try:
            load = os.getloadavg()
        except OSError:
            load = None
        return {
            'cpu': round(sum(per_cpu) / max(len(per_cpu), 1), 1),
            'cpus': per_cpu,
            'load': load,
            'mem': {'used': mem.total - mem.available, 'total': mem.total, 'percent': mem.percent},
            'temps': temperatures(),
            'disk': disk,
        }


def joystick_info(dev=JS_DEV):
    """Existence, name and identity of the joystick device."""
    if not os.path.exists(dev):
        return {'present': False, 'dev': dev}
    sysdir = '/sys/class/input/%s/device' % os.path.basename(dev)
    name = _read(os.path.join(sysdir, 'name'), '')
    uniq = _read(os.path.join(sysdir, 'uniq'), '')
    vendor = _read(os.path.join(sysdir, 'id', 'vendor'), '')
    product = _read(os.path.join(sysdir, 'id', 'product'), '')
    # uniq is the Bluetooth MAC for BT gamepads; USB pads often leave it empty
    key = uniq.lower() if uniq else ('%s:%s:%s' % (vendor, product, name)).lower()
    return {
        'present': True,
        'dev': dev,
        'name': name,
        'uniq': uniq,
        'vendor': vendor,
        'product': product,
        'key': key,
    }
