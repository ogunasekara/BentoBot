"""OccupancyGrid <-> image conversion, PGM/YAML I/O (no nav2_map_server needed).

Images are uint8, top row first (as in a PGM), with map_saver's trinary
values: 0 = occupied, 254 = free, 205 = unknown. OccupancyGrid rows start at
the bottom (row 0 is at origin y), so conversion flips vertically.
"""
import io
import math
import os

import numpy as np
import yaml
from PIL import Image

OCCUPIED = 0
FREE = 254
UNKNOWN = 205

# Thresholds for turning OccupancyGrid probabilities (0-100) into pixels (as
# map_saver does)
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25
# Thresholds written to map.yaml for map_server to read the pixels back. 205
# (unknown) is occupancy 50/255 = 0.19608, so free_thresh must be <= that or
# map_server loads unknown space as free; ROS 1 map_saver writes 0.196 too.
YAML_OCCUPIED_THRESH = 0.65
YAML_FREE_THRESH = 0.196


def grid_to_image(width, height, data):
    """OccupancyGrid data (int8 -1..100, row-major from the bottom) -> image."""
    grid = np.asarray(data, dtype=np.int8).reshape(height, width)
    img = np.full((height, width), UNKNOWN, dtype=np.uint8)
    img[(grid >= 0) & (grid <= FREE_THRESH * 100)] = FREE
    img[grid >= OCCUPIED_THRESH * 100] = OCCUPIED
    return np.ascontiguousarray(np.flipud(img))


def normalize_image(img):
    """Snap arbitrary gray values (e.g. from an edited map) to trinary."""
    img = np.asarray(img, dtype=np.uint8)
    p = (255 - img.astype(np.float32)) / 255.0  # occupancy probability, negate: 0
    out = np.full(img.shape, UNKNOWN, dtype=np.uint8)
    out[p < YAML_FREE_THRESH] = FREE
    out[p > YAML_OCCUPIED_THRESH] = OCCUPIED
    return out


def png_bytes(img, compress_level=6):
    buf = io.BytesIO()
    Image.fromarray(img, mode='L').save(buf, format='PNG', compress_level=compress_level)
    return buf.getvalue()


def write_pgm(path, img):
    h, w = img.shape
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(b'P5\n# CREATOR: bentobot_web\n%d %d\n255\n' % (w, h))
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())
    os.replace(tmp, path)


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # header: magic, width, height, maxval, separated by whitespace / comments
    tokens = []
    i = 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        start = i
        while not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    i += 1  # single whitespace before the raster
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        # fall back to PIL for other variants
        return np.array(Image.open(path).convert('L'))
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i).reshape(h, w).copy()


def write_yaml(path, image_file, resolution, origin):
    doc = {
        'image': image_file,
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
        'negate': 0,
        'occupied_thresh': YAML_OCCUPIED_THRESH,
        'free_thresh': YAML_FREE_THRESH,
    }
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(doc, f, default_flow_style=None, sort_keys=False)
    os.replace(tmp, path)


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def thumbnail(img, max_size=320):
    """PNG thumbnail cropped to the explored area."""
    known = np.argwhere(img != UNKNOWN)
    if len(known):
        (y0, x0), (y1, x1) = known.min(0), known.max(0) + 1
        pad = 4
        y0, x0 = max(0, y0 - pad), max(0, x0 - pad)
        y1, x1 = min(img.shape[0], y1 + pad), min(img.shape[1], x1 + pad)
        img = img[y0:y1, x0:x1]
    pil = Image.fromarray(img, mode='L')
    pil.thumbnail((max_size, max_size), Image.NEAREST if max(pil.size) < max_size else Image.BOX)
    buf = io.BytesIO()
    pil.save(buf, format='PNG')
    return buf.getvalue()
