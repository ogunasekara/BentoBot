"""Map library on disk.

  <data_dir>/maps/<id>/
      meta.json              name, created, duration, resolution, size, parent...
      map.pgm, map.yaml      occupancy image (what AMCL localizes on; editable)
      posegraph.posegraph    slam_toolbox serialized pose graph (if saved)
      posegraph.data
      thumb.png

The id never changes (rename only changes meta.json), so map_server paths
stay valid.
"""
import datetime
import io
import json
import os
import re
import shutil
import threading
import zipfile

from . import mapimg

ID_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,80}$')


class MapError(Exception):
    pass


def now_iso():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def slugify(name):
    s = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')
    return s[:40] or 'map'


class MapStore:
    def __init__(self, root):
        self.root = root
        self.lock = threading.Lock()
        os.makedirs(root, exist_ok=True)

    # ---- paths ----

    def path(self, map_id, *parts):
        if not ID_RE.match(map_id or ''):
            raise MapError('invalid map id')
        return os.path.join(self.root, map_id, *parts)

    def exists(self, map_id):
        try:
            return os.path.isfile(self.path(map_id, 'meta.json'))
        except MapError:
            return False

    def yaml_path(self, map_id):
        return self.path(map_id, 'map.yaml')

    def posegraph_prefix(self, map_id):
        return self.path(map_id, 'posegraph')

    # ---- metadata ----

    def _read_meta(self, map_id):
        with open(self.path(map_id, 'meta.json')) as f:
            meta = json.load(f)
        meta['id'] = map_id
        d = self.path(map_id)
        meta['size_bytes'] = sum(os.path.getsize(os.path.join(d, n)) for n in os.listdir(d))
        meta['has_posegraph'] = os.path.isfile(self.path(map_id, 'posegraph.posegraph'))
        return meta

    def _write_meta(self, map_id, meta):
        meta = {k: v for k, v in meta.items() if k not in ('id', 'size_bytes', 'has_posegraph')}
        tmp = self.path(map_id, 'meta.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(meta, f, indent=2)
        os.replace(tmp, self.path(map_id, 'meta.json'))

    def list(self):
        out = []
        for name in os.listdir(self.root):
            if self.exists(name):
                try:
                    out.append(self._read_meta(name))
                except (OSError, ValueError):
                    continue
        out.sort(key=lambda m: m.get('created', ''), reverse=True)
        return out

    def get(self, map_id):
        if not self.exists(map_id):
            raise MapError('no such map')
        return self._read_meta(map_id)

    # ---- create ----

    def new_id(self, name):
        base = '%s-%s' % (slugify(name), datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
        map_id, n = base, 2
        while os.path.exists(os.path.join(self.root, map_id)):
            map_id = '%s-%d' % (base, n)
            n += 1
        return map_id

    def reserve(self, name):
        """Create an empty map directory (the pose graph is written into it first)."""
        with self.lock:
            map_id = self.new_id(name)
            os.makedirs(self.path(map_id))
        return map_id

    def write_image(self, map_id, img, resolution, origin, meta):
        """Write map.pgm/yaml/thumb and meta.json. img is a top-down uint8 image."""
        mapimg.write_pgm(self.path(map_id, 'map.pgm'), img)
        mapimg.write_yaml(self.path(map_id, 'map.yaml'), 'map.pgm', resolution, origin)
        with open(self.path(map_id, 'thumb.png'), 'wb') as f:
            f.write(mapimg.thumbnail(img))
        meta = dict(meta)
        meta.update({
            'resolution': float(resolution),
            'origin': [float(o) for o in origin],
            'width': int(img.shape[1]),
            'height': int(img.shape[0]),
            'modified': now_iso(),
        })
        meta.setdefault('created', meta['modified'])
        self._write_meta(map_id, meta)
        return self.get(map_id)

    def discard_reserved(self, map_id):
        shutil.rmtree(self.path(map_id), ignore_errors=True)

    # ---- edit ----

    def load_image(self, map_id):
        y = mapimg.read_yaml(self.yaml_path(map_id))
        img = mapimg.read_pgm(self.path(map_id, y.get('image', 'map.pgm')))
        return img, float(y['resolution']), [float(v) for v in y['origin']]

    def save_edit(self, map_id, img, overwrite, new_name=None):
        src = self.get(map_id)
        _, resolution, origin = self.load_image(map_id)
        if img.shape != (src['height'], src['width']):
            raise MapError('image size %dx%d does not match the map (%dx%d)'
                           % (img.shape[1], img.shape[0], src['width'], src['height']))
        img = mapimg.normalize_image(img)
        if overwrite:
            meta = {k: v for k, v in src.items()}
            meta['edited'] = True
            meta['version'] = int(src.get('version', 1)) + 1
            return self.write_image(map_id, img, resolution, origin, meta)

        name = new_name or '%s (edited)' % src['name']
        new_id = self.reserve(name)
        # keep the pose graph alongside: it doesn't contain the edits, but it
        # still lets slam_toolbox continue mapping from this run later
        for ext in ('posegraph', 'data'):
            p = self.path(map_id, 'posegraph.' + ext)
            if os.path.isfile(p):
                shutil.copy2(p, self.path(new_id, 'posegraph.' + ext))
        meta = {
            'name': name,
            'created': now_iso(),
            'duration_s': src.get('duration_s'),
            'source': 'edit',
            'parent': map_id,
            'edited': True,
            'version': 1,
        }
        return self.write_image(new_id, img, resolution, origin, meta)

    # ---- manage ----

    def rename(self, map_id, name):
        name = (name or '').strip()
        if not name:
            raise MapError('name must not be empty')
        meta = self.get(map_id)
        meta['name'] = name[:100]
        self._write_meta(map_id, meta)
        return self.get(map_id)

    def delete(self, map_id):
        self.get(map_id)
        shutil.rmtree(self.path(map_id))

    def zip_bytes(self, map_id):
        meta = self.get(map_id)
        d = self.path(map_id)
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for name in sorted(os.listdir(d)):
                if not name.endswith('.tmp'):
                    z.write(os.path.join(d, name), arcname=os.path.join(map_id, name))
        return meta, buf.getvalue()
