"""HTTP API, websocket hub and static file serving (aiohttp).

Websocket protocol (GET /ws), server -> browser only:
  text   {"type": "status", ...}     2 Hz   mode, nodes, topic rates, system, controller
  text   {"type": "telemetry", ...} 10 Hz   pose, odom, mcu_info, mcu_cmd, joy, amcl
  text   {"type": "path", "reset": bool, "frame": str, "points": [[x, y], ...]}
  binary <uint32 LE header length><JSON header><payload>
         header.type == "scan": payload = float32 ranges (0 = no return), 5 Hz
         header.type == "map":  payload = PNG (8-bit gray, top row first), on change
Commands go through the HTTP API.
"""
import asyncio
import json
import math
import os
import signal
import struct
import time
import traceback

import numpy as np
import psutil
from aiohttp import WSMsgType, web

from . import mapimg, ros_iface
from .bluetooth import BluetoothError
from .controllers import clean_params
from .maps import MapError, now_iso
from .modes import MODES, ModeError
from .system import joystick_info

STATUS_PERIOD = 0.5
TELEMETRY_PERIOD = 0.1
SCAN_PERIOD = 0.2
MAP_MIN_PERIOD = 1.0


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


def pack_binary(header, payload):
    h = json.dumps(header).encode()
    return struct.pack('<I', len(h)) + h + payload


class Client:
    def __init__(self, ws):
        self.ws = ws
        self.pending = {}  # key -> str | bytes; newer payloads replace unsent ones
        self.event = asyncio.Event()
        self.path_gen = None
        self.path_index = 0

    def put(self, key, payload):
        self.pending[key] = payload
        self.event.set()


class WebApp:
    def __init__(self, node, modes, store, profiles, bt, stats, logger):
        self.node = node
        self.modes = modes
        self.store = store
        self.profiles = profiles
        self.bt = bt
        self.stats = stats
        self.log = logger
        self.loop = None
        self.clients = set()
        self.tasks = []

        self.map_payload = None  # (key, bytes) of the map currently shown
        self.map_key = None
        self.map_encoding = False
        self.map_last_encode = 0.0
        self.file_map_id = None  # set while a localization run shows a map from disk
        self.current_run = ('idle', None, None)

        self.controller = {'present': False}
        self.bt_connected = []
        self.applied_profile = None
        self.teleop_seen = False
        self.last_status = None

        self.app = web.Application(middlewares=[self.error_middleware], client_max_size=64 * 1024 * 1024)
        self._routes()

    # ---- setup ----

    def _routes(self):
        r = self.app.router
        r.add_get('/ws', self.ws_handler)
        r.add_get('/api/status', self.get_status)
        r.add_get('/api/mode', self.get_mode)
        r.add_post('/api/mode/stop', self.stop_mode)
        r.add_get('/api/mode/log', self.get_mode_log)
        r.add_post('/api/mode/mapping/save', self.save_mapping)
        r.add_post('/api/mode/mapping/discard', self.discard_mapping)
        r.add_post('/api/mode/{mode}/start', self.start_mode)
        r.add_post('/api/teleop/enabled', self.set_teleop_enabled)
        r.add_get('/api/teleop/params', self.get_teleop_params)
        r.add_put('/api/teleop/params', self.put_teleop_params)
        r.add_get('/api/controllers', self.get_controllers)
        r.add_put('/api/controllers/{key}', self.put_controller)
        r.add_delete('/api/controllers/{key}', self.delete_controller)
        r.add_post('/api/joy/restart', self.restart_joy)
        r.add_post('/api/localization/initialpose', self.initial_pose)
        r.add_get('/api/maps', self.list_maps)
        r.add_get('/api/maps/{id}', self.get_map)
        r.add_patch('/api/maps/{id}', self.rename_map)
        r.add_delete('/api/maps/{id}', self.delete_map)
        r.add_get('/api/maps/{id}/download', self.download_map)
        r.add_get('/api/maps/{id}/thumb.png', self.map_thumb)
        r.add_get('/api/maps/{id}/map.pgm', self.map_pgm)
        r.add_put('/api/maps/{id}/image', self.save_map_image)
        r.add_get('/api/bt/status', self.bt_status)
        r.add_get('/api/bt/devices', self.bt_devices)
        r.add_post('/api/bt/scan', self.bt_scan)
        r.add_post('/api/bt/power', self.bt_power)
        r.add_post('/api/bt/devices/{address}/{action}', self.bt_action)
        r.add_delete('/api/bt/devices/{address}', self.bt_remove)
        static = self.node.p_static_dir
        if os.path.isdir(os.path.join(static, 'assets')):
            r.add_static('/assets', os.path.join(static, 'assets'))
        r.add_get('/{tail:.*}', self.index)

    @web.middleware
    async def error_middleware(self, request, handler):
        try:
            return await handler(request)
        except ApiError as e:
            return web.json_response({'error': e.message}, status=e.status)
        except (ModeError, MapError, BluetoothError, ValueError) as e:
            return web.json_response({'error': str(e)}, status=400)
        except web.HTTPException:
            raise
        except Exception as e:
            self.log.error('%s %s failed: %s' % (request.method, request.path, traceback.format_exc()))
            return web.json_response({'error': '%s: %s' % (type(e).__name__, e)}, status=500)

    async def start(self):
        self.loop = asyncio.get_running_loop()
        self.modes.listeners.append(lambda snap: self.loop.call_soon_threadsafe(self.on_mode_change, snap))
        self.node.graph_listeners.append(self.modes.update_nodes)
        self.tasks = [asyncio.ensure_future(self._guard(coro))
                      for coro in (self.status_loop(), self.telemetry_loop(), self.controller_loop())]
        if self.node.p_startup_mode in MODES and not MODES[self.node.p_startup_mode].needs_map:
            asyncio.ensure_future(self.startup_mode(self.node.p_startup_mode))
        elif self.node.p_startup_mode not in ('', 'idle'):
            self.log.warn('ignoring startup_mode %r' % self.node.p_startup_mode)

    async def stop(self):
        for t in self.tasks:
            t.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for c in list(self.clients):
            await c.ws.close()

    async def _guard(self, coro):
        try:
            await coro
        except asyncio.CancelledError:
            pass
        except Exception:
            self.log.error('background task crashed: %s' % traceback.format_exc())

    async def startup_mode(self, mode):
        # give the drivers a moment, and don't fight an external run (teleop.launch.py)
        await asyncio.sleep(3.0)
        snap = self.modes.snapshot()
        if snap['mode'] != 'idle':
            return
        try:
            self.modes.start(mode)
        except ModeError as e:
            self.log.error('startup mode failed: %s' % e)

    def on_mode_change(self, snap):
        """Runs on the asyncio loop whenever the mode manager changes state."""
        run = (snap['mode'], snap['map_id'], snap['started_at'])
        if run != self.current_run:  # a run started or ended (not just a phase change)
            self.current_run = run
            self.node.reset_path()
            self.node.clear_amcl()
            if snap['mode'] == 'localization' and snap['map_id']:
                self.file_map_id = snap['map_id']
                asyncio.ensure_future(self.load_file_map(snap['map_id']))
            else:
                # live /map from slam_toolbox (or an external run); a new run must
                # not show or save the previous run's map
                self.file_map_id = None
                self.node.map_msg = None
                self.set_map(None)
        self.push_status()

    # ---- map streaming ----

    def set_map(self, payload, key=None):
        self.map_payload = payload
        self.map_key = key
        for c in self.clients:
            c.put('map', payload if payload is not None else pack_binary({'type': 'map', 'clear': True}, b''))

    async def load_file_map(self, map_id):
        try:
            img, res, origin = await self.loop.run_in_executor(None, self.store.load_image, map_id)
            png = await self.loop.run_in_executor(None, mapimg.png_bytes, img)
        except Exception as e:
            self.log.error('loading map %s failed: %s' % (map_id, e))
            return
        if self.file_map_id != map_id:
            return
        header = {'type': 'map', 'width': img.shape[1], 'height': img.shape[0], 'resolution': res,
                  'origin': origin, 'source': 'file', 'map_id': map_id, 'stamp': time.time()}
        self.set_map(pack_binary(header, png), key=('file', map_id))

    async def encode_live_map(self):
        msg, seq = self.node.map_msg, self.node.map_seq
        self.map_encoding = True
        try:
            info = msg.info

            def work():
                img = mapimg.grid_to_image(info.width, info.height, msg.data)
                return mapimg.png_bytes(img, compress_level=3)
            png = await self.loop.run_in_executor(None, work)
            o = info.origin
            header = {'type': 'map', 'width': info.width, 'height': info.height,
                      'resolution': info.resolution,
                      'origin': [o.position.x, o.position.y, mapimg.quaternion_yaw(o.orientation)],
                      'source': 'live', 'stamp': time.time()}
            if self.file_map_id is None and self.node.map_msg is not None:
                self.set_map(pack_binary(header, png), key=('live', seq))
        finally:
            self.map_encoding = False
            self.map_last_encode = time.monotonic()

    # ---- periodic loops ----

    def status(self):
        node = self.node
        snap = self.modes.snapshot()
        names = set(node.node_names)
        expected = list(ros_iface.BASE_NODES)
        spec = MODES.get(snap['mode'])
        if spec:
            expected += spec.nodes
        return {
            'type': 'status',
            'time': time.time(),
            'mode': snap,
            'nodes': {n: n in names for n in expected},
            'all_nodes': sorted(names),
            'topics': node.topic_snapshot(),
            'system': self.stats.snapshot(),
            'controller': dict(self.controller, bluetooth=self.bt_connected,
                               profile=self.applied_profile),
            'teleop_enabled': node.teleop_enabled,
            'map': {'available': self.map_payload is not None,
                    'source': self.map_key[0] if self.map_key else None},
            'web': {'port': node.p_port, 'iface': node.p_iface},
        }

    def push_status(self):
        self.last_status = json.dumps(self.status())
        for c in self.clients:
            c.put('status', self.last_status)

    async def status_loop(self):
        while True:
            if self.clients:
                self.push_status()
            await asyncio.sleep(STATUS_PERIOD)

    async def telemetry_loop(self):
        node = self.node
        last_scan_seq, last_scan_time = 0, 0.0
        while True:
            await asyncio.sleep(TELEMETRY_PERIOD)
            now = time.monotonic()

            # live map (mapping run / external runs)
            if (self.file_map_id is None and node.map_msg is not None and not self.map_encoding
                    and self.map_key != ('live', node.map_seq)
                    and now - self.map_last_encode >= MAP_MIN_PERIOD):
                asyncio.ensure_future(self._guard(self.encode_live_map()))

            if not self.clients:
                continue

            t = node.telemetry()
            t['type'] = 'telemetry'
            t['teleop_enabled'] = node.teleop_enabled
            text = json.dumps(t)
            for c in self.clients:
                c.put('telemetry', text)

            scan, lp = node.scan, node.laser_pose
            if (scan is not None and lp is not None and node.scan_seq != last_scan_seq
                    and now - last_scan_time >= SCAN_PERIOD):
                last_scan_seq, last_scan_time = node.scan_seq, now
                r = np.asarray(scan.ranges, dtype=np.float32)
                bad = ~np.isfinite(r) | (r < scan.range_min) | (r > scan.range_max)
                r = np.where(bad, np.float32(0.0), r)
                header = {'type': 'scan', 'angle_min': scan.angle_min,
                          'angle_increment': scan.angle_increment, 'pose': lp}
                payload = pack_binary(header, r.astype('<f4').tobytes())
                for c in self.clients:
                    c.put('scan', payload)

            for c in self.clients:
                if c.path_gen != node.path_gen or c.path_index < len(node.path):
                    c.event.set()  # the sender computes the path delta itself

    async def controller_loop(self):
        """Tracks the joystick, and applies a saved axis profile when it changes."""
        bt_poll = 0
        while True:
            js = await self.loop.run_in_executor(None, joystick_info)
            self.controller = js
            teleop_up = 'bentobot_teleop' in self.node.node_names
            if not teleop_up:
                self.applied_profile = None  # it'll come back with default parameters
            if bt_poll <= 0:
                self.bt_connected = await self.bt.connected_controllers()
                bt_poll = 3
            bt_poll -= 1
            key = js.get('key') if js.get('present') else None
            if key and teleop_up and self.applied_profile != key:
                prof = self.profiles.get(key)
                if prof:
                    try:
                        await ros_iface.teleop_set_params(self.node, prof['params'])
                        self.applied_profile = key
                        self.log.info('applied controller profile %r' % prof['name'])
                    except Exception as e:
                        self.log.warn('applying controller profile failed: %s' % e)
                else:
                    self.applied_profile = None
            await asyncio.sleep(1.0)

    # ---- websocket ----

    async def ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=10.0, compress=False)
        await ws.prepare(request)
        client = Client(ws)
        self.clients.add(client)
        client.put('status', json.dumps(self.status()))
        if self.map_payload is not None:
            client.put('map', self.map_payload)
        sender = asyncio.ensure_future(self._sender(client))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            self.clients.discard(client)
            sender.cancel()
        return ws

    async def _sender(self, client):
        node = self.node
        try:
            while not client.ws.closed:
                await client.event.wait()
                client.event.clear()
                pending, client.pending = client.pending, {}
                for key in ('status', 'map', 'telemetry', 'scan'):
                    payload = pending.get(key)
                    if payload is None:
                        continue
                    if isinstance(payload, bytes):
                        await client.ws.send_bytes(payload)
                    else:
                        await client.ws.send_str(payload)
                path = node.path
                if client.path_gen != node.path_gen:
                    client.path_gen, client.path_index = node.path_gen, 0
                    await client.ws.send_str(json.dumps(
                        {'type': 'path', 'reset': True, 'frame': node.path_frame, 'points': path}))
                    client.path_index = len(path)
                elif client.path_index < len(path):
                    pts = path[client.path_index:]
                    client.path_index = len(path)
                    await client.ws.send_str(json.dumps(
                        {'type': 'path', 'reset': False, 'frame': node.path_frame, 'points': pts}))
        except (asyncio.CancelledError, ConnectionResetError, RuntimeError):
            pass

    # ---- helpers ----

    @staticmethod
    async def body(request):
        if not request.can_read_body:
            return {}
        try:
            data = await request.json()
        except ValueError:
            raise ApiError(400, 'invalid JSON body')
        if not isinstance(data, dict):
            raise ApiError(400, 'JSON body must be an object')
        return data

    async def blocking(self, fn, *args):
        return await self.loop.run_in_executor(None, fn, *args)

    # ---- static ----

    async def index(self, request):
        if request.path.startswith('/api/'):
            raise ApiError(404, 'no such endpoint')
        index = os.path.join(self.node.p_static_dir, 'index.html')
        if not os.path.isfile(index):
            return web.Response(text='bentobot_web is running, but the frontend is not built '
                                     '(%s missing). The API is at /api/status.' % index,
                                status=503)
        return web.FileResponse(index, headers={'Cache-Control': 'no-cache'})

    # ---- status / mode ----

    async def get_status(self, request):
        s = self.status()
        s['telemetry'] = self.node.telemetry()
        return web.json_response(s)

    async def get_mode(self, request):
        return web.json_response(self.modes.snapshot())

    async def get_mode_log(self, request):
        return web.json_response({'lines': list(self.modes.log)})

    async def start_mode(self, request):
        mode = request.match_info['mode']
        body = await self.body(request)
        if mode not in MODES:
            raise ApiError(404, 'unknown mode %r' % mode)
        map_id = body.get('map_id')
        yaml_path = None
        if MODES[mode].needs_map:
            if not map_id or not self.store.exists(map_id):
                raise ApiError(400, 'pick a saved map')
            yaml_path = self.store.yaml_path(map_id)
        self.modes.start(mode, map_id=map_id, map_yaml=yaml_path)
        return web.json_response(self.modes.snapshot())

    async def stop_mode(self, request):
        await self.blocking(self.modes.stop)
        return web.json_response(self.modes.snapshot())

    async def save_mapping(self, request):
        body = await self.body(request)
        name = (body.get('name') or '').strip()
        if not name:
            raise ApiError(400, 'enter a map name')
        stop = bool(body.get('stop', True))
        snap = self.modes.snapshot()
        if snap['mode'] != 'mapping' or snap['phase'] != 'running':
            raise ApiError(409, 'no mapping run is active')
        msg = self.node.map_msg
        if msg is None:
            raise ApiError(409, 'no /map received yet; drive around a bit first')

        map_id = await self.blocking(self.store.reserve, name)
        warnings = []
        try:
            prefix = self.store.posegraph_prefix(map_id)
            await ros_iface.serialize_pose_graph(self.node, prefix)
            # Foxy's SerializePoseGraph response is empty, so check the files, and
            # don't stop slam_toolbox while it may still be writing them
            if not await self.wait_for_files([prefix + '.posegraph', prefix + '.data']):
                warnings.append('slam_toolbox did not finish writing the pose graph files')
        except Exception as e:
            warnings.append('pose graph not saved: %s' % e)

        info = msg.info
        o = info.origin
        origin = [o.position.x, o.position.y, mapimg.quaternion_yaw(o.orientation)]
        meta = {'name': name[:100], 'created': now_iso(), 'duration_s': snap['elapsed'],
                'source': 'mapping', 'version': 1}
        try:
            img = await self.blocking(mapimg.grid_to_image, info.width, info.height, msg.data)
            meta = await self.blocking(self.store.write_image, map_id, img, info.resolution, origin, meta)
        except Exception:
            await self.blocking(self.store.discard_reserved, map_id)
            raise
        if stop and not snap['external']:
            await self.blocking(self.modes.stop)
        elif stop:
            warnings.append('this run was started outside the web UI, so it keeps running')
        return web.json_response({'map': meta, 'warnings': warnings})

    @staticmethod
    async def wait_for_files(paths, timeout=60.0):
        """True once all files exist and their sizes held still for a second."""
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            sizes = tuple(os.path.getsize(p) if os.path.isfile(p) else -1 for p in paths)
            if min(sizes) > 0 and sizes == last:
                return True
            last = sizes
            await asyncio.sleep(1.0)
        return False

    async def discard_mapping(self, request):
        snap = self.modes.snapshot()
        if snap['mode'] != 'mapping':
            raise ApiError(409, 'no mapping run is active')
        await self.blocking(self.modes.stop)
        return web.json_response(self.modes.snapshot())

    # ---- teleop / controllers ----

    async def set_teleop_enabled(self, request):
        body = await self.body(request)
        enabled = bool(body.get('enabled'))
        if not enabled:
            # belt and braces: zero the motors directly too, even if teleop is down
            self.node.publish_stop()
        try:
            await ros_iface.teleop_set_enabled(self.node, enabled)
        except Exception as e:
            raise ApiError(503, 'teleop did not respond (%s). The motors were sent a zero command, '
                                'but teleop may resume on the next /joy message.' % e)
        return web.json_response({'enabled': enabled})

    async def get_teleop_params(self, request):
        try:
            params = await ros_iface.teleop_get_params(self.node)
        except Exception as e:
            raise ApiError(503, 'teleop did not respond: %s' % e)
        return web.json_response(params)

    async def put_teleop_params(self, request):
        params = clean_params(await self.body(request))
        try:
            await ros_iface.teleop_set_params(self.node, params)
        except Exception as e:
            raise ApiError(503, 'setting teleop parameters failed: %s' % e)
        return web.json_response(params)

    async def get_controllers(self, request):
        return web.json_response({'current': self.controller, 'applied': self.applied_profile,
                                  'profiles': self.profiles.all()})

    async def put_controller(self, request):
        key = request.match_info['key']
        body = await self.body(request)
        prof = self.profiles.save(key, body.get('name') or key, body.get('params') or {})
        if self.controller.get('key') == key:
            try:
                await ros_iface.teleop_set_params(self.node, prof['params'])
                self.applied_profile = key
            except Exception as e:
                raise ApiError(503, 'profile saved, but applying it failed: %s' % e)
        return web.json_response(prof)

    async def delete_controller(self, request):
        key = request.match_info['key']
        self.profiles.delete(key)
        if self.applied_profile == key:
            self.applied_profile = None
        return web.json_response({'deleted': key})

    async def restart_joy(self, request):
        n = await self.blocking(self._kill_joy)
        if not n:
            raise ApiError(404, 'joy_linux_node is not running')
        return web.json_response({'restarted': n})

    @staticmethod
    def _kill_joy():
        # base.launch.py starts joy with respawn=True, so killing it restarts it
        n = 0
        for p in psutil.process_iter(['cmdline']):
            cmd = p.info.get('cmdline') or []
            if any(part.endswith('joy_linux_node') for part in cmd):
                try:
                    p.send_signal(signal.SIGINT)
                    n += 1
                except psutil.Error:
                    pass
        return n

    async def wait_for_joystick(self, timeout=10.0):
        """After a controller connects: js0 present? /joy flowing? Restart joy once if not."""
        deadline = time.monotonic() + timeout
        js = joystick_info()
        while not js['present'] and time.monotonic() < deadline:
            await asyncio.sleep(0.5)
            js = joystick_info()
        if not js['present']:
            return {'js0': False, 'joy': False, 'restarted': False}

        def joy_ok():
            return not self.node.topic_snapshot()['/joy']['stale']
        restarted = False
        for _ in range(6):
            if joy_ok():
                break
            await asyncio.sleep(0.5)
        if not joy_ok():
            restarted = bool(await self.blocking(self._kill_joy))
            for _ in range(10):
                await asyncio.sleep(0.5)
                if joy_ok():
                    break
        return {'js0': True, 'name': js.get('name'), 'joy': joy_ok(), 'restarted': restarted}

    async def initial_pose(self, request):
        body = await self.body(request)
        try:
            x, y, yaw = float(body['x']), float(body['y']), float(body['yaw'])
        except (KeyError, TypeError, ValueError):
            raise ApiError(400, 'x, y and yaw are required')
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ApiError(400, 'x, y and yaw must be finite')
        self.node.publish_initial_pose(x, y, yaw)
        return web.json_response({'x': x, 'y': y, 'yaw': yaw})

    # ---- maps ----

    def _in_use(self, map_id):
        snap = self.modes.snapshot()
        return snap['mode'] == 'localization' and snap['map_id'] == map_id

    async def list_maps(self, request):
        return web.json_response(await self.blocking(self.store.list))

    async def get_map(self, request):
        return web.json_response(await self.blocking(self.store.get, request.match_info['id']))

    async def rename_map(self, request):
        body = await self.body(request)
        meta = await self.blocking(self.store.rename, request.match_info['id'], body.get('name'))
        return web.json_response(meta)

    async def delete_map(self, request):
        map_id = request.match_info['id']
        if self._in_use(map_id):
            raise ApiError(409, 'this map is in use by the localization run')
        await self.blocking(self.store.delete, map_id)
        return web.json_response({'deleted': map_id})

    async def download_map(self, request):
        meta, data = await self.blocking(self.store.zip_bytes, request.match_info['id'])
        return web.Response(body=data, content_type='application/zip', headers={
            'Content-Disposition': 'attachment; filename="%s.zip"' % meta['id']})

    async def map_thumb(self, request):
        p = self.store.path(request.match_info['id'], 'thumb.png')
        if not os.path.isfile(p):
            raise ApiError(404, 'no thumbnail')
        return web.FileResponse(p, headers={'Cache-Control': 'no-cache'})

    async def map_pgm(self, request):
        map_id = request.match_info['id']
        self.store.get(map_id)
        return web.FileResponse(self.store.path(map_id, 'map.pgm'), headers={
            'Cache-Control': 'no-cache', 'Content-Type': 'image/x-portable-graymap'})

    async def save_map_image(self, request):
        """Body: raw uint8 pixels (width*height, top row first), from the editor."""
        map_id = request.match_info['id']
        mode = request.query.get('as', 'new')
        if mode not in ('new', 'overwrite'):
            raise ApiError(400, 'as must be new or overwrite')
        try:
            w, h = int(request.query['width']), int(request.query['height'])
        except (KeyError, ValueError):
            raise ApiError(400, 'width and height are required')
        data = await request.read()
        if len(data) != w * h:
            raise ApiError(400, 'expected %d bytes, got %d' % (w * h, len(data)))
        img = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        meta = await self.blocking(self.store.save_edit, map_id, img, mode == 'overwrite',
                                   request.query.get('name'))
        warnings = []
        if mode == 'overwrite' and self._in_use(map_id):
            warnings.append('the running localization still uses the old map; restart the run to load the edits')
            if self.file_map_id == map_id:
                await self.load_file_map(map_id)
        return web.json_response({'map': meta, 'warnings': warnings})

    # ---- bluetooth ----

    async def bt_status(self, request):
        return web.json_response(await self.bt.status())

    async def bt_devices(self, request):
        status = await self.bt.status()
        devices = await self.bt.devices() if status.get('available') else []
        return web.json_response({'status': status, 'devices': devices})

    async def bt_scan(self, request):
        body = await self.body(request)
        await self.bt.scan(bool(body.get('on', True)))
        return web.json_response(await self.bt.status())

    async def bt_power(self, request):
        body = await self.body(request)
        await self.bt.set_powered(bool(body.get('on', True)))
        return web.json_response(await self.bt.status())

    async def bt_action(self, request):
        address = request.match_info['address']
        action = request.match_info['action']
        await self.bt.action(address, action)
        result = {'address': address, 'action': action}
        if action in ('connect', 'pair'):
            result['joystick'] = await self.wait_for_joystick()
        self.bt_connected = await self.bt.connected_controllers()
        return web.json_response(result)

    async def bt_remove(self, request):
        await self.bt.action(request.match_info['address'], 'remove')
        return web.json_response({'removed': request.match_info['address']})
