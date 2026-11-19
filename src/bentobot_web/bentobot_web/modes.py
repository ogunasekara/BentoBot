"""Mode manager: runs one mode launch file at a time as a subprocess.

Each mode is a launch file in bentobot_bringup. The launch runs in its own
process group. Stopping sends SIGINT to `ros2 launch` (which shuts its nodes
down cleanly), then SIGTERM and finally SIGKILL to the whole group. The child
gets PR_SET_PDEATHSIG so it dies with the backend, and the group id is kept
in a pid file so a restarted backend can clean up after a crash.

To add a mode (e.g. navigation), add a ModeSpec to MODES.
"""
import collections
import ctypes
import os
import signal
import subprocess
import threading
import time


class ModeError(Exception):
    pass


class ModeSpec:
    def __init__(self, name, launch_file, nodes, needs_map=False, label=None):
        self.name = name
        self.launch_file = launch_file
        # nodes whose presence means the mode is up (also used to detect a
        # mode started outside the web UI, e.g. by teleop.launch.py)
        self.nodes = nodes
        self.needs_map = needs_map
        self.label = label or name

    def launch_args(self, map_yaml=None):
        if self.needs_map:
            return ['map:=' + map_yaml]
        return []


MODES = collections.OrderedDict([
    ('mapping', ModeSpec('mapping', 'mapping.launch.py', ['slam_toolbox'], label='Mapping run')),
    ('localization', ModeSpec('localization', 'localization.launch.py', ['map_server', 'amcl'],
                              needs_map=True, label='Localization run')),
])

SIGINT_TIMEOUT = 10.0
SIGTERM_TIMEOUT = 5.0
# a stopped node can linger in the ROS graph until its DDS lease (20 s) expires
EXTERNAL_GRACE = 30.0
EXTERNAL_CONFIRM = 3  # graph polls (1 s apart) before a run counts as external


def _set_pdeathsig():
    # runs in the child between fork and exec
    try:
        libc = ctypes.CDLL('libc.so.6', use_errno=True)
        libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except Exception:
        pass


def _start_time(pid):
    """Process start time (clock ticks since boot) from /proc/<pid>/stat."""
    try:
        with open('/proc/%d/stat' % pid) as f:
            # the command name (field 2) may contain spaces; it ends at the last ')'
            return f.read().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


class ModeManager:
    def __init__(self, pid_file, logger=None, package='bentobot_bringup'):
        self.pid_file = pid_file
        self.package = package
        self.logger = logger
        self.lock = threading.RLock()
        self.proc = None
        self.mode = 'idle'
        self.phase = 'idle'  # idle | starting | running | stopping
        self.map_id = None
        self.started_at = None
        self.external = False
        self.error = None
        self.log = collections.deque(maxlen=400)
        self.listeners = []
        self._stopping = False
        self._nodes = set()
        self._ended_at = 0.0
        self._external_seen = 0
        self.cleanup_stale()

    # ---- helpers ----

    def _log(self, msg):
        self.log.append('[bentobot_web] ' + msg)
        if self.logger:
            self.logger.info(msg)

    def _notify(self):
        for cb in list(self.listeners):
            try:
                cb(self.snapshot())
            except Exception as e:  # listener bugs must not break the manager
                if self.logger:
                    self.logger.error('mode listener failed: %r' % e)

    def cleanup_stale(self):
        """Kill a mode process group left behind by a crashed backend.

        The pid file stores "<pid> <start time>", so a reused pid (e.g. after a
        container restart) is never mistaken for our old launch process.
        """
        try:
            with open(self.pid_file) as f:
                pid, start = f.read().split()
            pid = int(pid)
        except (OSError, ValueError):
            return
        self._remove_pid_file()
        if _start_time(pid) != start:
            return
        try:
            os.killpg(pid, signal.SIGTERM)
            self._log('killed stale mode process group %d' % pid)
            time.sleep(2.0)
            os.killpg(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

    def _remove_pid_file(self):
        try:
            os.remove(self.pid_file)
        except OSError:
            pass

    # ---- state ----

    def snapshot(self):
        with self.lock:
            spec = MODES.get(self.mode)
            return {
                'mode': self.mode,
                'label': spec.label if spec else 'Idle',
                'phase': self.phase,
                'map_id': self.map_id,
                'started_at': self.started_at,
                'elapsed': (time.time() - self.started_at) if self.started_at else None,
                'external': self.external,
                'error': self.error,
                'pid': self.proc.pid if self.proc else None,
                'modes': [{'name': m.name, 'label': m.label, 'needs_map': m.needs_map}
                          for m in MODES.values()],
            }

    def update_nodes(self, names):
        """Called periodically with the current ROS node names."""
        changed = False
        with self.lock:
            self._nodes = set(names)
            if self.proc is not None:
                spec = MODES[self.mode]
                if self.phase == 'starting' and all(n in self._nodes for n in spec.nodes):
                    self.phase = 'running'
                    changed = True
            elif (not self.external and self.mode == 'idle'
                  and time.time() - self._ended_at > EXTERNAL_GRACE):
                # (the grace period lets nodes of a run we just stopped leave
                # the ROS graph before they could be mistaken for an external run)
                found = [spec for spec in MODES.values() if all(n in self._nodes for n in spec.nodes)]
                self._external_seen = self._external_seen + 1 if found else 0
                if found and self._external_seen >= EXTERNAL_CONFIRM:
                    spec = found[0]
                    self.mode, self.phase, self.external = spec.name, 'running', True
                    self.started_at = time.time()
                    self.error = None
                    self._log('detected %s started outside the web UI' % spec.name)
                    changed = True
            elif self.external:
                spec = MODES[self.mode]
                if not any(n in self._nodes for n in spec.nodes):
                    self._log('external %s run ended' % self.mode)
                    self.mode, self.phase, self.external = 'idle', 'idle', False
                    self.started_at = None
                    changed = True
        if changed:
            self._notify()

    # ---- start / stop ----

    def start(self, mode, map_id=None, map_yaml=None):
        spec = MODES.get(mode)
        if spec is None:
            raise ModeError('unknown mode %r' % mode)
        if spec.needs_map and not map_yaml:
            raise ModeError('%s needs a map' % mode)
        with self.lock:
            if self.mode != 'idle' or self.proc is not None:
                raise ModeError('%s is already running; stop it first' % self.mode)
            cmd = ['ros2', 'launch', self.package, spec.launch_file] + spec.launch_args(map_yaml)
            self.log.clear()
            self._log('starting: ' + ' '.join(cmd))
            try:
                # line-buffered node output, so the launch log in the UI is live
                env = dict(os.environ, PYTHONUNBUFFERED='1',
                           RCUTILS_LOGGING_BUFFERED_STREAM='0')
                self.proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    env=env, start_new_session=True, preexec_fn=_set_pdeathsig)
            except OSError as e:
                self.proc = None
                raise ModeError('failed to start %s: %s' % (mode, e))
            try:
                with open(self.pid_file, 'w') as f:
                    f.write('%d %s' % (self.proc.pid, _start_time(self.proc.pid)))
            except OSError:
                pass
            self.mode, self.phase, self.map_id = mode, 'starting', map_id
            self.started_at = time.time()
            self.external = False
            self.error = None
            self._stopping = False
            threading.Thread(target=self._watch, args=(self.proc,), daemon=True).start()
        self._notify()

    def _watch(self, proc):
        for raw in proc.stdout:
            self.log.append(raw.decode(errors='replace').rstrip())
        code = proc.wait()
        with self.lock:
            if self.proc is not proc:
                return
            if not self._stopping:
                self.error = '%s exited unexpectedly (code %s)' % (self.mode, code)
                self._log(self.error)
                # make sure nothing of the group is left running
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            else:
                self._log('%s stopped (code %s)' % (self.mode, code))
            self.proc = None
            self.mode, self.phase, self.map_id = 'idle', 'idle', None
            self.started_at = None
            self._ended_at = time.time()
            self._remove_pid_file()
        self._notify()

    def stop(self):
        """Blocking: run it in a worker thread from asyncio."""
        with self.lock:
            if self.external:
                raise ModeError('this %s run was started outside the web UI (e.g. teleop.launch.py); '
                                'stop it there' % self.mode)
            proc = self.proc
            if proc is None:
                return
            self._stopping = True
            self.phase = 'stopping'
        self._notify()
        self._log('stopping %s (SIGINT)' % self.mode)
        for sig, timeout, target in ((signal.SIGINT, SIGINT_TIMEOUT, 'proc'),
                                     (signal.SIGTERM, SIGTERM_TIMEOUT, 'group'),
                                     (signal.SIGKILL, SIGTERM_TIMEOUT, 'group')):
            try:
                if target == 'proc':
                    os.kill(proc.pid, sig)
                else:
                    self._log('escalating to %s' % signal.Signals(sig).name)
                    os.killpg(proc.pid, sig)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        # nodes that ignored SIGINT may still be around after launch exited
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self.lock:
                if self.proc is not proc:
                    return
            time.sleep(0.05)

    def shutdown(self):
        try:
            self.stop()
        except ModeError:
            pass
