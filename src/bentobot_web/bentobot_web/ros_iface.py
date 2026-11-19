"""ROS side of bentobot_web.

Spins in a background thread. Callbacks only store the latest message and
count rates; the asyncio side reads these snapshots and streams them. Heavy
work (PNG encoding) happens off the executor thread.
"""
import asyncio
import collections
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

import tf2_ros
from bentobot_interfaces.msg import MCUInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

try:
    from slam_toolbox.srv import SerializePoseGraph
except ImportError:  # slam_toolbox not installed (e.g. testing off-robot)
    SerializePoseGraph = None

from .mapimg import quaternion_yaw

# nodes of base.launch.py, shown on the dashboard
BASE_NODES = ['joy', 'bentobot_teleop', 'bentobot_mcu_bridge', 'LD06', 'scan_resampler',
              'bentobot_odom', 'laser_transform', 'rosbridge_websocket']

STALE_AFTER = 1.0

TELEOP_PARAMS = ['axis_linear', 'axis_angular', 'scale_linear', 'scale_angular',
                 'invert_linear', 'invert_angular', 'deadzone', 'joy_timeout']


class RateTracker:
    def __init__(self, window=2.0, stale=True):
        self.window = window
        self.stale = stale  # whether going quiet should be flagged
        self.times = collections.deque(maxlen=1000)
        self.count = 0

    def tick(self):
        self.times.append(time.monotonic())
        self.count += 1

    def snapshot(self, now):
        while self.times and now - self.times[0] > self.window:
            self.times.popleft()
        n = len(self.times)
        if n >= 2:
            rate = (n - 1) / max(self.times[-1] - self.times[0], 1e-3)
        else:
            rate = 0.0
        age = (now - self.times[-1]) if self.times else None
        return {
            'rate': round(rate, 1),
            'age': round(age, 2) if age is not None else None,
            'stale': self.stale and (age is None or age > STALE_AFTER),
            'count': self.count,
        }


def _param_value(v):
    if isinstance(v, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    if isinstance(v, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    if isinstance(v, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(v))


def _from_param_value(pv):
    return {
        ParameterType.PARAMETER_BOOL: lambda: pv.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: pv.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: pv.double_value,
        ParameterType.PARAMETER_STRING: lambda: pv.string_value,
    }.get(pv.type, lambda: None)()


def _yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WebNode(Node):
    def __init__(self):
        super().__init__('bentobot_web')

        self.p_startup_mode = str(self.declare_parameter('startup_mode', 'idle').value)
        self.p_port = int(self.declare_parameter('port', 8080).value)
        self.p_iface = str(self.declare_parameter('iface', 'wlan0').value)
        self.p_data_dir = str(self.declare_parameter('data_dir', '/bentobot_ws/data').value)
        self.p_static_dir = str(self.declare_parameter('static_dir', '/opt/bentobot_web/static').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)

        self.rates = {
            '/scan': RateTracker(),
            '/mcu_info': RateTracker(),
            '/odom': RateTracker(),
            '/joy': RateTracker(),
            '/mcu_cmd': RateTracker(stale=False),
            '/map': RateTracker(window=30.0, stale=False),
            '/amcl_pose': RateTracker(window=10.0, stale=False),
        }
        self.scan = None
        self.scan_seq = 0
        self.mcu_info = None
        self.odom = None
        self.joy = None
        self.mcu_cmd = None
        self.map_msg = None
        self.map_seq = 0
        self.amcl = None
        self.teleop_enabled = None
        self.node_names = []

        self.pose = None        # robot pose in map (or odom) frame
        self.laser_pose = None  # scan frame pose in the same frame
        self.path = []          # [(x, y)] in self.path_frame
        self.path_frame = None
        self.path_gen = 0       # bumped when the path is reset

        sensor = qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self.on_scan, sensor)
        self.create_subscription(MCUInfo, '/mcu_info', self.on_mcu_info, sensor)
        self.create_subscription(Odometry, '/odom', self.on_odom, sensor)
        self.create_subscription(Joy, '/joy', self.on_joy, sensor)
        self.create_subscription(Twist, '/mcu_cmd', self.on_mcu_cmd, sensor)
        # slam_toolbox and map_server both latch /map
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(Bool, '/bentobot_teleop/enabled', self.on_teleop_enabled, latched)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.mcu_cmd_pub = self.create_publisher(Twist, '/mcu_cmd', 10)

        self.cli_teleop_enable = self.create_client(SetBool, '/bentobot_teleop/set_enabled')
        self.cli_teleop_set = self.create_client(SetParameters, '/bentobot_teleop/set_parameters')
        self.cli_teleop_get = self.create_client(GetParameters, '/bentobot_teleop/get_parameters')
        self.cli_serialize = (self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
                              if SerializePoseGraph else None)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self.pose_tick)
        self.create_timer(1.0, self.graph_tick)
        self.graph_listeners = []

    # ---- subscriptions ----

    def on_scan(self, msg):
        self.rates['/scan'].tick()
        self.scan = msg
        self.scan_seq += 1

    def on_mcu_info(self, msg):
        self.rates['/mcu_info'].tick()
        self.mcu_info = msg

    def on_odom(self, msg):
        self.rates['/odom'].tick()
        self.odom = msg

    def on_joy(self, msg):
        self.rates['/joy'].tick()
        self.joy = msg

    def on_mcu_cmd(self, msg):
        self.rates['/mcu_cmd'].tick()
        self.mcu_cmd = msg

    def on_map(self, msg):
        self.rates['/map'].tick()
        self.map_msg = msg
        self.map_seq += 1

    def on_amcl(self, msg):
        self.rates['/amcl_pose'].tick()
        self.amcl = msg

    def on_teleop_enabled(self, msg):
        self.teleop_enabled = bool(msg.data)

    # ---- timers ----

    def _lookup(self, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        return {'frame': target, 'x': tr.x, 'y': tr.y, 'yaw': quaternion_yaw(q), 'age': round(age, 2)}

    def pose_tick(self):
        pose = self._lookup(self.map_frame, self.base_frame)
        # a map -> odom TF older than a few seconds is left over from a run that ended
        if pose is None or pose['age'] > 3.0:
            pose = self._lookup(self.odom_frame, self.base_frame)
        self.pose = pose
        if pose is None:
            self.laser_pose = None
            return
        scan = self.scan
        self.laser_pose = self._lookup(pose['frame'], scan.header.frame_id) if scan else None

        if pose['frame'] != self.path_frame:
            self.reset_path(pose['frame'])
        if not self.path or math.hypot(pose['x'] - self.path[-1][0], pose['y'] - self.path[-1][1]) > 0.05:
            self.path.append((round(pose['x'], 3), round(pose['y'], 3)))
            if len(self.path) > 20000:
                self.path = self.path[::2]
                self.path_gen += 1

    def reset_path(self, frame=None):
        self.path = []
        self.path_frame = frame
        self.path_gen += 1

    def graph_tick(self):
        try:
            self.node_names = sorted(set(self.get_node_names()))
        except Exception:
            return
        for cb in self.graph_listeners:
            cb(self.node_names)

    # ---- snapshots for the web side ----

    def topic_snapshot(self):
        now = time.monotonic()
        return {name: r.snapshot(now) for name, r in self.rates.items()}

    def telemetry(self):
        out = {'pose': self.pose}
        o = self.odom
        if o is not None:
            p, t = o.pose.pose, o.twist.twist
            out['odom'] = {'x': p.position.x, 'y': p.position.y, 'yaw': quaternion_yaw(p.orientation),
                           'v': t.linear.x, 'w': t.angular.z}
        m = self.mcu_info
        if m is not None:
            out['mcu_info'] = {'left': m.left_vel, 'right': m.right_vel, 'gyro_z': m.imu_ang_vel}
        c = self.mcu_cmd
        if c is not None:
            out['mcu_cmd'] = {'v': c.linear.x, 'w': c.angular.z}
        j = self.joy
        if j is not None:
            out['joy'] = {'axes': [round(a, 3) for a in j.axes], 'buttons': list(j.buttons)}
        a = self.amcl
        if a is not None:
            p, cov = a.pose.pose, a.pose.covariance
            out['amcl'] = {
                'x': p.position.x, 'y': p.position.y, 'yaw': quaternion_yaw(p.orientation),
                'sx': math.sqrt(max(cov[0], 0.0)), 'sy': math.sqrt(max(cov[7], 0.0)),
                'syaw': math.sqrt(max(cov[35], 0.0)),
            }
        return out

    def clear_amcl(self):
        self.amcl = None

    # ---- actions ----

    def publish_stop(self):
        self.mcu_cmd_pub.publish(Twist())

    def publish_initial_pose(self, x, y, yaw, sxy=0.5, syaw=math.radians(15)):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = cov[7] = sxy * sxy
        cov[35] = syaw * syaw
        msg.pose.covariance = cov
        self.initialpose_pub.publish(msg)


async def ros_call(client, request, timeout=5.0):
    """Await an rclpy service call from asyncio (the executor spins in another thread)."""
    if client is None:
        raise RuntimeError('service client not available')
    if not client.service_is_ready():
        loop = asyncio.get_running_loop()
        ready = await loop.run_in_executor(None, client.wait_for_service, min(timeout, 2.0))
        if not ready:
            raise RuntimeError('service %s is not available' % client.srv_name)
    loop = asyncio.get_running_loop()
    afut = loop.create_future()

    def done(f):
        def resolve():
            if afut.done():
                return
            exc = f.exception()
            if exc is not None:
                afut.set_exception(exc)
            else:
                afut.set_result(f.result())
        loop.call_soon_threadsafe(resolve)

    fut = client.call_async(request)
    fut.add_done_callback(done)
    return await asyncio.wait_for(afut, timeout)


async def teleop_set_enabled(node, enabled):
    req = SetBool.Request()
    req.data = bool(enabled)
    res = await ros_call(node.cli_teleop_enable, req, timeout=3.0)
    return res.success


async def teleop_set_params(node, values):
    req = SetParameters.Request()
    req.parameters = [Parameter(name=k, value=_param_value(v)) for k, v in values.items()]
    res = await ros_call(node.cli_teleop_set, req, timeout=3.0)
    failed = [r.reason for r in res.results if not r.successful]
    if failed:
        raise RuntimeError('; '.join(failed))


async def teleop_get_params(node):
    req = GetParameters.Request()
    req.names = TELEOP_PARAMS
    res = await ros_call(node.cli_teleop_get, req, timeout=3.0)
    return {n: _from_param_value(v) for n, v in zip(TELEOP_PARAMS, res.values)}


async def serialize_pose_graph(node, prefix):
    req = SerializePoseGraph.Request()
    req.filename = prefix
    res = await ros_call(node.cli_serialize, req, timeout=30.0)
    return res
