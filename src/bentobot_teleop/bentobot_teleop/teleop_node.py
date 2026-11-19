import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class TeleopNode(Node):
    """Maps /joy sticks to /mcu_cmd.

    Axis indices, scales and inversion are parameters (they can be changed at
    runtime with `ros2 param set`), since controllers number their axes
    differently. `~/set_enabled` (std_srvs/SetBool) pauses teleop: the node
    publishes one zero Twist and ignores /joy until it is re-enabled. The
    current state is latched on `~/enabled`.
    """

    def __init__(self):
        super().__init__('bentobot_teleop')

        self.axis_linear = self.declare_parameter('axis_linear', 1).value
        self.axis_angular = self.declare_parameter('axis_angular', 2).value
        self.scale_linear = self.declare_parameter('scale_linear', 1.0).value
        self.scale_angular = self.declare_parameter('scale_angular', 10.0).value
        self.invert_linear = self.declare_parameter('invert_linear', False).value
        self.invert_angular = self.declare_parameter('invert_angular', False).value
        self.deadzone = self.declare_parameter('deadzone', 0.0).value
        # stop the robot if /joy goes quiet for this long (joy_linux runs with
        # autorepeat, so silence means the controller is gone); 0 disables
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value
        self.enabled = self.declare_parameter('enabled', True).value
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.mcu_cmd_pub = self.create_publisher(Twist, 'mcu_cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '~/enabled', latched)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.enable_srv = self.create_service(SetBool, '~/set_enabled', self.set_enabled_callback)

        self.last_joy_time = None
        self.moving = False
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)
        self.publish_enabled()

    def on_set_parameters(self, params):
        ints = ('axis_linear', 'axis_angular')
        floats = ('scale_linear', 'scale_angular', 'deadzone', 'joy_timeout')
        bools = ('invert_linear', 'invert_angular', 'enabled')
        updates = {}
        for p in params:
            if p.name in ints:
                if not isinstance(p.value, int) or p.value < 0:
                    return SetParametersResult(successful=False, reason=p.name + ' must be an int >= 0')
                updates[p.name] = p.value
            elif p.name in floats:
                if not isinstance(p.value, (int, float)) or isinstance(p.value, bool):
                    return SetParametersResult(successful=False, reason=p.name + ' must be a number')
                updates[p.name] = float(p.value)
            elif p.name in bools:
                if not isinstance(p.value, bool):
                    return SetParametersResult(successful=False, reason=p.name + ' must be a bool')
                updates[p.name] = p.value
        for name, value in updates.items():
            setattr(self, name, value)
        if 'enabled' in updates:
            self.apply_enabled()
        return SetParametersResult(successful=True)

    def set_enabled_callback(self, request, response):
        self.enabled = bool(request.data)
        self.apply_enabled()
        response.success = True
        response.message = 'teleop ' + ('enabled' if self.enabled else 'disabled')
        self.get_logger().info(response.message)
        return response

    def apply_enabled(self):
        if not self.enabled:
            self.stop()
        self.publish_enabled()

    def publish_enabled(self):
        self.enabled_pub.publish(Bool(data=bool(self.enabled)))

    def stop(self):
        self.mcu_cmd_pub.publish(Twist())
        self.moving = False

    def axis(self, axes, index, invert):
        if index >= len(axes):
            self.get_logger().warn(
                'axis %d not present (controller has %d axes)' % (index, len(axes)),
                throttle_duration_sec=5.0)
            return None
        value = axes[index]
        if abs(value) < self.deadzone:
            value = 0.0
        return -value if invert else value

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        if not self.enabled:
            return

        lin = self.axis(msg.axes, self.axis_linear, self.invert_linear)
        ang = self.axis(msg.axes, self.axis_angular, self.invert_angular)
        if lin is None or ang is None:
            return

        cmd = Twist()
        cmd.linear.x = self.scale_linear * lin
        cmd.angular.z = self.scale_angular * ang

        self.mcu_cmd_pub.publish(cmd)
        self.moving = cmd.linear.x != 0.0 or cmd.angular.z != 0.0

    def watchdog_callback(self):
        if not self.moving or self.joy_timeout <= 0.0 or self.last_joy_time is None:
            return
        quiet = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if quiet > self.joy_timeout:
            self.get_logger().warn('no /joy for %.1f s, stopping' % quiet)
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
