import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class TeleopNode(Node):
    def __init__(self):
        super().__init__('bentobot_teleop')

        self.mcu_cmd_pub = self.create_publisher(Twist, 'mcu_cmd', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        left_analog_vert = msg.axes[1]
        left_analog_hori = msg.axes[2]

        lin_vel = 1.0 * left_analog_vert
        ang_vel = 10 * left_analog_hori

        cmd = Twist()
        cmd.linear.x = lin_vel
        cmd.angular.z = ang_vel

        self.mcu_cmd_pub.publish(cmd)


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
