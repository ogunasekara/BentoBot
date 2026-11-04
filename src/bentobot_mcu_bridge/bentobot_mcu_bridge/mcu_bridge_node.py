import threading

import rclpy
import serial
from rclpy.node import Node

from bentobot_interfaces.msg import MCUInfo
from geometry_msgs.msg import Twist


def parse_line(line):
    line = line.strip()
    vals = line.split(' ')

    if len(vals) < 3:
        return []

    try:
        return [float(vals[0]), float(vals[1]), float(vals[2])]
    except ValueError:
        return []


class MCUBridgeNode(Node):
    def __init__(self):
        super().__init__('bentobot_mcu_bridge')

        port = self.declare_parameter(
            'serial_port', '/dev/serial/by-id/usb-Teensyduino_USB_Serial_14093920-if00').value
        baudrate = self.declare_parameter('baudrate', 115200).value

        self.ser = serial.Serial(port, baudrate, timeout=1)

        self.mcu_info_pub = self.create_publisher(MCUInfo, 'mcu_info', 10)
        self.mcu_cmd_sub = self.create_subscription(Twist, 'mcu_cmd', self.mcu_cmd_callback, 10)

        # serial reads block, so they run outside the executor
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def mcu_cmd_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        s = str(lin_vel) + ' ' + str(ang_vel) + '\n'
        self.ser.write(s.encode())

    def read_loop(self):
        while rclpy.ok():
            line = self.ser.readline().decode(errors='ignore')
            measurements = parse_line(line)

            # publish if we have all measurements
            if len(measurements) == 3:
                msg = MCUInfo()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.left_vel = measurements[0]
                msg.right_vel = measurements[1]
                msg.imu_ang_vel = measurements[2]
                self.mcu_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MCUBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
