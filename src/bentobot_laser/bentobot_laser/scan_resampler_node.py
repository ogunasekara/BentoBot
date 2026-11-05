import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanResamplerNode(Node):
    """Re-bins LaserScans to a fixed number of beams.

    The LD06 driver sizes each scan by the number of points it received in that
    rotation, so the beam count drifts by a few readings from scan to scan.
    slam_toolbox drops any scan whose size differs from the first one it saw.
    """

    def __init__(self):
        super().__init__('scan_resampler')

        # slightly below the LD06's typical ~455 points per scan at 10 Hz, so
        # every output beam gets at least one reading
        self.beam_count = self.declare_parameter('beam_count', 450).value

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, msg):
        n_in = len(msg.ranges)
        n_out = self.beam_count
        if n_in < 2:
            return

        has_intensities = len(msg.intensities) == n_in
        out_inc = (msg.angle_max - msg.angle_min) / (n_out - 1)

        ranges = [math.nan] * n_out
        intensities = [math.nan] * n_out

        for i in range(n_in):
            r = msg.ranges[i]
            if math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            j = int(round((angle - msg.angle_min) / out_inc))
            j = min(max(j, 0), n_out - 1)

            # keep the closest return in each beam, like the driver does
            if math.isnan(ranges[j]) or r < ranges[j]:
                ranges[j] = r
                if has_intensities:
                    intensities[j] = msg.intensities[i]

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = out_inc
        out.time_increment = msg.scan_time / (n_out - 1)
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = intensities if has_intensities else []

        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResamplerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
