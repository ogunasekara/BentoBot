#!/usr/bin/env python

import rospy
import serial
import time

from bentobot_mcu_bridge.msg import MCUInfo
from geometry_msgs.msg import Twist

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def parse_line(line):
    line = line.strip()
    vals = line.split(' ')

    if len(vals) < 3: 
        return []
    else:
        return [float(vals[0]), float(vals[1]), float(vals[2])]

def mcu_cmd_callback(msg):
    lin_vel = msg.linear.x
    ang_vel = msg.angular.z

    s = str(lin_vel) + ' ' + str(ang_vel) + '\n'
    ser.write(s.encode())

if __name__ == '__main__':
    rospy.init_node('bentobot_mcu_bridge')
    mcu_info_pub = rospy.Publisher('mcu_info', MCUInfo, queue_size=10)
    mcu_cmd_sub = rospy.Subscriber('mcu_cmd', Twist, mcu_cmd_callback)

    rate = rospy.Rate(100)

    try:
        while not rospy.is_shutdown():
            line = ser.readline().decode()
            measurements = parse_line(line)
            
            # publish if we have all measurements
            if len(measurements) == 3:
                msg = MCUInfo()
                msg.header.stamp = rospy.Time.now()
                msg.left_vel = measurements[0]
                msg.right_vel = measurements[1]
                msg.imu_ang_vel = measurements[2]
                mcu_info_pub.publish(msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        ser.close()