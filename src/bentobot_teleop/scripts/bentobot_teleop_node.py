#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joy_callback(msg):
    left_analog_vert = msg.axes[1]
    left_analog_hori = msg.axes[2]

    lin_vel = 1.0 * left_analog_vert
    ang_vel = 1.5 * left_analog_hori

    msg = Twist()
    msg.linear.x = lin_vel
    msg.angular.z = ang_vel

    mcu_cmd_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bentobot_mcu_bridge')

    mcu_cmd_pub = rospy.Publisher('mcu_cmd', Twist, queue_size=10)
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

    rospy.spin()