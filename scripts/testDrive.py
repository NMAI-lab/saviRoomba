#!/usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def perform_drive(perceive):

    percept = str(perceive.data)
    print("perception is: " + percept)

    if percept == "position(false,true,false)":
        direction = "forward"
    elif percept == "position(true,true,false)":
        direction = "left"
    elif percept == "position(true,false,false)":
        direction = "left"
    elif percept == "position(false,true,true)":
        direction = "right"
    elif percept == "position(false,false,true)":
        direction = "right"
    else:
        direction = "stop"

    driver = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()

    if direction == "forward":
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0
        driver.publish(vel_msg)
    elif direction == "left":
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.1
        driver.publish(vel_msg)
    elif direction == "right":
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.1
        driver.publish(vel_msg)
    else:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        driver.publish(vel_msg)


def do_drive():
    rospy.init_node('doDrive', anonymous=True)
    rospy.Subscriber('perceptions', String, perform_drive)

    rospy.spin()


if __name__ == '__main__':
    try:
        do_drive()
    except rospy.ROSInterruptException:
        pass
