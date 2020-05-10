#!/usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

is_driving = False


def perform_action(action):

    act = str(action.data)
    print("action is: " + act)
    direction = ""
    if re.search("drive", act):
        if re.search("forward", act):
            direction = "forward"
        elif re.search("left", act):
            direction = "left"
        elif re.search("right", act):
            direction = "right"
        elif re.search("stop", act):
            direction = "stop"
        perform_drive(direction)
    if act == "dock_bot":
        docker = rospy.Publisher('dock', Empty, queue_size=10)
        docker.publish()
    elif act == "undock_bot":
        docker = rospy.Publisher('undock', Empty, queue_size=10)
        docker.publish()


def perform_drive(direction):
    print("direction is: " + direction)
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


def decode_action():
    rospy.init_node('actionDecoder', anonymous=True)
    rospy.Subscriber('actions', String, perform_action)

    rospy.spin()


if __name__ == '__main__':
    try:
        decode_action()
    except rospy.ROSInterruptException:
        pass
