#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def perform_drive(line):

    line = str(line.data)

    print("line was: " + line)

    driver = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()

    if line == "center":
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0
        driver.publish(vel_msg)
    elif line == "left":
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.1
        driver.publish(vel_msg)
    elif line == "right":
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.1
        driver.publish(vel_msg)
    else:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        driver.publish(vel_msg)


def getline():
    rospy.init_node('getline', anonymous=True)
    rospy.Subscriber('linepath', String, perform_drive)

    rospy.spin()


if __name__ == '__main__':
    try:
        getline()
    except rospy.ROSInterruptException:
        pass

