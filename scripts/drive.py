#!/usr/bin/env python

import rospy
from geometry_msgs.msgs import Twist

def perform_drive(line):

    line = str(line)

    print("line was: " + line)

    driver = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    vel = Twist(Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    if line == "center":
        driver.publish(vel)
    else:
        vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        driver.publish(vel)


def getline():
    rospy.init_node('getline', anonymous=True)
    rospy.Subscriber('linepath', String, perform_drive)

    rospy.spin()


if __name__ == '__main__':
    try:
        getline()
    except rospy.ROSInterruptException:
        pass