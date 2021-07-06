#!/usr/bin/env python

# @author: Patrick Gavigan
# @author: Simon Yacoub

# SUBSCRIBER:   none
# PUBLISHER:    String object to 'beacons' node

import rospy
import time
from bluepy.btle import Scanner
from std_msgs.msg import String
from reader import BeaconReader


def runBeacons(pub):
    print("hello beacons")


def rosMain():
    
    pub = rospy.Publisher('/sensors/Beacon', String, queue_size=5)
    rospy.init_node('bluetoothBeacons', anonymous=True)
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
        runBeacons(pub)
        rate.sleep()
    


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass

