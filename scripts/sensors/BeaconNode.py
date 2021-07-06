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


def pollBeacons(timeout):
    beaconList = Scanner().scan(timeout)
    
    # Return beacon data
    

def runBeacons(pub,rate):
    print("hello beacons")
    
    beaconParameters = BeaconReader()
    period = 1/rate
    pollBeacons(period)
    
    print("goodbye beacons")
    
    


def rosMain():
    
    pub = rospy.Publisher('/sensors/Beacon', String, queue_size=5)
    rospy.init_node('bluetoothBeacons', anonymous=True)
    rate = 60
    rate = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        runBeacons(pub,rate)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass

