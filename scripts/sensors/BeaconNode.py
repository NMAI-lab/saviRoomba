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


# Polls for bluetooth devices, returns signal strength in a dict, indexed by
# MAC address.
# beaconParameters: Parameters of the beacons we care about
# timeout: How long to wait when polling for beacons
def pollBeacons(beaconParameters, timeout):
    beaconScan = Scanner().scan(timeout)
    foundBeacons = dict()
    for beacon in beaconScan:
        if beacon.addr in beaconParameters.read_beacons().keys():
            foundBeacons[beacon.addr] = beacon.rssi
    return foundBeacons
    

def removeOutliers(rangeList):
    # Remove outliers from range list
    print("outlier function")
    
def publishData(pub, data):
    
    # Publish the data to ROS
    print("publish data")
        

def runBeacons(pub,rate):
    #print("hello beacons")
    
    beaconParameters = BeaconReader()
    period = 1.0/rate
    foundBeacons = pollBeacons(beaconParameters, period)
    print(foundBeacons)
    
    
    #print("goodbye beacons")
    
    


def rosMain():
    
    pub = rospy.Publisher('/sensors/Beacon', String, queue_size=5)
    rospy.init_node('bluetoothBeacons', anonymous=True)
    frequency = 60
    rate = rospy.Rate(frequency)
    
    while not rospy.is_shutdown():
        runBeacons(pub,frequency)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass

