#!/usr/bin/env python

# @author: Patrick Gavigan
# @author: Simon Yacoub

# SUBSCRIBER:   none
# PUBLISHER:    String object to 'beacons' node

import rospy
from bluepy.btle import Scanner
from std_msgs.msg import String
from reader import BeaconReader
import statistics


# Polls for bluetooth devices, returns signal strength in a dict, indexed by
# MAC address.
# beaconParameters: Parameters of the beacons we care about
# timeout: How long to wait when polling for beacons
def pollBeacons(beaconParameters, timeout):
    beaconScan = Scanner().scan(timeout)
    # beaconScan = Scanner().scan(1)
    foundBeacons = dict()
    for beacon in beaconScan:
        if beacon.addr in beaconParameters.keys():
            foundBeacons[beacon.addr] = beacon.rssi
    return foundBeacons

# Convert the signal strength data to a distance (hopefully in meters)
def calculateDistance(beaconData,beaconParameters):
    macAddresses = beaconData.keys()
    #if len(macAddresses) == 0:
    #    return None

    distances = dict()
    for mac in macAddresses:
        environmentVariable = beaconParameters[mac][0]
        measuredValue = beaconParameters[mac][1]
        distances[mac] = pow(10,(measuredValue - beaconData[mac])/(10*environmentVariable))
    return distances
    
def manageBeaconQueue(beaconHistory,newData,queueLength=10):
    beaconNames = beaconHistory.keys()
    newBeaconDataNames = newData.keys()

    for currentBeacon in beaconNames:
        currentQueue = beaconHistory[currentBeacon]

        if currentBeacon in newBeaconDataNames:
            currentQueue.append(newData[currentBeacon])
        
        currentQueue = removeOutliers(currentQueue)
        
        # If the queue is too long, pop the oldest element
        if len(currentQueue) > queueLength:
            currentQueue.pop(0)
        
    return beaconHistory

    
# Remove outliers from range list
def removeOutliers(rangeList,queueLength=10):
    if len(rangeList) > queueLength:
        dataMean = statistics.mean(rangeList)
        dataStd = statistics.stdev(rangeList)
        maxValue = dataMean + (3 * dataStd)
        minValue = dataMean - (3 * dataStd)
    
        for i in range(len(rangeList)):
            if (rangeList[i] < minValue) or (rangeList[i] > maxValue):
                rangeList[i] = -1       # Flag the outlier
            
    rangeList = filter(lambda a: a != -1, rangeList) 
    return rangeList

    
# Get averages for the distances
def calculateAverages(beaconData):
    beaconNames = beaconData.keys()
    beaconAverages = dict()
    for beaconName in beaconNames:
        if len(beaconData[beaconName]) > 1:
            beaconAverages[beaconName] = statistics.mean(beaconData[beaconName])
        elif len(beaconData[beaconName]) == 1:
            beaconAverages[beaconName] = beaconData[beaconName][0]
    return beaconAverages


def publishData(pub, data):
    message = ""
    beaconNames = data.keys()
    for beaconName in beaconNames:
        message = message + "beacon(" + str(beaconName) + "," + str(data[beaconName]) + ")"
    
    rospy.loginfo(message)
    pub.publish(message)
        

def runBeacons(pub,rate,beaconParameters,beaconHistory):
    period = 1.0/rate
    beaconData = pollBeacons(beaconParameters, period)
    distances = calculateDistance(beaconData,beaconParameters)

    beaconHistory = manageBeaconQueue(beaconHistory,distances)

    # Calculate the average for each queue
    beaconAverage = calculateAverages(beaconHistory)
    
    # Publish averages
    publishData(pub,beaconAverage)
    

def rosMain():
    
    # Set up the ROS node
    pub = rospy.Publisher('/sensors/Beacon', String, queue_size=5)
    rospy.init_node('bluetoothBeacons', anonymous=True)
    frequency = 5
    rate = rospy.Rate(frequency)
    
    # Get the beacon parameters from the file
    beaconReader = BeaconReader()
    beaconParameters = beaconReader.read_beacons()
    
    # Dict for the beacon history data queues
    beaconHistory = dict()
    beaconNames = beaconParameters.keys()
    for beacon in beaconNames:
        beaconHistory[beacon] = list()
    
    # Run the node
    while not rospy.is_shutdown():
        runBeacons(pub,frequency,beaconParameters,beaconHistory)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass

