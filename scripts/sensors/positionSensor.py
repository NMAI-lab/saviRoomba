#!/usr/bin/env python

# @author: Patrick Gavigan

# SUBSCRIBER:   String object from 'beacons' node
# PUBLISHER:    String object with position data

#import rospy
#from std_msgs.msg import String
from reader import BeaconReader
import statistics
from threading import Semaphore 
import os
import math

messageReceived = "beacon(e2:77:fc:f9:04:93,0.707) beacon(d0:6a:d2:02:42:eb,0.707) beacon(ee:16:86:9a:c2:a8,0.707) beacon(e4:87:91:3d:1e:d7,0.707)"
sem = Semaphore()


# Load the beacon parameters, return in a dict
def loadBeaconParameters():
    script_dir = os.path.dirname(__file__)
    filename = 'beacons'
    fullpath = os.path.join(script_dir,filename)
    beacons = ''
    with open(fullpath,'r') as f:
        beacons = f.read()
        f.close()
    beaconParameters = dict()
    lines = beacons.splitlines()
    for l in lines:
        csv = l.split(',')
        beaconParameters[csv[0]] = (float(csv[1]),float(csv[2]),csv[3],float(csv[4]),float(csv[5]))
    return beaconParameters


# Receive a beacon message and store it in a global variable. Thread safe using a semaphore
def receiveBeacon(data):
    global messageReceived, sem
    sem.acquire()
    messageReceived = data.data
    sem.release()
    
def getBeaconString():
    global messageReceived, sem
    message = ""
    
    # Put in the loop to make sure it doesn't return until there is data
    while message == "":
        sem.acquire()
        message = messageReceived
        sem.release()
    return message


# Extract the beacon message data and put it in a dict
def processBeaconString(beaconString):
    beaconData = dict()
    
    beaconString = beaconString.replace("beacon","")
    beaconString = beaconString.replace(" ","")
    splitBeacon = beaconString.split(")(")
    
    for item in splitBeacon:
        components = item.split(",")
        mac = components[0]
        distance = components[1]
        
        mac = mac.replace("(","")
        mac = mac.replace(")","")
        distance = distance.replace("(","")
        distance = distance.replace(")","")
        
        beaconData[mac] = float(distance)
    
    return(beaconData)
    

def getPositionRangeTuples(beaconData,beaconParameters):
    tupleList = list()
    beaconList = beaconData.keys()
    
    # Index locations for the x and y coordinates
    xIndex = 3
    yIndex = 4
    
    for beacon in beaconList:
        x = beaconParameters[beacon][xIndex]
        y = beaconParameters[beacon][yIndex]
        measuredRange = beaconData[beacon]
        tupleList.append((x,y,measuredRange))
    return tupleList
    
# Calculate intercpt between two circles a and b.
# The parameters contain tuples of the form (x,y,r)
# Returns two intercept coordinates
# https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
def calculateIntercepts(a,b):
    (x1,y1,r1) = a
    (x2,y2,r2) = b
    
    try:
        # Distance between the centers of the circles
        R = calculatePointDistance((x1,y1,),(x2,y2))
    
        xFirstTerm = ((1/2) * (x1+x2)) + ((((r1**2) - (r2**2)) / (2 * (R **2))) * (x2 - x1))
        yFirstTerm = ((1/2) * (y1+y2)) + ((((r1**2) - (r2**2)) / (2 * (R **2))) * (y2 - y1))
    
        secondTermRoot = math.sqrt((2 * (((r1**2) + (r2**2))/(R**2))) - ((((r1**2) - (r2**2)) ** 2) / (R**4)) - 1)
    
        xSecondTerm = (1/2) * secondTermRoot * (y2-y1)
        ySecondTerm = (1/2) * secondTermRoot * (x2-x1)
    
        xpos = xFirstTerm + xSecondTerm
        ypos = yFirstTerm + ySecondTerm
        xneg = xFirstTerm - xSecondTerm
        yneg = yFirstTerm - ySecondTerm
    
        pos = (xpos,ypos)
        neg = (xneg,yneg)
    
        return (pos,neg)
    except ValueError:
        return None


def calculatePointDistance(a,b):
    (x1,y1) = a
    (x2,y2) = b
    return math.sqrt(((x2-x1) ** 2) + ((y2-y1) ** 2))

def averagePointLocation(coordList):
    xList = list()
    yList = list()
    for coord in coordList:
        xList.append(coord[0])
        yList.append(coord[1])
    xMean = statistics.mean(xList)
    yMean = statistics.mean(yList)
    return (xMean,yMean)

def standardDeviationPointLocation(coordList):
    xList = list()
    yList = list()
    for coord in coordList:
        xList.append(coord[0])
        yList.append(coord[1])
    xStdev = statistics.stdev(xList)
    yStdev = statistics.stdev(yList)
    return (xStdev,yStdev)

def groupResults(results):
    a = list()
    b = list()
    
    for result in results:
        if len(a) == 0:
            a.append(result[0])
            b.append(result[1])
        else:
            # 4 options:
            # result[0] can be closer to a or b
            # or result[1] can be closer to a or b
            aMean = averagePointLocation(a)
            bMean = averagePointLocation(b)
            
            rangeList = list()
            aRange0Index = 0
            rangeList.append(calculatePointDistance(result[0],aMean))
            
            aRange1Index = 1
            rangeList.append(calculatePointDistance(result[1],aMean))
            
            bRange0Index = 2
            rangeList.append(calculatePointDistance(result[0],bMean))
            
            #bRange1Index = 3
            rangeList.append(calculatePointDistance(result[1],bMean))
            
            minRangeIndex = rangeList.index(min(rangeList))
            
            if minRangeIndex == aRange0Index:
                a.append(result[0])
                b.append(result[1])
            elif minRangeIndex == aRange1Index:
                a.append(result[1])
                b.append(result[0])
            elif minRangeIndex == bRange0Index:
                b.append(result[0])
                a.append(result[1])
            else:
                b.append(result[1])
                a.append(result[0])
                
    return(a,b)
                

def calculatePosition(positionRangeList):
    
    positionEstimates = list()
    
    for i in range(len(positionRangeList)):
        for j in range(i+1,len(positionRangeList)):
            a = positionRangeList[i]
            b = positionRangeList[j]
            result = calculateIntercepts(a, b)
            if result != None:
                positionEstimates.append(result)

    (a,b) = groupResults(positionEstimates)
    aStdev = standardDeviationPointLocation(a)
    bStdev = standardDeviationPointLocation(b)
    
    aDevOneNum = aStdev[0] + aStdev[1]
    bDevOneNum = bStdev[0] + bStdev[1]
    
    if aDevOneNum < bDevOneNum:
        return averagePointLocation(a)
    else:
        return averagePointLocation(b)                



def runPositionSensor():
    
    # Get the beacon parameters from the file
    beaconParameters = loadBeaconParameters()
    
    # Get latest beacon data, turn it into a dict
    beaconString = getBeaconString()
    beaconData = processBeaconString(beaconString)

    # Get the position estimate    
    positionRangeList = getPositionRangeTuples(beaconData,beaconParameters)
    position = calculatePosition(positionRangeList)
    return position
    
def rosMain():
    
    rospy.init_node('position', anonymous=True)
    
    # Set up the subscriber
    rospy.Subscriber('sensors/Beacon', String, receiveBeacon)
    
    
    # Set up the ROS node
    pub = rospy.Publisher('/sensors/Position', String, queue_size=5)
    
    frequency = 60
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
    

def test():
    #beaconMessage = "beacon(e2:77:fc:f9:04:93,1.5) beacon(d0:6a:d2:02:42:eb,1.5) beacon(ee:16:86:9a:c2:a8,1.5) beacon(e4:87:91:3d:1e:d7,1.5)"
    runPositionSensor()


if __name__ == '__main__':
    test()
    
#    try:
#        rosMain()
#    except rospy.ROSInterruptException:
#        pass

