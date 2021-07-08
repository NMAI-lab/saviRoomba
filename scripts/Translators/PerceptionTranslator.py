#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from threading import Semaphore   

# @author: Patrick Gavigan
# @author: Simon Yacoub

# PerceptionTranslator.py creates a node that publishes to the perceptions topic. The message that are published 
# are a string containing a complete perception for each sensor.
# "battery(...) irWall(.....) beacon(.....) bumper(.....)"

batteryPerception = ""
irPerception = ""
beaconPerception = ""
bumperPerception = ""
updateReady = [False,False,False,False]
batteryIndex = 0
irIndex = 1
beaconIndex = 2
bumperIndex = 3
sem = Semaphore()

# Translates sensor info from battery/charge_ratio into perceptions
def translateBattery(data, args):
    (perceptionPublisher) = args
    battery = data.data   
    global batteryPerception, updateReady, batteryIndex, sem
    sem.acquire()
    batteryPerception = "battery({})".format(battery)
    updateReady[batteryIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)

# Translates sensor info from /sensor/Infrared into perceptions
def translateIR(data, args):
    (perceptionPublisher) = args
    ir = data.data
    global irPerception, updateReady, irIndex, sem
    sem.acquire()
    irPerception = "irWall({})".format(ir)
    updateReady[irIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)

# Translates sensor info from /sensor/Beacon into perceptions
def translateBeacon(data, args):
    (perceptionPublisher) = args
    beacon = data.data
    #rospy.loginfo("Raw data from beacon: " + beacon)
    #beacon = beacon.split("$")
    global beaconPerception, updateReady, beaconIndex, sem
    sem.acquire()
    beaconPerception = beacon.replace(":","")
    #for aBeacon in beacon:
    #    if(aBeacon != ''):
    #        aBeacon = aBeacon.replace(":","")
    #        beaconPerception = " beacon(" + aBeacon + ") " + beaconPerception
    updateReady[beaconIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)

    # Translates sensor info from /sensor/Bumper into perceptions
def translateBumper(data, args):
    (perceptionPublisher) = args
    bumper = data.data
    global bumperPerception, updateReady, bumperIndex, sem
    sem.acquire()
    bumperPerception = "bumper({})".format(bumper)
    updateReady[bumperIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)
    #if "pressed" in bumperPerception:
    #    sendAsyncUpdate(perceptionPublisher, bumperPerception)
    

def translateOdometer(data, args):
    position = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    orientation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    print(position)
    print(orientation)
    

def sendUpdate(publisher):
    global batteryPerception, irPerception, beaconPerception, bumperPerception, updateReady, sem
    sem.acquire()    
    if not False in updateReady:
        perception = batteryPerception + " " + irPerception + " " + beaconPerception + " " + bumperPerception  
        rospy.loginfo(perception)
        publisher.publish(perception)
        updateReady = [False,False,False,False]
    sem.release()

def sendAsyncUpdate(publisher, message):
    global sem
    sem.acquire()    
    #perception = batteryPerception + " " + irPerception + " " + beaconPerception + " " + bumperPerception  
    rospy.loginfo(message)
    publisher.publish(message)
    sem.release()

# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('PerceptionTranslator', anonymous=True)
    perceptionPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('sensors/Infrared', String, translateIR, (perceptionPublisher))
    rospy.Subscriber('sensors/Beacon', String, translateBeacon, (perceptionPublisher))
    rospy.Subscriber('battery/charge_ratio', Float32, translateBattery, (perceptionPublisher))
    rospy.Subscriber('sensors/Bumper', String, translateBumper, (perceptionPublisher))
    rospy.Subscriber('odom', Odometry, translateOdometer, (perceptionPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
    