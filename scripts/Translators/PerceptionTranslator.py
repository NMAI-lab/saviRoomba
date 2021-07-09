#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from std_msgs.msg import Float64
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from ca_msgs.msg import Bumper 
from threading import Semaphore 
import math 

# @author: Patrick Gavigan
# @author: Simon Yacoub

# PerceptionTranslator.py creates a node that publishes to the perceptions topic. The message that are published 
# are a string containing a complete perception for each sensor.
# "battery(...) irWall(.....) beacon(.....) bumper(.....)"

batteryPerception = ""
#irPerception = ""
#beaconPerception = ""
bumperPerception = ""
odomPositionPerception = ""
odomOrientationPerception = ""
updateReady = [False,False,False,False]#,False,False]
batteryIndex = 0
bumperIndex = 1
odomPositionIndex = 2
odomOrientationIndex = 3
#irIndex = 4
#beaconIndex = 5
sem = Semaphore()



# https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

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
# def translateIR(data, args):
#     (perceptionPublisher) = args
#     ir = data.data
#     global irPerception, updateReady, irIndex, sem
#     sem.acquire()
#     irPerception = "irWall({})".format(ir)
#     updateReady[irIndex] = True
#     sem.release()
#     sendUpdate(perceptionPublisher)

# Translates sensor info from /sensor/Beacon into perceptions
# def translateBeacon(data, args):
#     (perceptionPublisher) = args
#     beacon = data.data
#     #rospy.loginfo("Raw data from beacon: " + beacon)
#     #beacon = beacon.split("$")
#     global beaconPerception, updateReady, beaconIndex, sem
#     sem.acquire()
#     beaconPerception = beacon.replace(":","")
#     #for aBeacon in beacon:
#     #    if(aBeacon != ''):
#     #        aBeacon = aBeacon.replace(":","")
#     #        beaconPerception = " beacon(" + aBeacon + ") " + beaconPerception
#     updateReady[beaconIndex] = True
#     sem.release()
#     sendUpdate(perceptionPublisher)

    # Translates sensor info from /sensor/Bumper into perceptions
def translateBumper(data, args):
    (perceptionPublisher) = args
    create1 = (data.is_left_pressed, data.is_right_pressed)
    create2 = (data.is_light_left, data.is_light_front_left, data.is_light_center_left, data.is_light_center_right, data.is_light_front_right, data.is_light_right)
    if (True in create1) or (True in create2):
        message = "bumper(pressed)"
    else:
        message = "bumper(unpressed)"

    global bumperPerception, updateReady, bumperIndex, sem
    sem.acquire()
    bumperPerception = message
    updateReady[bumperIndex] = True
    sem.release()
    sendUpdate(perceptionPublisher)
   

def translateOdometer(data, args):
    position = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    orientation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    (x, y, z, w) = orientation
    (_,_,yawRad) = euler_from_quaternion(x, y, z, w)
    yaw = math.degrees(yawRad)
    
    global odomPositionPerception, odomOrientationPerception, updateReady, odomPositionIndex, odomOrientationIndex, sem
    sem.acquire()
    odomPositionPerception = "odomPosition{}".format(position)
    odomOrientationPerception = "odomYaw({})".format(yaw)
    updateReady[odomPositionIndex] = True
    updateReady[odomOrientationIndex] = True
    sem.release()
    

def sendUpdate(publisher):
    global batteryPerception, odomPositionPerception, odomOrientationPerception, beaconPerception, bumperPerception, updateReady, sem
    sem.acquire()    
    if not False in updateReady:
        # perception = batteryPerception + " " + irPerception + " " + beaconPerception + " " + bumperPerception  + " " + odomPositionPerception + " " + odomOrientationPerception
        perception = batteryPerception + " " + bumperPerception  + " " + odomPositionPerception + " " + odomOrientationPerception
        rospy.loginfo(perception)
        publisher.publish(perception)
        for i in range(len(updateReady)):
            updateReady[i] = False
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
    #rospy.Subscriber('sensors/Infrared', String, translateIR, (perceptionPublisher))
    #rospy.Subscriber('sensors/Beacon', String, translateBeacon, (perceptionPublisher))
    rospy.Subscriber('battery/charge_ratio', Float32, translateBattery, (perceptionPublisher))
    rospy.Subscriber('bumper', Bumper, translateBumper, (perceptionPublisher))
    rospy.Subscriber('odom', Odometry, translateOdometer, (perceptionPublisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
    