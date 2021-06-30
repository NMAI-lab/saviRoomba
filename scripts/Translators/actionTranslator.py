#!/usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

# @author: Devon Daley 
# @author: Patrick Gavigan
# @author: Simon Yacoub

# SUBSCRIBER:   String object from 'actions' node
# PUBLISHER:    Twist object to 'cmd_vel' node
#               null object to 'dock' node

# This script is meant to take all the action decisions from our reasoner and publish them to the roomba (via cmd_vel)

# Decode and execute the action
def decodeAction(data, args):
    actionMessage = Twist() #the message

    # Get the parameters
    action = str(data.data)
    (drivePublisher, dockPublisher, undockPublisher) = args
    rospy.loginfo("Action: " + action)

    """
    Handle basic movement commands from actions topic
    Actions:
    drivexy(x,y) Drive the robot linearly with x(+ forward, - backward) and angularly with y(+ right, - left)
    drive(forward) Drive the robot forward
    drive(stop) Stops the robot
    drive(backward) Drive the robot backward
    drive(sleft) Drive the robot slightly left
    drive(sright) Drive the robot slightly right
    drive(bleft) Drives the robot back and a little left. This is used when the robot gets to close to the wall and needs to realign itself
    drive(avoidright) Drives in an arc to avoid an obstacle to the robots right.
    turn(left) Does a 45 degree turn left (stops robot first)
    turn(right) Does 45 degree turn right (stops robot first)
    station(dock) Docks the robot
    station(undock) Undocks the robot
    """
    actionMessage = getTwistMesg(action)

    if re.search("drivexy", action): #if it is a drivexy action
        parameter = re.search('\((.*)\)', action).group(1) # Extract the "x,y" between the brackets 
        parameter = parameter.split(",") #split around the comma
        x = float(parameter[0]) #convert x value into float from string
        y = float(parameter[1]) #convert y value into float from string
        #perform safety/sanity checks on the drive values
        if(abs(x) <= 0.5 and abs(y) <= 4.25):
            message = Twist()
            message.linear.x = x
            message.angular.z = y
            drivePublisher.publish(message)
     
    elif re.search("drive", action): #if it is a drive action
        if(action == "drive(forward)"): # Drive the robot forward
            actionMessage = getTwistMesg("forward")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(stop)"): # Stops the robot
            actionMessage = getTwistMesg("drive(stop)")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(backward)"): # Drive the robot backward
            actionMessage = getTwistMesg("backward")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(sleft)"): # Drive the robot backward
            actionMessage = getTwistMesg("sleft")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(sright)"): # Drive the robot backward
            actionMessage = getTwistMesg("sright")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(bleft)"): # Drive the robot backward
            actionMessage = getTwistMesg("bleft")
            drivePublisher.publish(actionMessage)
        elif(action == "drive(avoidright)"): # Drive the robot backward
            actionMessage = getTwistMesg("avoidright")
            drivePublisher.publish(actionMessage)
            
    elif re.search("turn", action): #if it is a turn action
        if(action == "turn(left)"): #Does a 45 degree turn left (stops robot first)
            actionMessage = getTwistMesg("left")
            tmp = String()
            tmp.data = "stop"
            decodeAction(tmp, args)
            drivePublisher.publish(actionMessage)
        elif(action == "turn(right)"): #Does 45 degree turn right (stops robot first)
            actionMessage = getTwistMesg("right")
            tmp = String()
            tmp.data = "drive(stop)"
            decodeAction(tmp, args)
            drivePublisher.publish(actionMessage)
        
    #If it is a station action
    elif action == "station(dock)":
        dockPublisher.publish()
    elif action == "station(undock)":
        undockPublisher.publish()   # Publish to the undock topic
    
    

    
        
"""
Get a Twist message which consists of a linear and angular component which can be negative or positive.
linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)
angular.z (+)     Rotate counter-clockwise (rad/s)
         (-)     Rotate clockwise (rad/s)
Limits:
-0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25 (4rads = 45deg)
"""
def getTwistMesg(action):
    message = Twist()
    
    if action == "forward":
        message.linear.x = 0.1
        message.angular.z = 0
    elif action == "backward":
        message.linear.x = -0.2
        message.linear.z = 0
    elif action == "left":
        message.linear.x = 0
        message.angular.z = 4
    elif action == "right":
        message.linear.x = 0
        message.angular.z = -4
    elif action == "sleft": # slight left (not sure need to test)
        message.linear.x = 0.05
        message.angular.z = 0.5
    elif action == "sright": # slight right (not sure need to test)
        message.linear.x = 0.05
        message.angular.z = -0.5
    elif action == "avoidright": #obstacle avoidance when obstacle is to the right of the object
        message.linear.x = 0.08
        message.angular.z = -0.5
    elif action == "bleft": # This move the robot back and a little left. This is used when the robot gets to close to the wall and needs to readjust itself
        message.linear.x = -0.1
        message.angular.z = 0.5
    elif action == "stop":
        message.linear.x = 0
        message.angular.z = 0
    
    return message

# Main execution
def rosMain():
    drivePublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    dockPublisher = rospy.Publisher('dock', Empty, queue_size=1)
    undockPublisher = rospy.Publisher('undock', Empty, queue_size=1)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (drivePublisher, dockPublisher, undockPublisher))

    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
