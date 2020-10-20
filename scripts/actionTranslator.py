#!/usr/bin/env python

import rospy
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from driverLineSensor import getLine

lastTurn = "left"

# Decode and execute the action
def decodeAction(data, args):
    
    # Get the parameters
    action = str(data.data)
    (drivePublisher, dockPublisher, undockPublisher, destinationPublisher) = args
    rospy.loginfo("Action: " + action)
    
    # Handle the docking station cases
    if action == "station(dock)":
        dockPublisher.publish()
    elif action == "station(undock)":
        undockPublisher.publish()   # Publish to the undock topic
        
        for i in range(10):
            drive("undock")
        
        turn(drivePublisher,"left") # Turn the robot around
        
    # Extract the action parameter between the brackets
    parameter = re.search('\((.*)\)', action).group(1)
    
    # Deal with drive action
    if re.search("drive", action):
        drive(drivePublisher,parameter)
        
    # Deal with turn message (similar to drive action but continues until the 
    # line sensor detects "c")
    elif re.search("turn", action):
        turn(drivePublisher,parameter)
    
    # Deal with passing setDestination action to the appropriate topic
    elif re.search("setDestination", action):
        destinationPublisher.publish(parameter)
    
    # Deal with invalid action
    else:
        rospy.loginfo("Invalid action ignored")

# Turn command, repeated drive commands until the lince sensor detects c again
def turn(publisher, parameter):
    
    # Get the turn started
    drive(publisher,parameter)
    
    # Keep turning until the line is centered again
    while getLine()[0] != "c":
            drive(publisher,parameter, False)
            
    # Stop, once the line is centered again
    drive(publisher, "stop")


# Drive command for the robot
def drive(publisher, parameter, driveParam = True):
    message = getTwistMesg(parameter, driveParam)
    publisher.publish(message)
        
# Get the message to send to the robot in order to drive it
def getTwistMesg(parameter, drive):
    message = Twist()
    
    global lastTurn
    
    if parameter == "forward":
        message.linear.x = 0.05
        message.angular.z = 0
    elif parameter == "left":
        lastTurn = parameter
        message.linear.x = 0.01
        message.angular.z = 0.05
    elif parameter == "right":
        lastTurn = parameter
        message.linear.x = 0.01
        message.angular.z = -0.05
    elif parameter == "stop":
        #message.linear.x = 0.1
        #message.angular.z = 0.1
        message.linear.x = 0
        message.angular.z = 0
    elif parameter == "undock":
        message.linear.x = -0.01
        message.linear.z = 0
    else:                           # Line lost or across
        if lastTurn == "right":
            message.linear.x = 0.005
            message.angular.z = -0.05
        else:
            message.linear.x = 0.005
            message.angular.z = 0.05
        
    #if (drive == False) and (parameter != "forward"):
    #    message.linear.x = 0
    
    return message

# Main execution
def rosMain():
    drivePublisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    dockPublisher = rospy.Publisher('dock', Empty, queue_size=10)
    undockPublisher = rospy.Publisher('undock', Empty, queue_size=10)
    destinationPublisher = rospy.Publisher('setDestination', String, queue_size=10)
    rospy.init_node('actionTranslator', anonymous=True)
    rospy.Subscriber('actions', String, decodeAction, (drivePublisher, dockPublisher, undockPublisher, destinationPublisher))
    rospy.spin()

# Start things up
if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
