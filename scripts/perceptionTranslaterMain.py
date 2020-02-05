#!/usr/bin/env python

#Created on Tue Feb  4 14:52:12 2020
#@author: patrickgavigan

import rospy
from std_msgs.msg import String
from ca_msgs.msg import Bumper


# Receive the bumper message and call the publisher
def receiveSensorData(sensorData):

    # Get a string for leftPressedString
    if (sensorData.is_left_pressed):
        leftPressedString = "true"
    else:
        leftPressedString = "false"
        
    # Get a string for rightPressedString
    if (sensorData.is_right_pressed):
        rightPressedString = "true"
    else:
        rightPressedString = "false"
    
    # Build the perception message and publish
    perceptionMessage = "bumper(" + leftPressedString + "," + rightPressedString + ")"
    publishAsPerception(perceptionMessage)


# Publish the bumper perception
def publishAsPerception(message):
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.loginfo(message)
    pub.publish(message)


# Main method: subscribe to relevant sensor data type and receive the data
def perceptionTranslatorMain():

    # Setup the node
    rospy.init_node('perceptionTranslater', anonymous=True)
    
    # Listen to bumper messages
    rospy.Subscriber('bumper', Bumper, receiveSensorData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    try:
        perceptionTranslatorMain()
    except rospy.ROSInterruptException:
        pass
