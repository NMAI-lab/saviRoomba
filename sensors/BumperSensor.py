#!/usr/bin/env python

# @author: Simon Yacoub

# SUBSCRIBER:   Bumper object from 'bumper' node
# PUBLISHER:    String object to 'perceptions' node

import rospy
from ca_msgs.msg import Bumper 
from std_msgs.msg import String

# This script is intended to detect the bumper sensor and inform the reasoner via 'perceptions'

x = 0

# Translate the line sensor data into a perception and publish
def bumpAndDoSomething(data, args):

    # Extract the publisher and the message data
    (publisher) = args
    is_left_pressed = data.is_left_pressed
    is_right_pressed = data.is_right_pressed 
    
    # Bumper light sensors (Create 2 only) in order from left to right
    # Value = true if an obstacle detected
    is_light_left = data.is_light_left
    is_light_front_left = data.is_light_front_left
    is_light_center_left = data.is_light_center_left
    is_light_center_right = data.is_light_center_right
    is_light_front_right = data.is_light_front_right
    is_light_right = data.is_light_right

    message = String()
        
    if(is_left_pressed or is_right_pressed):message.data = "pressed"
    else: message.data = "unpressed"

    # Publish the perception
    publisher.publish(message)


# Initialize the node, setup the publisher and subscriber
def rosMain():
    rospy.init_node('BumperSensor', anonymous=True)
    publisher = rospy.Publisher('/sensors/bumper', String, queue_size=10)
    rospy.Subscriber('bumper', Bumper, bumpAndDoSomething, (publisher))
    rospy.spin()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass