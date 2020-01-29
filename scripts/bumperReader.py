#!/usr/bin/env python
# Created on Mon Jan 27 14:38:44 2020
# @author: patrickgavigan

import rospy
from ca_msgs.msg import Bumper

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print(data)
    print("Bumper data is_light_center_right: " + str(data.is_light_center_right))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('actionListener', anonymous=True)
    rospy.init_node('bumperListener', anonymous=True)
    
    rospy.Subscriber('bumper', Bumper, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
