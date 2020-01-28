#!/usr/bin/env python
# Created on Mon Jan 27 14:38:44 2020
# @author: patrickgavigan

import rospy
#from ca_msgs.msg import PlaySong
#from std_msgs.msg import String
from std_msgs.msg import Bool

def playSong():
    #pub = rospy.Publisher('playSong', PlaySong, queue_size=10)
    checkLedPub = rospy.Publisher('check_led', Bool, queue_size=10)
    debrisLedPub = rospy.Publisher('debris_led', Bool, queue_size=10)
    dockLedPub = rospy.Publisher('dock_led', Bool, queue_size=10)
    powerLedPub = rospy.Publisher('power_led', Bool, queue_size=10)
    spotLedPub = rospy.Publisher('spot_led', Bool, queue_size=10)
    
    message = True
    
    rospy.init_node('myTry', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        if message == True:
            message = False
        else:
            message = True
        
        rospy.loginfo(message)
        checkLedPub.publish(message)
        debrisLedPub.publish(message)
        dockLedPub.publish(message)
        powerLedPub.publish(message)
        spotLedPub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        playSong()
    except rospy.ROSInterruptException:
        pass
