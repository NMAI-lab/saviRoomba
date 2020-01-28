#!/usr/bin/env python
# Created on Mon Jan 27 14:38:44 2020
# @author: patrickgavigan

import rospy
from ca_msgs.msg import PlaySong

def playSong():
    pub = rospy.Publisher('playSong', PlaySong, queue_size=10)
    
    message = 0
    
    rospy.init_node('songTry', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        if message < 3:
            message = message + 1
        else:
            message = 0
            
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        playSong()
    except rospy.ROSInterruptException:
        pass
