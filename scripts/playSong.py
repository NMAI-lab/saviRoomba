#!/usr/bin/env python
# Created on Mon Jan 27 14:38:44 2020
# @author: patrickgavigan

import rospy
from ca_msgs.msg import PlaySong
#from std_msgs.msg import String

def playSong():
    pub = rospy.Publisher('play_song', PlaySong, queue_size=10)
    rospy.init_node('songPlayer', anonymous=True)
    #rate = rospy.Rate(1) # 1hz
    #while not rospy.is_shutdown():
    message = 1
    rospy.loginfo(message)
    pub.publish(message)
    #rate.sleep()

if __name__ == '__main__':
    try:
        playSong()
    except rospy.ROSInterruptException:
        pass
