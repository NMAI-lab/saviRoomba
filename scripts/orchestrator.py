#!/usr/bin/env python

import rospy
from std_msgs.msg import String

line_data = "line(lost)"
qr_data = "postPoint()"

def callback(data):
    line_data = data
    print(data)

#def qr_callback(data):
#    qr_data = data

def orchestrator():
    pub = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.init_node('percepts', anonymous=True)
    rate = rospy.Rate(1)
    rospy.Subscriber('linepath', String, callback)
    
    while not rospy.is_shutdown():
        #rospy.Subscriber('postpoint', String, qr_callback)
        
        #rospy.loginfo("{}, {}".format(line_data, qr_data))
        #pub.publish("{}, {}".format(line_data, qr_data))
        
        rospy.loginfo("{}".format(line_data))
        pub.publish("{}".format(line_data))
        rate.sleep()
        
if __name__ == '__main__':
    try:
        orchestrator()
    except rospy.ROSInterruptException:
        pass
        