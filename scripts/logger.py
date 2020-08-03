#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime
import csv

def log_csv(topic, data, timestamp, m):
    
    with open('log.csv', mode=m) as node_log:
        node_logger = csv.writer(node_log, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        node_logger.writerow([topic, data, timestamp])
    
def log_action(action):
    data = str(action.data)
    dateTimeObj = str(datetime.now())
    log_csv('actions', data, dateTimeObj, 'a+')

def log_perception(perception):
    data = str(perception.data)
    dateTimeObj = str(datetime.now())
    log_csv('perceptions', data, dateTimeObj, 'a+')
    


def log_nodes():
    rospy.init_node('nodeLogger', anonymous=True)
    rospy.Subscriber('actions', String, log_action)
    rospy.Subscriber('perceptions', String, log_perception)

    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        log_csv('Topic', 'Data', 'Timestamp', 'w')
        log_nodes()
    except rospy.ROSInterruptException:
        pass
    
    