#!/usr/bin/env python

# Main file for the navigator

# @author: Patrick Gavigan

import rospy
from RouteSearcher import RouteSearcher
from std_msgs.msg import String

# def sendDirection(data, args):
#     (publisher, searcher) = args

#     #solution = navigationSolution()
#     solution = list(searcher.astar(data.start,data.finish))
    
#     rospy.loginfo(solution)
#     publisher.publish(solution)    

# def updateDestination(data, args):
#     (searcher) = args
    
#     # Check if this is a setDestiantion() message
#     if "setDestination" in data:
        
#         searcher.setDestination(dest)

# def rosMain():
    
#     # Setup the searcher
#     searcher = RouteSearcher()
    
#     # Init the node
#     rospy.init_node('navigator', anonymous=True)

#     # Setup the publisher for the result
#     publisher = rospy.Publisher('perceptions', String, queue_size=10)
    
#     # Listen for post point messages on the perceptions topic
#     rospy.Subscriber('perceptions', String, performSearch, (publisher, searcher))
    
#     # Launch the tester
#     #tester()

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

def unitTest():
    # Setup the searcher
    searcher = RouteSearcher()
    searcher.setDestination("7")
    
    searcher.setDestination("2")
    turn = searcher.getNextDirection("4", "3")
    print("Turn should be left: " + turn)
    
    searcher.setDestination("2")
    turn = searcher.getNextDirection("5", "3")
    print("Turn should be right: " + turn)
    
    searcher.setDestination("5")
    turn = searcher.getNextDirection("4", "3")
    print("Turn should be forward: " + turn)
    
    searcher.setDestination("4")
    turn = searcher.getNextDirection("5", "3")
    print("Turn should be forward: " + turn)
    
    searcher.setDestination("3")
    turn = searcher.getNextDirection("1", "2")
    print("Turn should be left: " + turn)


if __name__ == '__main__':
    #try:
    unitTest()
#    except rospy.ROSInterruptException:
#        pass
        