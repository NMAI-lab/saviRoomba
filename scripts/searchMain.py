#!/usr/bin/env python

# Main file for the search implementation.
# Uses https://github.com/jrialland/python-astar

# @author: Patrick Gavigan

import rospy
#from savi_a_star.msg import navigationSolution, navigationProblem

from RouteSearcher import RouteSearcher


def performSearch(data, args):
    (publisher, searcher) = args

    #solution = navigationSolution()
    solutionPath = list(searcher.astar(data.start,data.finish))
    
    #rospy.loginfo(solution)
    #publisher.publish(solution)    


def rosMain():
    
    # Setup the searcher
    searcher = RouteSearcher()
    
    # Init the node
    rospy.init_node('searcher', anonymous=True)

    # Setup the publisher for the result
    publisher = rospy.Publisher('mapSearch/solutions', navigationSolution, queue_size=10)
    
    # Listen for messages
    rospy.Subscriber('mapSearch/problems', navigationProblem, performSearch, (publisher, searcher))
    
    # Launch the tester
    #tester()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
def tester():
    publisher = rospy.Publisher('mapSearch/problems', navigationProblem, queue_size=10)
    
    # Set the sensor publishing frequency
    rate = rospy.Rate(1) # 1hz
    
    request = navigationProblem()
    request.start = 'E'
    request.finish = 'B'

    # Publishing loop
    while not rospy.is_shutdown(): 
        publisher.publish(request)  
        rate.sleep()
        

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass