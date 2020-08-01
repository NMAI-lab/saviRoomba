#!/usr/bin/env python

# Main file for the navigator

# @author: Patrick Gavigan

#import rospy
from RouteSearcher import RouteSearcher

def rosMain():
    print("Hello")
    
    start = "1"
    finish = "4"
    
    # Setup the searcher
    searcher = RouteSearcher()
    
    searcher.getDestinationBearing("3", "5", "2")
    
    solutionPath = list(searcher.astar(start,finish))

    print("Solution path: " + str(solutionPath))
    
    print(str(solutionPath[0]))
    print(str(solutionPath[1]))
    print(str(solutionPath[2]))

def unitTest():
    # Setup the searcher
    searcher = RouteSearcher()
    
    turn = searcher.getNextDirection("3", "4", "2")
    print("Turn: " + turn)
       


if __name__ == '__main__':
    #try:
    unitTest()
#    except rospy.ROSInterruptException:
#        pass
        