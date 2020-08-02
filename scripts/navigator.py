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
        