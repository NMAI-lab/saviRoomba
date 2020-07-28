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
    
    searcher.getNextDirection("2", "1", "4")
    
    solutionPath = list(searcher.astar(start,finish))

    print("Solution path: " + str(solutionPath))
    
    print(str(solutionPath[0]))
    print(str(solutionPath[1]))
    print(str(solutionPath[2]))
   


if __name__ == '__main__':
    #try:
    rosMain()
#    except rospy.ROSInterruptException:
#        pass
        