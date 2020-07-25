#!/usr/bin/env python

# Main file for the navigator

# @author: Patrick Gavigan

#import rospy
from RouteSearcher import RouteSearcher

def rosMain():
    print("Hello")
    
    start = 'E'
    finish = 'B'
    
    # Setup the searcher
    searcher = RouteSearcher()
    
    searcher.getNextDirection('C', 'E', 'B')
    
    solutionPath = list(searcher.astar(start,finish))

    print(str(solutionPath))

if __name__ == '__main__':
    #try:
    rosMain()
#    except rospy.ROSInterruptException:
#        pass
        