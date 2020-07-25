#!/usr/bin/env python

# Definitions for the search functionality

# @author: Patrick Gavigan

# Uses https://github.com/jrialland/python-astar
# Install using: pip install astar
# https://pypi.org/project/astar/

from astar import AStar
import math
import json
import numpy as np

class RouteSearcher(AStar):

    # Define the map
    def __init__(self):
            
        # Load node graph
        f = open('nodeGraph.json')
        self.nodeGraph = json.load(f)
        
        # Load node locations
        f = open('nodeLocations.json')
        self.nodeLocations = json.load(f)

     # Compute the distance between two (x,y) tuples
    def heuristic_cost_estimate(self, n1, n2):      
        (x1,y1) = self.nodeLocations[n1]
        (x2,y2) = self.nodeLocations[n2]
        return math.hypot(x2 - x1, y2 - y1)

    # Return the distance between two neighbouring nodes  
    def distance_between(self, n1, n2):
        return [item for item in self.nodeGraph[n1] if item[0] == n2][0][1]

    # Return list of neighbours
    def neighbors(self, node):
        neighbourNodes = self.nodeGraph[node]
        neighbourNames = [a_tuple[0] for a_tuple in neighbourNodes]
        return neighbourNames
    
    # Returns a direction for where to go to continue on the journey
    def getNextDirection(self, current, previous, dest):
        (xCurrent, yCurrent) = self.nodeLocations[current]
        (xPrevious, yPrevious) = self.nodeLocations[previous]
        (xDest, yDest) = self.nodeLocations[dest]
        
        current = np.array([xCurrent, yCurrent])
        previous = np.array([xPrevious, yPrevious])
        destination = np.array([xDest, yDest])
        
        currentDirection = current - previous
        desiredDirection = destination - current
 
        print(currentDirection)
        print(desiredDirection)
        
#        distance = [xCurrent - xPrevious, yCurrent - yPrevious]
#        norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
#        currentDirection = [distance[0] / norm, distance[1] / norm]
        
        
        
#        desiredDirection = 1
        
#        return "DestLeft"
    
    #def getDirectionRange(A, B):
        