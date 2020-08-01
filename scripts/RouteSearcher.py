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
    
    # Returns the unit vector of the vector.
    def unitVector(self, vector):
        return vector / np.linalg.norm(vector)
    
    # Returns the angle in radians between vectors 'v1' and 'v2'::
    # >>> angle_between((1, 0, 0), (0, 1, 0))
    # 1.5707963267948966
    # >>> angle_between((1, 0, 0), (1, 0, 0))
    # 0.0
    # >>> angle_between((1, 0, 0), (-1, 0, 0))
    # 3.141592653589793
    def anglBetweenRad(self, v1, v2):
        v1_u = self.unitVector(v1)
        v2_u = self.unitVector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
       
        
    def angleBetweenDeg(self, v1, v2):
        return self.anglBetweenRad(v1, v2) * 180 / np.pi
    
    # Returns a direction for where to go to continue on the journey
    def getDestinationBearing(self, current, previous, dest):
        (xCurrent, yCurrent) = self.nodeLocations[current]
        (xPrevious, yPrevious) = self.nodeLocations[previous]
        (xDest, yDest) = self.nodeLocations[dest]
        
        current = np.array([xCurrent, yCurrent])
        previous = np.array([xPrevious, yPrevious])
        destination = np.array([xDest, yDest])
        
        currentDirection = current - previous
        desiredDirection = destination - current
 
        #print("current direction: " + str(currentDirection))
        #print("Desired direction: " + str(desiredDirection))
                
        turnAngle = self.angleBetweenDeg(currentDirection, desiredDirection)
        #print("Turn angle: " + str(turnAngle))
        return turnAngle
    
    # get the bearing angle for the next step in the plan
    def getNextTurnAngle(self, solutionPath, previous):
        
        # Case where we are at the destination, nothing really needs to be done
        if len(solutionPath) == 1:
            return 0
        
        # len is greater than 1. We only really case about where to go next
        else:
            nextPoint = solutionPath[1]
            current = solutionPath[0]
            return self.getDestinationBearing(current, previous, nextPoint)        
    
    
    def getNextDirection(self, current, previous, dest):
        # Deal with special case where we are already at the destination
        if current == dest:
            return "next(arrived)"
    
        # Deal with special case where there is no previous location
        if previous == -1:
            return "next(forward)"  # Try straight ahead, no other way to know what direction you are facing
                                    # TODO: Investigate if there are alternatives to this (perhaps pick something from the graph at random)
        
        solutionPath = list(self.astar(current,dest))
        bearing = self.getNextTurnAngle(solutionPath, previous)
        
        # Deal with only positive numbers        
        while bearing < 0:
            bearing = bearing + 360
            
        # Deal with case where the bearing is too big
        while bearing > 360:
            bearing = bearing - 360
            
        # Case where it is more or less straight ahead
        if ((bearing >= (360 - 45)) or (bearing < 45)):
            return "next(forward)"
        
        # Case where it is more or less the the right
        elif (bearing >= (90-45)) and (bearing < (90+45)):
            return "next(right)"
        
        # Case where it is more or less behind
        elif (bearing >= (180-45)) and (bearing < (180+45)):
            return "next(behind)"
        
        # Case where it is more or less to the left (all that is left)
        else:
            return "next(left)"
        