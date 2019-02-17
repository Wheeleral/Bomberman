# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from math import sqrt
import heapq
import random

class TestCharacter(CharacterEntity):			
    pathV = 0
    directions = []
    i = 1

    def do(self, wrld):
    	pass


class Node():
    def __init__(self, parent, pos, g, h):
        self.parent = parent  # for path
        self.pos = pos  # position of the node
        self.g = g  # how far from start
        self.h = h  # manhatten distance to goal
        self.f = g + h  # sum

    # find neighbors and determine if they are open or if they are obstacle
    def getNeighbors(self, gridCells, width, end, cost):
        x = self.pos[0]
        y = self.pos[1]

        neighbors = []
        # check for valid neighbors
        neighbors.append(self._validate(x + 1, y, gridCells, width, end, cost))  
        neighbors.append(self._validate(x - 1, y, gridCells, width, end, cost))
        neighbors.append(self._validate(x, y + 1, gridCells, width, end, cost))
        neighbors.append(self._validate(x, y - 1, gridCells, width, end, cost))
        neighbors.append(self._validate(x + 1, y + 1, gridCells, width, end, cost))  
        neighbors.append(self._validate(x + 1, y - 1, gridCells, width, end, cost))
        neighbors.append(self._validate(x - 1, y + 1, gridCells, width, end, cost))
        neighbors.append(self._validate(x - 1, y - 1, gridCells, width, end, cost))
        
        # return all neighbors that aren't obstacles
        return neighbors  

    # check if wall
    def _validate(self, x, y, gridCells, width, end, cost):  
        a =True
        if(x >= 0 and x < gridCells.width() and y >= 0 and y <gridCells.height()):
            if (not gridCells.wall_at(x,y)):  # convert from x,y to grid cell number
                cellCost = 0
                if (cellCost != 0):
                    # creates new node using cost map
                    return Node(self, (x, y), self.g + cellCost, heuristic((x, y), end))  
                else:
                    # create new node using default
                    return Node(self, (x, y), self.g + 1, heuristic((x, y), end))  
            else:
           	    return None
        else:
            return None


"""get the directions of a node and the next"""
def findDir(currNode, nextNode):  
    x = currNode[0]
    y = currNode[1]
    a = nextNode[0]
    b = nextNode[1]

    if (x + 1 == a and y == b):  
        return (1, 0)
    elif (x == a and y + 1 == b):  
        return (0, 1)
    elif (x - 1 == a and y == b):  
        return (-1, 0)
    elif (x == a and y - 1 == b):  
        return (0, -1)
    elif (x + 1 == a and y + 1 == b):  
        return (1, 1)
    elif (x + 1 == a and y - 1 == b):  
        return (1, -1)
    elif (x - 1 == a and y + 1 == b):  
        return (-1, 1)
    elif (x - 1 == a and y - 1 == b):  
        return (-1, -1)
    else:
        return (0,0)
