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
    max_depth = 4 # need to figure out how to set this
    #alpha = -math.inf
    #beta = math.inf

    def do(self, wrld):
    	pass

    """terminal test checks if state is max depth"""
    # might need to add more to this
    def terminal_test(self, depth):
        if depth >= self.max_depth:
            return True

        return False

    """returns a utility value"""
    def max_value(self, state, depth):
        if self.terminal_test(depth):
            return self.score_state(state)  # need to write this function
        
        v = -math.inf
        for action in self.get_successors(state):  # need to write this function
            v = max(v, self.exp_value(action[0], action[1], depth + 1))
            """if v >= self.beta:  
                return v
            
            self.alpha = max(self.alpha, v)"""
        
        return v

    """returns a utility value"""
    def exp_value(self, state, depth):
        if self.terminal_test(depth):
            return self.score_state(state)  # need to write this function
        
        v = 0
        # TODO: define p
        for action in self.get_successors(state):
            # probability for second scene will be equal across all 8 neighbors and for staying put
            # for scene 2, p = 1/9 ???
            # p <- PROBABILIY(action)
            p = 0.8  # dummy value for now
            v = v + (p * self.max_value(action[0], action[1], depth + 1))
            
            """if v <= self.alpha:
                return v
            
            self.beta = min(self.alpha, v) # maybe still here?"""
        
        return v

    """expectimax search"""
    def search(self, state, max_depth):
        current_depth = 0
        best_value = -math.inf
        best_action = None
        v = -math.inf

        # grab the board and column for each successor
        for s, a in self.get_successors(state):  # need to define this
            # start recursive search for best value
            v = max(v, self.exp_value(s, current_depth + 1))
            if v > best_value:
                best_value = v
                best_action = a

            # if an action results in a win, take that action
            if s.exit_at(a[0], a[1]): 
                return a

        return best_action 

    """ return a list of possible actions from current character position"""
    def get_successors(self, state):
        x = self.x 
        y = self.y

        successors = []
        # check for valid neighbors
        successors.append(self._validate(x + 1, y, wrld))  
        successors.append(self._validate(x - 1, y, wrld))
        successors.append(self._validate(x, y + 1, wrld))
        successors.append(self._validate(x, y - 1, wrld))
        successors.append(self._validate(x + 1, y + 1, wrld))  
        successors.append(self._validate(x + 1, y - 1, wrld))
        successors.append(self._validate(x - 1, y + 1, wrld))
        successors.append(self._validate(x - 1, y - 1, wrld))
        
        # return all neighbors that aren't obstacles
        return successors

    def _validate(self, x, y, wrld):  
        # check within bounds
        if(x >= 0 and x < wrld.width() and y >= 0 and y < wrld.height()):
            # check not a wall
            if not wrld.wall_at(x, y):
                return (wrld, (x, y))
        
        return None


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
