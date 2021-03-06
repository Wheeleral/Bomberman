# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from math import sqrt, inf
import heapq
import random
from operator import itemgetter

class TestCharacter(CharacterEntity):			
    pathV = 0
    directions = []
    i = 1
    max_depth = 1 # need to figure out how to set this
    wavefront = [] # 2d array for grid
    explosion = False
    #explosion = 0
    time_explode = 0

    def do(self, wrld):
        print("MOSNTOR", wrld.monsters)
        allmons = wrld.monsters.items()
        for w in wrld.monsters:
            print("A MON",)
        print("time:", self.time_explode)
        if self.wavefront == []:
            self.wavefront = [[0] * wrld.height() for i in range(wrld.width())]
            self.init_wavefront(wrld)
            self.populate_wavefront(wrld)
            print("wavefront: ", self.wavefront)
        # character will always be near explosion
        #if self.explosion_near(self.x, self.y, wrld):
        #    self.populate_wavefront(wrld) # rerun wavefront when a wall dies
        #self.explosion_wrld(wrld)
        if self.time_explode == 3:
            self.populate_wavefront(wrld)
            self.explosion = False
            self.time_explode = 0
            print("wavefront: ", self.wavefront)
                 
        action= self.search(wrld, self.max_depth)
        dx = action[0] - self.x
        dy = action[1] - self.y
        self.move(dx, dy)

        self.explosion = self.explosion_wrld(wrld)

        if self.explosion:
            self.time_explode += 1
        
        # Determine whether or not to place a bomb
        # FOR NOW: >0 and >1 passes variant5
        if self.getObstaclesAt(self.x, self.y, wrld) > 0:
            self.place_bomb()
        
        pass

    def explosion_wrld(self, wrld):
        """if wrld.explosion_at(x - 1, y - 1):
            return True
        if wrld.explosion_at(x, y - 1):
            return True
        if wrld.explosion_at(x + 1, y - 1):
            return True
        if wrld.explosion_at(x - 1, y):
            return True
        if wrld.explosion_at(x + 1, y):
            return True
        if wrld.explosion_at(x - 1, y + 1):
            return True
        if wrld.explosion_at(x, y + 1):
            return True
        if wrld.explosion_at(x + 1, y + 1):
            return True"""
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.explosion_at(x, y):
                    #self.populate_wavefront(wrld)
                    return True
        
        return False

    """terminal test checks if state is max depth"""
    # might need to add more to this
    def terminal_test(self, depth):
        if depth >= self.max_depth:
            return True

        return False

    """returns a utility value"""
    def max_value(self, state, a, depth):
        if self.terminal_test(depth):
            return self.score_state(state, a)  # need to write this function
        
        v = -inf
        for action in self.get_successors(state, a):  # need to write this function
            v = max(v, self.exp_value(action[0], action[1], depth + 1))
            """if v >= self.beta:  
                return v
            
            self.alpha = max(self.alpha, v)"""
        
        return v

    """returns a utility value"""
    def exp_value(self, state, a, depth):
        if self.terminal_test(depth):
            return self.score_state(state, a)  # need to write this function
        
        v = 0
        # TODO: define p
        for action in self.get_successors(state, a):
            # probability for second scene will be equal across all 8 neighbors and for staying put
            # for scene 2, p = 1/9 ???
            # p <- PROBABILIY(action)
            p = 0.8  # dummy value for now
            v = v + (p * self.max_value(action[0], action[1], depth + 1))
            print("EXp", action[1], " v", v )
        
        return v

    """expectimax search"""
    def search(self, state, max_depth):
        current_depth = 0
        best_value = -inf
        best_action = None
        v = -inf
        pos = (self.x, self.y)
        # grab the board and column for each successor
        for s, a in self.get_successors(state, pos):  # need to define this
            # start recursive search for best value

            v = max(v, self.exp_value(s, a, current_depth + 1))
            print("V", v, " A ", a)
            if v > best_value:
                best_value = v 
                best_action = a #tuple of x, y

            # if an action results in a win, take that action
            if s.exit_at(a[0], a[1]): 
                return a

        return best_action 

    """return a score for a world"""
    def score_state(self, wrld, action):
        score = 0
        dist = 30 # dummy value for now
        x = action[0]
        y = action[1]

        # at goal, give HUGE number
        # if within 2 spaces of monster, negative number
        # if within 1 space of monster, big negative number
        # if on monster, HUGE negative number
        # if within bomb radius, big negative number
        # higher number as we near the goal

        # nearing the goal:
        score += self.wavefront[x][y]

        # at goal:
        if wrld.exit_at(x, y):
            print("at exit")
            score += 100

	    #Checking for monsters
        if self.nearMonster(x, y, wrld):
	        score += -100
		
        if self.nearMonster2(x, y, wrld):
	        score += -80
		
        if self.nearMonster3(x,y, wrld):
	        score += -50


        if wrld.monsters_at(x,y):
            score += -1000
        
        # Checking whether within explosion range
        if self.nearBomb(x,y,wrld) > 0:
            # Determine how negative based on how close the character is to the bomb
            score += -(1 / self.nearBomb(x,y,wrld)) * 100
        
        #Determine how long till bomb explodes
        if wrld.bomb_at(x,y) is not None:
            if wrld.bomb_at(x,y).timer < 2:
                score += -1000
        
        # Checking for explosion - avoid going towards it
        if wrld.explosion_at(x, y) is not None:
            score += -1000
        print("Score", score, action)
        return score

    """ return a list of possible actions from current character position"""
    def get_successors(self, state, a):
        x = a[0] 
        y = a[1]

        successors = []
        # check for valid neighbors
        if self._validate(x + 1, y, state):
            successors.append((state, (x + 1, y)))  
        if self._validate(x - 1, y, state):
            successors.append((state, (x - 1, y)))
        if self._validate(x, y + 1, state):
            successors.append((state, (x, y + 1)))
        if self._validate(x, y - 1, state):
            successors.append((state, (x, y - 1)))
        if self._validate(x + 1, y + 1, state):
            successors.append((state, (x + 1, y + 1)))
        if self._validate(x + 1, y - 1, state):
            successors.append((state, (x + 1, y - 1)))
        if self._validate(x - 1, y + 1, state):
            successors.append((state, (x - 1, y + 1)))
        if self._validate(x - 1, y - 1, state):
            successors.append((state, (x - 1, y - 1)))
        if self._validate(x, y, state):
	        successors.append((state, (x,y)))
        
        # return all neighbors that aren't obstacles
        return successors

    def _validate(self, x, y, wrld):  
        # check within bounds
        if(x >= 0 and x < wrld.width() and y >= 0 and y < wrld.height()):
            # check not a wall
            if not wrld.wall_at(x, y):
                return True
        
        return False

	#Returns true if within one square of a monster
    def nearMonster(self, x, y, wrld):
        if(wrld.monsters_at(x+1, y) or wrld.monsters_at(x-1, y) or wrld.monsters_at(x, y+1) 
		    or wrld.monsters_at(x, y-1) or wrld.monsters_at(x+1, y+1) or wrld.monsters_at(x+1, y-1) 
		    or wrld.monsters_at(x-1, y+1) or wrld.monsters_at(x-1, y-1)):
	        return True
        return False
	#Return true is within two squares of a monster
    def nearMonster2(self, x, y, wrld):
	    if(wrld.monsters_at(x+2, y) or wrld.monsters_at(x-2, y) or wrld.monsters_at(x, y+2) 
		    or wrld.monsters_at(x, y-2) or wrld.monsters_at(x+2, y+1) or wrld.monsters_at(x+2, y+2)
		    or wrld.monsters_at(x+2, y-2) or wrld.monsters_at(x+2, y-1) 
	        or wrld.monsters_at(x-2, y+2) or wrld.monsters_at(x-2, y-2)
		    or wrld.monsters_at(x-2, y+1) or wrld.monsters_at(x-2, y-1)):
		    return True
	    else:
		    return False

    def nearMonster3(self, x, y, wrld):
	    if(wrld.monsters_at(x+3, y) or wrld.monsters_at(x-3, y)
		    or wrld.monsters_at(x+3, y+1) or wrld.monsters_at(x+3, y+2) or wrld.monsters_at(x+3, y+3)
		    or wrld.monsters_at(x+3, y-2) or wrld.monsters_at(x+3, y-1) or wrld.monsters_at(x+3, y-3) 
	        or wrld.monsters_at(x-3, y+2) or wrld.monsters_at(x-3, y+1) or wrld.monsters_at(x-3, y+3)
		    or wrld.monsters_at(x-3, y-2) or wrld.monsters_at(x-3, y-1) or wrld.monsters_at(x-3, y-3)
            or wrld.monsters_at(x+2, y+3) or wrld.monsters_at(x+2, y-3) 
            or wrld.monsters_at(x+1, y+3) or wrld.monsters_at(x+1, y-3)
	        or wrld.monsters_at(x, y+3) or wrld.monsters_at(x, y-3)
	        or wrld.monsters_at(x-1, y+3) or wrld.monsters_at(x-1, y-3)
	        or wrld.monsters_at(x-2, y+3) or wrld.monsters_at(x-2, y-3)
			):
		    return True
	    else:
		    return False

    # Return the distance from bomb if is within the range of explosion
    def nearBomb(self, x, y, wrld):
        bomb_distance = []
        
        # Distance from bomb in the x-position
        for xs in range(-4, 5, 1):
            if self._withinBound(x + xs, y, wrld):
                if wrld.bomb_at(x + xs, y):
                    bomb_distance.append(abs(xs))
                    # return abs(xs)
        
        # Distance from the bomb in the y-position
        for ys in range(-4, 5, 1):
            if self._withinBound(x, y + ys, wrld):
                if wrld.bomb_at(x, y + ys):
                    bomb_distance.append(abs(ys))
                    # return abs(ys)
        
        # Take the closest distance in either direction to the bomb
        if len(bomb_distance) > 0:
            return min(bomb_distance)
        
        return 0
    
    # Determines whether the position is within grid world
    def _withinBound(self, x, y, wrld):
        # check within bounds
        if(x >= 0 and x < wrld.width() and y >= 0 and y < wrld.height()):
            return True
        
        return False
        
    # Evaluates the number of obstacles in the current position of character 
    def getObstaclesAt(self, x, y, wrld):
        obstacles = 0
        
        # Determine the number of obstacles (hits) in the explosion range
        for xs in range(1, 5):
            # Get first obstacle on the right
            if self._withinBound(x + xs, y, wrld):
                if wrld.wall_at(x + xs, y):
                    obstacles += 1
                    break
                
                if wrld.monsters_at(x + xs, y):
                    obstacles += 1
                    break
                    
        for xs in range(1, 5):
            # Get first obstacle on the left
            if self._withinBound(x - xs, y, wrld):
                if wrld.wall_at(x - xs, y):
                    obstacles += 1
                    break
                
                if wrld.monsters_at(x - xs, y):
                    obstacles += 1
                    break
        
        for ys in range(1, 5):
            # Get first obstacle at the top
            if self._withinBound(x, y + ys, wrld):
                if wrld.wall_at(x, y + ys):
                    obstacles += 1
                    break
                
                if wrld.monsters_at(x, y + ys):
                    obstacles += 1
                    break
        
        for ys in range(1, 5):
            # Get first obstacle at the bottom
            if self._withinBound(x, y - ys, wrld):
                if wrld.wall_at(x, y - ys):
                    obstacles += 1
                    break
                
                if wrld.monsters_at(x, y - ys):
                    obstacles += 1
                    break
            
        return obstacles

    # Wavefront for cost of each cell
    def init_wavefront(self, wrld):
        # initialize wavefront
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                #if wrld.wall_at(x, y): 
                    #self.wavefront[x][y] = -1
                pass

        exit_x, exit_y = wrld.exitcell
        self.wavefront[exit_x][exit_y] = 100 # this is the goal
        #print("init:", self.wavefront)

    def populate_wavefront(self, wrld):
        print("enter populate")
        neighbors = []
        visited = []
        value = 100
        exit_x, exit_y = wrld.exitcell

        neighbors = self.get_neighbors(exit_x, exit_y, value - 1, wrld) # now includes walls
        visited.append((exit_x, exit_y))
        for n in neighbors:
            visited.append(n[1])

        while neighbors:
            #print("neighbors", neighbors)
            # grab first neighbor from list 
            curr_val, curr_coord = neighbors[0]
            visited.append(curr_coord)
            curr_x, curr_y = curr_coord
            # give it a value in wavefront
            if wrld.wall_at(curr_x, curr_y):
                self.wavefront[curr_x][curr_y] = curr_val - 5 # minus 5 for now
                curr_val -= 5
            else:
                self.wavefront[curr_x][curr_y] = curr_val

            # call neighbors on it and add to list
            children = self.get_neighbors(curr_x, curr_y, curr_val - 1, wrld)
            for c in children:
                if c[1] not in visited: # only add if we haven't checked this spot yet
                    neighbors.append(c)
                    visited.append(c[1])
            # remove from list
            neighbors.remove(neighbors[0])

    def get_neighbors(self, x, y, value, state):
        # list of tuples containing the value and a tuple of the coordinates
        neighbors = []
        # check for valid neighbors
        if self._withinBound(x + 1, y, state):
            neighbors.append((value, (x + 1, y)))  
        if self._withinBound(x - 1, y, state):
            neighbors.append((value, (x - 1, y)))
        if self._withinBound(x, y + 1, state):
            neighbors.append((value, (x, y + 1)))
        if self._withinBound(x, y - 1, state):
            neighbors.append((value, (x, y - 1)))
        if self._withinBound(x + 1, y + 1, state):
            neighbors.append((value, (x + 1, y + 1)))
        if self._withinBound(x + 1, y - 1, state):
            neighbors.append((value, (x + 1, y - 1)))
        if self._withinBound(x - 1, y + 1, state):
            neighbors.append((value, (x - 1, y + 1)))
        if self._withinBound(x - 1, y - 1, state):
            neighbors.append((value, (x - 1, y - 1)))
        
        # return all neighbors that aren't obstacles
        return neighbors


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
