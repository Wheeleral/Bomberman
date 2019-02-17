# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from math import sqrt
import heapq
import random

class AStarTestCharacter(CharacterEntity):			
    pathV = 0
    directions = []
    i = 1

    def do(self, wrld):
    	start = (self.dx, self.dy)
    	exit = wrld.exitcell
    	print("astar" ,start, exit)
    	
    	if(self.pathV == 0):
    	    path, self.directions = AStar(start, exit, wrld, wrld.width(), [])
    	    print("The path:", path)
    	    self.pathV=path
    	    for dir in self.pathV:
    	        self.set_cell_color(dir[0], dir[1], Fore.RED + Back.BLUE)

    	print("The path: ", self.pathV, "part ", self.pathV[self.i])
    	print("Directions", self.directions[self.i][0], "and ", self.directions[self.i][1])
    	self.move(self.directions[self.i][0], self.directions[self.i][1])
    	self.i+=1

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

"""use Manhattan distance to get current distance"""
def manhattan(p1, p2):  
    return abs(p1[0] - p2[0]) + abs(p2[1] - p2[1])

"""use Euclidean distance to get dis to goal"""
def heuristic(p1, p2):  
    x = abs(p1[0] - p2[0])
    y = abs(p1[1] - p2[1])
    return sqrt((x * x) + (y * y))

"""calculate the distance THIS IS THE MAIN FUNCTION"""
def AStar(start, end, gridCells, width, cost):  
    #set up frontier nodes using a min heap
    frontier = []
    #startNode = Node(None, start, 0, manhattan(start, end)) 
    startNode = Node(None, start, 0, manhattan(start, end)) 
    # puts items into min heap
    heapq.heappush(frontier, (startNode.f,  random.randint(1,1001)*random.randint(1,1001),  startNode))  
    #heapq.heappush(frontier, (startNode.f, startNode))  

    visited = {}
    
    while (len(frontier) > 0):  # if no solution, exit the while loop
        # The current node is the shortest distance
        cur = heapq.heappop(frontier) 
        cur = cur[2]
        #cur = cur[1]
        # don't pursue paths that have already been searched
        if (cur.pos in visited):  
            continue
        if (cur.pos == end):  # quit condition
            path = tracePath(start, cur, visited)  # probably trace path && return something useful
            dir = dirPath(path)
            return (path, dir)
        for neighbor in cur.getNeighbors(gridCells, width, end, cost):
            i = 0
            if (neighbor != None):
                heapq.heappush(frontier, (neighbor.f,  random.randint(1,1001)*random.randint(1,1001) , neighbor))  # put the neighbor into the min heap
                #heapq.heappush(frontier, (neighbor.f, neighbor))  # put the neighbor into the min heap

            i+=1
        visited[cur.pos] = cur  

    print("YOU FAILED")
    return (None, None, None)

"""Takes last node taken in from A* (Final) and the dictionary of visited nodes, 
and returns the path from start to end """
def tracePath(start, final, visited):
    path = []
    cur = final
    path.append(cur.pos)
    while (cur.pos != start):
        parent = visited.get(cur.parent.pos)
        # add to the empty list, path, the parent node, until reaching the start position/node
        path.append(parent.pos)  
        cur = cur.parent
    path.reverse()  # reverses the list so that it goes start to finish
    return path

"""gets the waypoints"""
def dirPath(path):  
    if not path:
        return
    currDir = findDir(path[0], path[1])
    nextDir = findDir(path[0], path[1])
    dir = []
    n = 0
    # check if the direction of each set of consecutive nodes is different
    for item in path: 
        if (len(path) > n + 1):
            nextDir = findDir(path[n], path[n + 1])

            n = n + 1
            dir.append(currDir)
            currDir = nextDir
    
    dir.append(currDir)
    return dir

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
