# This is necessary to find the main code
import sys
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
from math import sqrt
from world import World
import queue

class TestCharacter(CharacterEntity):

    def do(self, wrld):
        # Your code here
        #self.move(1, 0)
        # Start with A*
            # g(n) = manhattan distance
            # h(n) = euclidean distance
        start = {self.x, self.y}
        goal = wrld.exitcell
        graph = wrld.grid

        path, cost = self.a_star_search(graph, start, goal)

        #for node in path:
            # something like this
        #    self.move(node)

    def a_star_search(self, graph, start, goal):
        frontier = queue.PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current = frontier.get()
            
            if current == goal:
                break
            
            for next in graph.neighbors(current):
                new_cost = cost_so_far[current] + self.manhattan(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
        
        return came_from, cost_so_far

    def heuristic(self, goal, current):
        """
        Calculate Euclidean distance between start and goal
        Start and goal are tuples of {x, y}
        """
        current_x, current_y = current
        goal_x, goal_y = goal

        x = abs(goal_x - current_x)
        y = abs(goal_y - current_y)

        cost = sqrt((x * x) + (y * y))

        return cost

    def manhattan(self, start, current):
        """
        Calculate the Manhattan distance between start and goal
        Start and goal are tuples of {x, y}
        """
        start_x, start_y = start
        current_x, current_y = current

        x = abs(current_x - start_x)
        y = abs(current_y - start_y)

        cost = x + y
