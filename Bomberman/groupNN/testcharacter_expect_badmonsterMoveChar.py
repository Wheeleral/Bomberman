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
from sensed_world import SensedWorld

# TODO: modify score function values

class TestCharacter(CharacterEntity):			
    pathV = 0
    directions = []
    i = 1
    max_depth = 2 # need to figure out how to set this
    wavefront = [] # 2d array for grid
    explosion = False
    time_explode = 0

    def do(self, wrld):
        #print("time:", self.time_explode)
        if self.wavefront == []:
            self.wavefront = [[0] * wrld.height() for i in range(wrld.width())]
            self.init_wavefront(wrld)
            self.populate_wavefront(wrld)
            #print("wavefront: ", self.wavefront)

        if self.time_explode == 3:
            self.populate_wavefront(wrld)
            self.explosion = False
            self.time_explode = 0
            #print("wavefront: ", self.wavefront)
                 
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

    #---------- EXPECTIMAX SEARCH ----------#
    """checks if state is max depth"""
    def terminal_test(self, depth):
        if depth >= self.max_depth:
            return True

        return False

    """returns a utility value"""
    def max_value(self, wrld, a, depth):
        if self.terminal_test(depth):
            return self.score_state(wrld, a)  
        
        v = -inf
        for action in self.get_successors(state):  
            cpyWrld = SensedWorld.from_world(action[0])
            me = cpyWrld.me(self)
            #print("ME", me, a[0], a[1])
            me.x = action[1][0]
            me.y = action[1][1] 
            v = max(v, self.exp_value(cpyWrld, action[1], depth + 1))
        
        return v

    """returns a utility value"""
    """THIS IS HOW THE MONSTER MOVES"""
    def exp_value(self, wrld, a, depth):
        if self.terminal_test(depth):
            return self.score_state(wrld, a)  
        
        v = 0
        closest_monster = self.find_monster(wrld)
        #print("monster loc:", closest_monster[1])

        # if monster no longer exists 
        if not closest_monster:
            v = v + (self.max_value(wrld, a, depth + 1)) 

            return v
     
        # get successors of that monster in copied worlds
        monster_actions = self.get_monster_actions(wrld, closest_monster[1])
        
        for action in monster_actions: 
            action[0].next() # move the monster
            
            p = self.monster_prob(action[0], a, action[1])
            #print("monster action ", action, "prob", p)
            #p = 1/8
            # pass copied world with new monster loc and the old action for the character
            v = v + ((p) * self.max_value(action[0], a, depth + 1)) 
        
        return v

    """expectimax search"""
    def search(self, wrld, max_depth):
        current_depth = 0
        best_value = -inf
        best_action = None
        v = -inf

        # grab the board and column for each successor
        for s, a in self.get_successors(wrld):  # need to define this
            # start recursive search for best value
            cpyWrld = SensedWorld.from_world(s)
            me = cpyWrld.me(self)
            #print("ME", me, a[0], a[1])
            #me.x = a[0]
            #me.y = a[1]
            me.move(a[0] - me.x, a[1] - me.y )
            v = max(v, self.exp_value(cpyWrld, a, current_depth + 1))
            print("V", v, " A", a)
            if v > best_value:
                best_value = v 
                best_action = a #tuple of x, y

            # if an action results in a win, take that action
            if s.exit_at(a[0], a[1]): 
                return a

        return best_action 

    """returns the most likely move for monster"""
    def monster_prob(self, wrld, char_loc, monster_loc):
        cx, cy = char_loc
        mx, my = monster_loc

        # find distance between character and closest monster
        dx = abs(mx - cx)
        dy = abs(my - cy)
        euclidean = sqrt((dx * dx) + (dy * dy))

        #print("char_loc:", char_loc, "monster_loc:", monster_loc, "weight:", 1/euclidean)

        # monster will probably chase character, therefore, higher weight goes to actions
        # that move monster closer to character
        if(euclidean == 0):
	        return 1
        return 1/euclidean

    """returns a score for a world"""
    def score_state(self, wrld, action):
        score = 0
        dist = 30 # dummy value for now
        x =  action[0]
        y =action[1]
        print("x", x, "y", y)

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
            score += 10000

	    #Checking for monsters
        if self.nearMonster(x, y, wrld):
	        score += -800
		
        if self.nearMonster2(x, y, wrld):
	        score += -600
		
        if self.nearMonster3(x, y, wrld):
	        score += -550

        if self.nearMonster4(x, y, wrld):
	        score += -350

        if wrld.monsters_at(x, y):
            score += -1000
        
        if self.surrounded(x, y, wrld) < 5:
	        score += -500
        if self.surrounded(x, y, wrld) < 7:
	        score += -100
        
        # Checking whether within explosion range
        if self.nearBomb(x, y, wrld) > 0:
            # Determine how negative based on how close the character is to the bomb
            #print("bomb value of ", x, " and ", y , "score", -(2 / self.nearBomb(x,y,wrld)) * 100 )
            score += -(3 / self.nearBomb(x,y,wrld)) * 300
        
        #Determine how long till bomb explodes
        if wrld.bomb_at(x,y) is not None:
            if wrld.bomb_at(x,y).timer < 2:
                score += -1000
        
        # Checking for explosion - avoid going towards it
        if wrld.explosion_at(x, y) is not None:
            score += -1000
        #print("score ", score, " a ", action)
        return score

    #---------- EXPECTIMAX HELPERS ----------#
    """returns a list of possible monster actions from current monster position"""
    def get_monster_actions(self, wrld, loc):
        x, y = loc  
        arr = [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1),
               (x - 1, y), (x + 1, y),
               (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]

        successors = []
        # check for valid neighbors
        for elt in arr:
            if self._validate(elt[0], elt[1], wrld):
                # copy the world so we can move the monster
                cpyWrld = SensedWorld.from_world(wrld)
                monster = cpyWrld.monsters_at(x, y)[0]
                monster.move(elt[0], elt[1])

                successors.append((cpyWrld, elt))
        
        # return all neighbors that aren't obstacles
        return successors

    """returns the nearest monster to the character"""
    def find_monster(self, wrld): 
        count = 0
        monsters = []
        closest_m = None
        closest_dist = 0
        me = wrld.me(self)
        x = me.x
        y = me.y

        # grab all monsters
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                curr_monsters = wrld.monsters_at(x, y)
                if curr_monsters:
                    for m in curr_monsters:
                        monsters.append((m, (x, y)))
        
        # return closest one
        for m in monsters:
            dx = abs(x - m[1][0])
            dy = abs(y - m[1][1])
            if (closest_m == None):
                closest_m = m
                closest_dist = sqrt((dx * dx) + (dy * dy))
            else:
                dist = sqrt((dx * dx) + (dy * dy))
                if dist < closest_dist:
                    closest_m = m
                    closest_dist = dist
        
        return closest_m

    """returns num of empty spaces nearby"""
    def surrounded(self, x, y, wrld):
        Nwalls = 0
        for xs in range (-1, 2, 1):
	        for ys in range(-1, 2 ,1 ):
		        if(self._withinBound(x + xs, y + ys, wrld)):
			        if(wrld.empty_at(x+xs, y + ys)):
				         Nwalls += 1
        return Nwalls

    """returns a list of possible character actions from current character position"""
    def get_successors(self, wrld):
        x = self.x 
        y = self.y

        arr = [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1),
               (x - 1, y), (x, y), (x + 1, y),
               (x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]

        successors = []
        # check for valid neighbors
        for elt in arr:
            if self._validate(elt[0], elt[1], wrld):
                successors.append((wrld, elt))
        
        # return all neighbors that aren't obstacles
        return successors

    """checks if location is within the world and not a wall"""
    def _validate(self, x, y, wrld):  
        # check within bounds
        if(x >= 0 and x < wrld.width() and y >= 0 and y < wrld.height()):
            # check not a wall
            if not wrld.wall_at(x, y):
                return True
        
        return False

    """checks if location is within one square of a monster"""
    def nearMonster(self, x, y, wrld):
        if(wrld.monsters_at(x+1, y) or wrld.monsters_at(x-1, y) or wrld.monsters_at(x, y+1) 
		    or wrld.monsters_at(x, y-1) or wrld.monsters_at(x+1, y+1) or wrld.monsters_at(x+1, y-1) 
		    or wrld.monsters_at(x-1, y+1) or wrld.monsters_at(x-1, y-1)):
	        return True
        return False

    """checks if location is within two squares of a monster"""
    def nearMonster2(self, x, y, wrld):
	    if(wrld.monsters_at(x+2, y) or wrld.monsters_at(x-2, y) or wrld.monsters_at(x, y+2) 
		    or wrld.monsters_at(x, y-2) or wrld.monsters_at(x+2, y+1) or wrld.monsters_at(x+2, y+2)
		    or wrld.monsters_at(x+2, y-2) or wrld.monsters_at(x+2, y-1) 
	        or wrld.monsters_at(x-2, y+2) or wrld.monsters_at(x-2, y-2)
		    or wrld.monsters_at(x-2, y+1) or wrld.monsters_at(x-2, y-1)):
		    return True
	    else:
		    return False
    
    """checks if location is within three squares of a monster"""
    def nearMonster3(self, x, y, wrld):
        for xs in range (-3, 4, 1):
	        for ys in range(-3,4 ,1 ):
		        if(self._withinBound(x + xs, y + ys, wrld)):
			        if(wrld.monsters_at(x+xs, y+ ys)):
				        return True
        return False
    
    """checks if location is within four squares of a monster"""
    def nearMonster4(self, x, y, wrld):
        for xs in range (-4, 5, 1):
	        for ys in range(-4,5 ,1 ):
		        if(self._withinBound(x + xs, y + ys, wrld)):
			        if(wrld.monsters_at(x+xs, y+ ys)):
				        return True
        return False
    
    """returns the distance from bomb if location is within the range of explosion"""
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
    
    """determines whether the position is within grid world"""
    def _withinBound(self, x, y, wrld):
        # check within bounds
        if(x >= 0 and x < wrld.width() and y >= 0 and y < wrld.height()):
            return True
        
        return False
        
    #---------- OTHER HELPERS ----------#
    """checks if there's an explosion anywhere in the world"""
    def explosion_wrld(self, wrld):
        for x in range(wrld.width()):
            for y in range(wrld.height()):
                if wrld.explosion_at(x, y):
                    return True
        
        return False

    """evaluates the number of obstacles in the current position of character"""
    def getObstaclesAt(self, x, y, wrld):
        obstacles = 0
        
        # Determine the number of obstacles (hits) in the explosion range
        for delta in range(1, 5):
            # Get first obstacle on the right
            if self._withinBound(x + delta, y, wrld):
                if wrld.wall_at(x + delta, y) or wrld.monsters_at(x + delta, y):
                    obstacles += 1
                    break
            
            # Get first obstacle on the left
            if self._withinBound(x - delta, y, wrld):
                if wrld.wall_at(x - delta, y) or wrld.monsters_at(x - delta, y):
                    obstacles += 1
                    break

            # Get first obstacle at the top
            if self._withinBound(x, y + delta, wrld):
                if wrld.wall_at(x, y + delta) or wrld.monsters_at(x, y + delta):
                    obstacles += 1
                    break       

            # Get first obstacle at the bottom
            if self._withinBound(x, y - delta, wrld):
                if wrld.wall_at(x, y - delta) or wrld.monsters_at(x, y - delta):
                    obstacles += 1
                    break
            
        return obstacles

    #---------- WAVEFRONT ALGORITHM -----------#
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
        neighbors = []
        visited = []
        value = 100
        exit_x, exit_y = wrld.exitcell

        neighbors = self.get_neighbors(exit_x, exit_y, value - 1, wrld) # now includes walls
        visited.append((exit_x, exit_y))
        for n in neighbors:
            visited.append(n[1])

        while neighbors:
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

    def get_neighbors(self, x, y, value, wrld):
        arr = [(x + 1, y), (x - 1, y), 
               (x, y + 1), (x, y - 1), 
               (x + 1, y + 1), (x + 1, y - 1), 
               (x - 1, y + 1), (x - 1, y - 1)]

        # list of tuples containing the value and a tuple of the coordinates
        neighbors = []

        for elt in arr:
            if self._withinBound(elt[0], elt[1], wrld):
                neighbors.append((value, elt))

        # return all neighbors that aren't obstacles
        return neighbors
