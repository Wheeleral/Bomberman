# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster

# TODO This is your code!
sys.path.insert(1, '../group24')
from testcharacter_expect_badmonsterMoveChar import TestCharacter #fudge4
#~50-60%

# Create the game
random.seed(8) # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
g.add_monster(SelfPreservingMonster("monster", # name
                                    "M",       # avatar
                                    3, 13,     # position
                                    2          # detection range
))

# TODO Add your character
g.add_character(TestCharacter("me", # name
                              "C",  # avatar
                              0, 0  # position
))


# Run!
g.go(1)
