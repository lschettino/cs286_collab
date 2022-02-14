'''
Harvard CS 286 Spring 2022
'''

import random
from ninja_turtles import NinjaTurtles
#import rospy
import time
import pprint

class Robot:
    def __init__(self, x, y, name):
        # index for row position in grid env
        self.x = x
        # index for column position in grid env
        self.y = y
        self.neighbors = set()
        self.sim_bot = NinjaTurtles(x,y,name)
        self.sim_bot.remove_bot('turtle1')
        self.sim_bot.add_bot()

class Env:
    def __init__(self, bots, size=10):
        # number of rows and columns in square grid
        self.size = size 
        # list of Robot objects
        self.bots = bots 
        # number of Robots in this Env
        self.num_bots = len(bots)
        # 2D list containing sets, empty or otherwise, of Robot objects at each coordinate location
        self.grid = self.update_grid()
        
        # List to store different flocks
        self.flocks = [set(bots)]
        
        self.steps = 0



    ################ General Helper Functions ######################        
    def move_bot(self, bot, move_cmd):
        '''
        Update position of bot (Robot obj) using move_cmd (tuple).
        Note that move_cmd = (x,y) where x and y are each integers between -1 and 1, inclusive.
        '''
        bot.x += move_cmd[0]
        bot.y += move_cmd[1]
        if bot.x >= self.size:
            bot.x = self.size-1
        if bot.x < 0:
            bot.x = 0
        if bot.y >= self.size:
            bot.y = self.size-1
        if bot.y < 0:
            bot.y = 0
        
        bot.sim_bot.go_to(bot.x,bot.y)

    
    def update_grid(self):
        grid = [[set() for i in range(self.size)] for i in range(self.size)]
        for b in self.bots:
            grid[b.x][b.y].add(b)
	
        self.grid = grid 
        return grid


    def display_grid(self):
        self.update_grid()
        # prints grid with number of bots in each coordinate location
        print("Grid["+("%d" %self.size)+"]["+("%d" %self.size)+"]")
        for j in range(self.size-1,-1,-1):
            print(j ,'|', end =" ")
            for i in range(0,self.size):
                print(len(self.grid[i][j]), end ="  ") # switched from i j 
            print()
        
        print("--", end="  ")
        for i in range(0,self.size):
            print("-", end ="  ")
        print()

        print("  ", end="  ")
        for i in range(0,self.size):
            print(i, end ="  ")
        print()
    
    
    def _move_towards_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step towards the location.
        '''
        x,y = bot.x, bot.y 
        '''
          Your code to update x,y
        '''
        x_move = min(1, max((loc[0] - x), -1))
        y_move = min(1, max((loc[1] - y), -1))

        move_cmd = x_move, y_move

        self.move_bot(bot, move_cmd)

        return (bot.x, bot.y)


    def _move_away_step(self, bot, loc):
        '''
        Moves a bot (Robot obj) by one step away from a location.
        '''
        x,y = bot.x, bot.y 
        '''
          Your code to update x,y
        '''
        x_move = min(1, max((loc[0] - x), -1))
        y_move = min(1, max((loc[1] - y), -1))
        
        if x_move == 0 and y_move == 0: 
            self._move_random_step(bot)
        else: 
            move_cmd = (-1 * x_move), (-1 * y_move)
            self.move_bot(bot, move_cmd)
            return (bot.x, bot.y)        

    def _move_random_step(self, bot): 
        '''
        Moves a bot (Robot obj) by one step towards a random location.
        '''
        x_step, y_step = bot.x , bot.y
        '''
          Your code to update y_step, y_step
        '''
        loc = (random.randint(0, self.size), random.randint(0, self.size))
        self._move_towards_step(bot, loc) 
        return (bot.x, bot.y)
    

    def get_centroid(self, flock):
        '''
        Calulcate the centroid of a flock using bot (Robot Obj) positions
        '''
        x_c, y_c = 0,0
        '''
        Your code to update x_c, y_c
        '''
        if self.num_bots == 0:
            print('There are no robots for which to compute ')
        
        x_c = int(sum([bot.x for bot in flock])/len(flock))
        y_c = int(sum([bot.y for bot in flock])/len(flock))

        return (x_c, y_c)

    
    def bot_sense(self, bot, sense_r):
        '''
        Get the neighboring robots of a bot (Robot obj) within its sensing radius
        Hint: self.grid stores the positions of all the bots (Robot obj) in a given iteration. This can be used to find the neightbors of a bot using its position.
        Note: A bot is not a neighbor of itself.
        '''
        neighbors = set()
        x_low = max(bot.x - sense_r, 0)
        x_high = min(bot.x + sense_r, self.size-1)
        y_low = max(bot.y - sense_r, 0)
        y_high = min(bot.y + sense_r, self.size-1)

        
        for i in range(x_low, x_high + 1): 
            for j in range(y_low, y_high + 1): 
                neighbors = neighbors.union(self.grid[i][j])

        assert bot in neighbors, f"""Bot {bot.x,bot.y} {self.grid[bot.x][bot.y]} is not in the neighborhood {neighbors}\n
        Range x:{max(bot.x - sense_r, 0), min(bot.x + sense_r + 1, self.size)}, y: {max(bot.y - sense_r, 0), min(bot.y + sense_r + 1, self.size)}"""
        neighbors.remove(bot) 
        bot.neighbors = neighbors  


    def update_flocks(self, grouping_alg = 'greedy'):
        '''
        Generate flock(s) at each timestep based on the position of each robot and the robots within its neighborhood
        '''

        flocks = [] 

        if grouping_alg == 'greedy':
            bot_in_flocks = set()

            for bot in self.bots: 
                # ensure each bot can only belong to one flock 
                if bot not in bot_in_flocks: 
                    # only worry about bots that are not in a flock already
                    
                    # Create new flock with current bot
                    new_flock = set([bot])
                    for neighbor in bot.neighbors: 
                        # add neighbors to flock if they aren't in another flock
                        if neighbor not in bot_in_flocks:
                            new_flock.add(neighbor)
                    
                    bot_in_flocks = bot_in_flocks.union(new_flock)
                    flocks.append(new_flock)



        elif grouping_alg == 'transitive':
            # If two non-neighboring robots have a neighbor in common, they are part of the same flock
            for bot in self.bots:
                a=0

        print(f'After updating, there are {len(flocks)} flocks')
        self.flocks = flocks
                
        
    
    
    ################ General Helper Functions ######################


    ################ Centralized communication ######################
    def flock(self, loc, t=5):
        '''
        Aggregate all bots to grid coordinate loc (tuple)
        Then have the flock safe wander for t (int) steps.
        Afterwards, disperse. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate(loc)
        self.display_grid()
        time.sleep(3)
        
        for i in range(t):
            print(f"SAFE WANDER {i}")
            self.safe_wander(True)
            self.display_grid()
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse()
        self.display_grid()



    def aggregate(self, loc):
        '''
        Move all bots to grid coordinate loc (tuple).
        After this method is called, all aggregation should be complete (each bot will have taken all
        steps, likely more than one, to completely aggregate.)
        
        Use move_bot() and _move_towards() functions
        '''

        # Check that all of the bots are at the given grid coordinate
        while not all([(bot.x,bot.y) == loc for bot in self.bots]): 
            for bot in self.bots: 
                if (bot.x, bot.y) != loc: 
                    self._move_towards_step(bot, loc) 

                

    def safe_wander(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction

        Use move_bot() and _move_random_step() functions
        '''
        # all bots in a flock move in the same direction 
        if flock: 
            print("\tWander as a flock")   
            for flock in self.flocks: 
                # Generate random location to move towards
                loc = (random.randint(0, self.size), random.randint(0, self.size))

                # Move every bot in the flock towards the chosen random location 
                for bot in flock: 
                    self._move_towards_step(bot, loc)
        else:
            for bot in self.bots: 
                self._move_random_step(bot)
    

            
    def disperse(self):
        '''
        Move all bots away from centroid, each in a random direction, for 3 steps.
        Use the move_bot(), _move_away_step() and get_centroid() functions.
        '''
        # move in direction opposite to the vector pointing to centroid 
        centroids = [] 
        for flock in self.flocks: 
            x_c, y_c = self.get_centroid(flock)
            centroids.append((x_c, y_c))
        for i in range(3): 
            for j, flock in enumerate(self.flocks): 
                for bot in flock: 
                    self._move_away_step(bot, centroids[j])
            self.display_grid()
 

    
    ################ Centralized communication ######################





    ################ Decentralized Communication ######################
    def flock_sense(self, sense_r, t=5):
        '''
        Aggregate all bots using sensing radius sense_r.
        Then have the flock(s) safe wander for t (int) steps.
        Afterwards, disperse flock/s beyond aggregation centroid/s. 
        Display the grid after each of these steps, including after each safe wander interation.
        '''
        print("AGGREGATE")
        self.aggregate_sense(sense_r)
        self.display_grid()

        time.sleep(3)


        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
            self.display_grid()

        
        time.sleep(3)
        print("DISPERSE")
        self.disperse_sense()
        self.display_grid()


    def aggregate_sense(self, sense_r):
        '''
        Aggregate bots into one or more flocks, each using sensing radius of sense_r (int).
        Use bot_sense() and update_flocks() functions
        '''
        
        
        def is_abs_aggregate():
            ''' 
            Check wether all bots in the grid are in the same location  
            
            '''
            
            # Select first robot in the grid to see if all the rest are at its location
            bot = self.bots[0]
            loc = bot.x, bot.y
            return all([(bot.x,bot.y) == loc for bot in self.bots])

        for bot in self.bots:
            self.bot_sense(bot, sense_r)
        self.update_flocks()

       
        while not is_abs_aggregate():            
            for index, flock in enumerate(self.flocks): 
                flock_centroid = self.get_centroid(flock)

                while not all([(bot.x,bot.y) == flock_centroid for bot in flock]):


                    # move robots towards the centroid
                    for bot in flock: 
                        # Only move the bots that are not at the centroid
                        if (bot.x, bot.y) != flock_centroid: 
                            self._move_towards_step(bot, flock_centroid)
                    
                    # Update flock centroid after all robots have moved in the direction of the centroid
                    flock_centroid = self.get_centroid(flock)
                
            self.safe_wander(True) 
            self.update_grid() 
            for bot in self.bots: 
                 self.bot_sense(bot,sense_r)
            self.update_flocks() 
 


    def safe_wander_sense(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction
        '''
        
        # all bots in a flock move in the same direction 
        self.safe_wander(True)


    def disperse_sense(self):
        '''
        Move all bots away from their respective flock's centroid.
        '''

        self.disperse()
        

    ################ Decentralized Communication ######################


if __name__ == "__main__":
    #rospy.init_node("Cowabunga")
    #Use the same names when generating results for part HW1 Q1.(d)
    bot1 = Robot(1,1,'t1')
    bot2 = Robot(9,1,'t2')
    bot3 = Robot(9,3,'t3')
    bot4 = Robot(1, 5,'t4')

    #bot5 = Robot(2,4,'t5')
    #bot6 = Robot(6,4,'t6')
    #bot7 = Robot(9,7,'t7')
    #bot8 = Robot(2,6,'t8')

    #bot9 = Robot(1,1,'t9')
    #bot10 = Robot(4,5,'t10')
    #bot11 = Robot(8,8,'t11')
    #bot12 = Robot(1,8,'t12')

    env_size = 14

    bots = [bot1, bot2, bot3, bot4]


    print(f'Running with {len(bots)} robots')

    env = Env(bots, env_size)

    #env.flock((5,5))
    
    time.sleep(3)
    env.flock_sense(3)
    
