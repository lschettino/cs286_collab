'''
Harvard CS 286 Spring 2022
'''

import random
from ninja_turtles import NinjaTurtles
import rospy
import time

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
            print(b.x, b.y)
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
        n_robots = len(flock)
    
    # what is the centroid of a flock with no robots? 
        if n_robots > 0: 
            x_sum = 0 
            y_sum = 0 
            for bot in self.bots:
                x_sum += bot.x 
                y_sum += bot.y 
            x_c = x_sum / n_robots
            y_c = y_sum / n_robots 
        else: 
            n_robots = len(self.bots)
            x_sum = 0 
            y_sum = 0
            for robot in self.bots: 
                x_sum += bot.x 
                y_sum += bot.y 
            x_c = x_sum / n_robots
            y_c = y_sum / n_robots
        return (int(x_c), int(y_c))

    
    def bot_sense(self, bot, sense_r):
        '''
        Get the neighboring robots of a bot (Robot obj) within its sensing radius
        Hint: self.grid stores the positions of all the bots (Robot obj) in a given iteration. This can be used to find the neightbors of a bot using its position.
        Note: A bot is not a neighbor of itself.
        '''
        # should we form a "circle" or a square with the radius? 
        neighbors = set()
        x, y = bot.x, bot.y 
        min_x = max(x - sense_r, 0) 
        max_x = min(x + sense_r, self.size - 1) 
        min_y = max(y - sense_r, 0) 
        max_y = min(y + sense_r, self.size - 1)  
        for i in range(min_x, max_x): 
            for j in range(min_y, max_y): 
                neighbors.union(self.grid[i][j])
        neighbors.remove(bot) 
        bot.neighbors = neighbors  


    def update_flocks(self):
        '''
        Generate flock(s) at each timestep based on the position of each robot and the robots within its neighborhood
        '''
        

        '''
            Your code to update self.flocks
        '''
        flocks = [] 
        bot_in_flocks = set()
        for bot in self.bots: 
            # ensure each bot can only belong to one flock 
            if bot in bot_in_flocks: 
                continue 
            else: 
                # include bot in flock
                new_flock = set([bot])
                for neighbor in bot.neighbors: 
                    # add neighbors to flock if they aren't in another flock
                    if neighbor not in bot_in_flocks:
                        new_flock.add(neighbor)
                bot_in_flocks.union(new_flock)
                flocks.append(new_flock)
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
        
        for count in range(t):
            print("SAFE WANDER", count)
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
        # am i aggregate? (outer loop) 
	# move the bots 
        def is_aggregate(): 
            for bot in self.bots: 
                if (bot.x, bot.y) != loc: 
                    return False 
            return True     
        while not is_aggregate(): 
            for bot in self.bots: 
                if (bot.x, bot.y) != loc: 
                    self._move_towards_step(bot, loc) 

                
    # one single flock for part a 

    def safe_wander(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction

        Use move_bot() and _move_random_step() functions
        '''
    # all bots in a flock move in the same direction 
# in general, if everyone is connected 
        if flock: 
            print("flock is true")   
            for each_flock in self.flocks: 
                print("looping through flock")
                loc = (random.randint(0, self.size), random.randint(0, self.size))
                for bot in each_flock: 
                    print(str(bot.x) + "," + str(bot.y)) 
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
        time.sleep(3)

        for count in range(t):
            print("SAFE WANDER", count)
            self.safe_wander(True)
        
        time.sleep(3)
        print("DISPERSE")
        self.disperse_sense()
        self.display_grid()


    def aggregate_sense(self, sense_r):
        '''
        Aggregate bots into one or more flocks, each using sensing radius of sense_r (int).
        Use bot_sense() and update_flocks() functions
        '''
        pass


    def safe_wander_sense(self, flock=False):
        '''
        Move each bot one step. 
        If flock, all bots in a flock move in same random direction.
        Otherwise, each bot moves in its own random direction
        '''
        pass


    def disperse_sense(self):
        '''
        Move all bots away from their respective flock's centroid.
        '''
        pass

    ################ Decentralized Communication ######################


if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    
    #Use the same names when generating results for part HW1 Q1.(d)
    bot1 = Robot(1,1,'t1')
    bot2 = Robot(9,1,'t2')
    bot3 = Robot(9,9,'t3')
    bot4 = Robot(1, 5,'t4')



    bots = [bot1, bot2, bot3, bot4]

    env = Env(bots, 14)


    env.flock((2, 2)) 

    

    # env.flock((5,5))

    # env.flock_sense(2)
