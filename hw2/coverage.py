import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.stats import multivariate_normal

class Robot(object):

    def __init__(self, state, k=0.1):
        self._state = state     # 2-vec
        self._stoch_state = self._state + np.random.randn(2)
        self.input = [0,0]      # movement vector later

        self.k = k * 0.001

    def update(self):     # update the robot state
        #print("update: ", self.k * self.input)
        self._state += self.k * self.input
        #print(self.state)
        self._stoch_state = self._state + np.random.randn(2)
        self.input = [0, 0]

    @property
    def state(self):
        return np.array(self._state)

    def stoch_state(self):
        return np.array(self._stoch_state)

    

class Environment(object):

    def __init__(self, width, height, res, robots, alpha = -10, sigma = 0, cov = 5, target = []):       # width and height are in pixels, so actual dimensions are width * res in meters
        self.width = width
        self.height = height
        self.res = res

        # bottom left corner is 0, 0 both in pixels and in meters
        self.robots = robots        # initialized w/ array of Robot objects
        self.meas_func = np.zeros((len(robots)))
        self.dist = np.zeros((2, len(robots)))

        # define the points you're iterating over
        self.pointsx = np.arange(0, width, res)
        self.pointsy = np.arange(0, height, res)

        self.alpha = alpha
        self.sigma = sigma
        self.cov = cov

        self.target = target


    # calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
    def mix_func(self, point, value=1):   

        # Get list of all the costs for every robot
        f_lists = np.array([self.f(robot.state, point) for robot in self.robots])
        #print(f_lists)
        #if self.alpha < 0 and (np.any(f_lists == 0) or np.isnan(f_lists).any()): 
            #print("entered if", [robot.state for robot in self.robots], point)
            #return 1

        # Apply power of alpha coeffient
        raised_f = f_lists ** self.alpha

        # Compute mixing function as (sum of the f_i^alpha )^(1/alpha)
        g_alpha = np.sum(raised_f) ** (1/self.alpha)
        return g_alpha

    def f(self, p,q):
        """
        Cost of sensing, or servicing function
        Computes the cost of a robot in state p (point position) for sensing in point q of the bounded region Q. 
        Parameters
        ----------
        p:
        """
        f_try = np.sqrt(sum((q - p)**2))  
        # We define the distance to be the euclidean distance between the 2 points
        
        
        return f_try 


    def phi(self,q):
        """
        Weighting of importance function over the region Q (Assumed to be constant for question a)
        """
        return 1

    def update_gradient(self, iter = 0):
        rv = None
        if(type(self.target) is np.ndarray):
            rv =  multivariate_normal(mean = self.target[:, iter], cov = self.cov)
        else:
            rv =  multivariate_normal(mean = self.target[-1], cov = self.cov)

        # for x in self.pointsx:
        #     for y in self.pointsy:
        #         value = 1
        #         value = rv.pdf((x,y))

        #         self.mix_func(np.array([x, y]), value)


        #TODO Add checks for alpha and undefined variables


        
        # Accumulator variable for individual partial sums (each partial sum is a 2D array)
        robot_partial_sums = [[] for robot in env.robots]
            
        ps = []
        for i, robot in enumerate(self.robots):
            ps.append(robot.state)
        # Perform integral over Q space 
        # (Space Q is modelled as discrete, hence a summation, over all the points is used to approximate the integral)
        
        
        for x in self.pointsx:
            for y in self.pointsy:
                for i, robot in enumerate(self.robots):
                # Define point as a tuple of 2 coordinates
                    q = np.array([x, y])
                    value = 1
                    # value = rv.pdf((x,y)) 
                    g_alpha = self.mix_func(q, value=value)
                
                    f_try = self.f(robot.state, q)
                    if ps[i][0] == x and ps[i][1] == y: 
                        f_try = 1 
                    
                    partial_sum = ((f_try/g_alpha) ** (self.alpha-1)) * (q - robot.state) * value
                    
                    robot_partial_sums[i].append(partial_sum)         

            # Compute individual robot gradient p_i_dot as the sum of all of the partial sums
        for i, robot in enumerate(self.robots):
            sum_var = np.sum(robot_partial_sums[i], axis=0)
            #print("sum var", sum_var)
            robot.input = sum_var #if not np.isnan(sum_var).any() else 0, 0 
            
       

    def moves(self):
        for bot in self.robots:
            bot.update()




    # def display_grid(self):

    #     grid = [[set() for i in range(self.width)] for i in range(self.height)]
    #     for b in self.bots:
    #         grid[b.x][b.y].add(b)
	

    #     # prints grid with number of bots in each coordinate location
    #     print("Grid["+("%d" %self.size)+"]["+("%d" %self.size)+"]")
    #     for j in range(self.size-1,-1,-1):
    #         print(j ,'|', end =" ")
    #         for i in range(0,self.size):
    #             print(len(self.grid[i][j]), end ="  ") # switched from i j 
    #         print()
        
    #     print("--", end="  ")
    #     for i in range(0,self.size):
    #         print("-", end ="  ")
    #     print()

    #     print("  ", end="  ")
    #     for i in range(0,self.size):
    #         print(i, end ="  ")
    #     print()
        



# function to run the simulation
def run_grid(env, iter):
    x = []
    y = []

    # initialize state
    for i, bot in enumerate(env.robots):

        x.append([bot.state[0]])
        y.append([bot.state[1]])

    # run environment for iterations
    for k in range(iter):
        print(k)
        #env.target.append(target(k))
        env.update_gradient(k)
        env.moves()

        for i, bot in enumerate(env.robots):

            x[i].append(bot.state[0])
            y[i].append(bot.state[1])

        if (k % 50 == 0):
            print(k)

    # set up the plot
    fig, ax= plt.subplots()
    points = []

    # plt the robot points

    for i in range(len(env.robots)):
        print(x[i], y[i])
        ax.scatter(x[i], y[i], alpha=(i+1)/len(env.robots), label = "robot " + str(i)) 
        points.append([x[i][-1], y[i][-1]])
       
    # if there is a target setup plot it
    # if(type(env.target) is np.ndarray):
    #     for i in range(env.target.shape[1]):
    #         plt.scatter(env.target[0, i], env.target[1, i], alpha=(i+1)/env.target.shape[1])

    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    ax.plot(b_x, b_y)  
    target_x, target_y = [x for x, _ in env.target], [y for _, y in env.target]
    ax.plot(target_x, target_y, marker="x", color = "red")

    # set Voronoi
    vor = Voronoi(np.array(points))
    voronoi_plot_2d(vor, ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))
    plt.legend() 
    plt.show()
    
# generate target points
def target(iter):

    r = 3 
    prd = 800
    x = r * np.cos(2 * np.pi * iter/prd) + 5
    y = r * np.sin(2 * np.pi * iter/prd) + 5
    return np.array([x, y])

if __name__ == "__main__":

    rob1 = Robot([4, 1])
    rob2 = Robot([2, 2])
    rob3 = Robot([5, 6])
    rob4 = Robot([3, 4])
    robots = [rob1, rob2, rob3, rob4]

    env = Environment(10, 10, 0.1, robots, alpha = -10)

    #env = Environment(1, 1, 0.2, robots)
    #env = Environment(10, 10, 0.1, robots, target=(5,5))


    run_grid(env, 200)
