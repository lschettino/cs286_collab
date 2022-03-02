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
        
        self._state += self.k * self.input
        
        self._stoch_state = self._state + np.random.randn(2)
        self.input = [0, 0]

    @property
    def state(self):
        return np.array(self._state)
    @property
    def stoch_state(self):
        return np.array(self._stoch_state)

    

class Environment(object):

    def __init__(self, width, height, res, robots, alpha = -10, sigma = 0, cov = 5, target = [], moving_target = False):       # width and height are in pixels, so actual dimensions are width * res in meters
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
        
        self.moving_target = moving_target


    # calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
    def mix_func(self, point, value=1):   

        # Get list of all the costs for every robot
        f_lists = np.array([self.f(robot.state, point) for robot in self.robots])
        
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
        #f_try = np.sqrt(sum((q - p)**2))
        f_try = sum((q - p)**2) * 0.5
        #f_try = 0.5 * np.abs((q - p))**2)
        # We define the distance to be the euclidean distance between the 2 points
        
        
        return f_try 


    

    def update_gradient(self, iter = 0):
    
        if self.target == []: 
            phi = lambda x, y : 1 
        else: 
            rv = None
            if(type(self.target) is np.ndarray):
                rv =  multivariate_normal(mean = self.target[iter], cov = self.cov)
            else:
                
                rv =  multivariate_normal(mean = self.target, cov = self.cov)
            
            phi = lambda x, y : rv.pdf((x, y))
       

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
                    
                    g_alpha = self.mix_func(q)
                    value = phi(x, y)
                    
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





# function to run the simulation
def run_grid(env, iter):
    x = []
    y = []

    # initialize state
    for i, bot in enumerate(env.robots):

        x.append([bot.state[0]])
        y.append([bot.state[1]])
   
    if env.moving_target: 
        env.target = target(iter)
   
    # run environment for iterations
    for k in range(iter):
        print(k)
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
        ax.scatter(x[i], y[i], alpha=(i+1)/len(env.robots), label = "robot " + str(i)) 
        points.append([x[i][-1], y[i][-1]])
    
  
    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    ax.plot(b_x, b_y)  
    
    
    
    # logic for plotting target (if there is one)
    if len(env.target) > 0: # checking whether target is empty 
        print(type(env.target))
        if isinstance(env.target, np.ndarray): # cheking whether target is list of moving targets 
            target_x, target_y = [x for x, _ in env.target], [y for _, y in env.target]
            for i in range(len(env.target)): 
                ax.plot(target_x[i], target_y[i], color="blue", marker='o', alpha=(i + 1)/len(env.target))
        else: # just a single target 
            target_x, target_y = env.target
            ax.plot(target_x, target_y, color="red", marker="x")
        
        
    # set Voronoi for coverage problems 
    if env.alpha < -1: 
        vor = Voronoi(np.array(points))
        voronoi_plot_2d(vor, ax=ax)
        
    
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))
    ax.set_title("Trajectory plots: alpha = 1, k = 0.1, \n constant importance, iter = 200")
    # "Trajectory plots: alpha = -10, k = 1, \n constant importance values, iter = 200"
    # "alpha = -10, k = 1, single target (5,5), iter = 200"
    # "alpha = -10, k = 1, constant importance, iter = 200 with stochastic noise"
    plt.legend() 
    plt.savefig("./graphs/exp_k0.1_a1.png")
    
# generate target points
def target(iter):

    r = 3 
    prd = 800
    lst = [] 
    
    for i in range(iter): 
        x = r * np.cos(2 * np.pi * i/prd) + 5
        y = r * np.sin(2 * np.pi * i/prd) + 5
        lst.append((x, y))
    return np.array(lst)

if __name__ == "__main__":

    rob1 = Robot([4, 1],k = 0.1)
    rob2 = Robot([2, 2],k = 0.1)
    rob3 = Robot([5, 6],k = 0.1)
    rob4 = Robot([3, 4],k = 0.1)
    robots = [rob1, rob2, rob3, rob4]

    env = Environment(10, 10, 0.1, robots, alpha = 1)

    #env = Environment(1, 1, 0.2, robots)
    # env = Environment(10, 10, 0.1, robots, alpha = 0.9)


    run_grid(env, 200)
