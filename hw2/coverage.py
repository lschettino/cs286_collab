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

        raise NotImplementedError

    def update_gradient(self, iter = 0):
        # rv = None
        # if(type(self.target) is np.ndarray):
        #     rv =  multivariate_normal(mean = self.target[:, iter], cov = self.cov)
        # else:
        #     rv =  multivariate_normal(mean = self.target, cov = self.cov)

        for x in self.pointsx:
            for y in self.pointsy:
                value = 1
                # value = rv.pdf((x,y))

                self.mix_func(np.array([x, y]), value)

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

    # run environment for iterations
    for k in range(iter):
        env.update_gradient(k)
        env.moves()

        for i, bot in enumerate(env.robots):

            x[i].append(bot.state[0])
            y[i].append(bot.state[1])

        if (k % 50 == 0):
            print(k)

    # set up the plot
    fig, ax = plt.subplots()
    points = []

    # plt the robot points
    plt.axes(ax)
    for i in range(len(env.robots)):
        plt.scatter(x[i], y[i], alpha=(i+1)/len(env.robots))
        points.append([x[i][-1], y[i][-1]])
    

    # if there is a target setup plot it
    if(type(env.target) is np.ndarray):
        for i in range(env.target.shape[1]):
            plt.scatter(env.target[0, i], env.target[1, i], alpha=(i+1)/env.target.shape[1])

    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    plt.plot(b_x, b_y)        

    # set Voronoi
    vor = Voronoi(np.array(points))
    voronoi_plot_2d(vor, ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))

    plt.show()
    
# generate target points
def target(iter):

    raise NotImplementedError

if __name__ == "__main__":

    rob1 = Robot([4, 1])
    rob2 = Robot([2, 2])
    rob3 = Robot([5, 6])
    rob4 = Robot([3, 4])
    robots = [rob1, rob2, rob3, rob4]

    # env = Environment(10, 10, 0.1, robots)
    # env = Environment(10, 10, 0.1, robots, target=(5,5))


    run_grid(env, 5)