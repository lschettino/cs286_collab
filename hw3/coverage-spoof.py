import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry.polygon import Polygon
from scipy.spatial import Voronoi, voronoi_plot_2d

class Server(object):

    def __init__(self, state, k=0.1):
        self._state = state     # 2-vec
        self.input = [0,0]      # movement vector late
        self.k = k * 0.001

    def update(self):     # update the robot state
        self._state += self.k * self.input
        self.input = [0, 0]

    @property
    def state(self):
        return np.array(self._state)

class Client(object):

    def __init__(self, state, spoofer = False):
        self.state = state      # 2-vec
        self.spoofer = spoofer  # if true, then client is a spoofer; else, client is legitimate.
        self.alpha = None       # confidence metric (Gil et al. Section 3); to be assigned by via the environment
        self.rho = None         # individual contribution to importance function (Gil et al.)

class Environment(object):

    def __init__(self, width, height, res, servers, clients, spoof_mean = 0.1, legit_mean = 0.8, alpha = -10):       
        # width and height are in pixels, so actual dimensions are width * res in meters
        # bottom left corner is 0, 0 both in pixels and in meters
        self.width = width
        self.height = height
        self.res = res

        self.servers = servers        # initialized w/ array of Server objects
        self.clients = clients        # initialized w/ array of Client objects

        self.spoof_mean = spoof_mean  # mean of distribution from which legitimate clients sample
        self.legit_mean = legit_mean  # mean of distribution from which spoofed clients sample
        
        self.meas_func = np.zeros((len(servers)))
        self.dist = np.zeros((2, len(servers)))

        # define the points you're iterating over
        self.pointsx = np.arange(0, width, res)
        self.pointsy = np.arange(0, height, res)

        self.alpha = alpha # "free parameter" from Schwager et al., Section 2.2

    def define_rho(self):
        raise NotImplementedError

    def sample_alphas(self, spoof_mean, legit_mean):
        # create local variable here to note which clients are spoofers!
        
        raise NotImplementedError

    def mix_func(self, point, value = 1):     
        # calc the mixing function for the function aka g_alpha, also record f(p, q) and dist, point is np.array([x,y])
        for i, bot in enumerate(self.servers):
            self.dist[:, i] = point - bot.state
            self.meas_func[i] = 0.5 * np.linalg.norm(self.dist[:, i])**2

        mixing = np.sum(self.meas_func[self.meas_func > 0]**self.alpha)**(1/self.alpha)

        for i, bot in enumerate(self.servers):
            if(self.meas_func[i] > 0):
                bot.input += (self.meas_func[i] / mixing)**(self.alpha - 1) * self.dist[:, i] * value

    def update_gradient(self):
        for i in self.pointsx:
            for j in self.pointsy:
                self.mix_func(np.array([i, j]), 1)

    def moves(self):
        for bot in self.servers:
            bot.update()



# function to run the simulation
def run_grid(env, iter):
    x = []
    y = []

    # set polygon bounds
    bounds = Polygon([(0,0), (10,0), (10,10), (0, 10)])
    b_x, b_y = bounds.exterior.xy
    plt.plot(b_x, b_y)        

    # initialize state
    env.define_rho()

    for i, bot in enumerate(env.servers):
        x.append([bot.state[0]])
        y.append([bot.state[1]])

    # run environment for iterations
    for k in range(iter):
        env.sample_alphas(env.spoof_mean, env.legit_mean)
        env.update_gradient()
        env.moves()

        for i, bot in enumerate(env.servers):
            x[i].append(bot.state[0])
            y[i].append(bot.state[1])

        if (k % 50 == 0):
            print(k)

    # set up the plot
    fig, ax = plt.subplots()
    points = []

    # plt the server points
    plt.axes(ax)
    for i in range(len(env.servers)):
        plt.scatter(x[i], y[i], alpha=(i+1)/len(env.servers))
        points.append([x[i][-1], y[i][-1]])

    # plot the client points
    for c in env.clients:
        if c.spoofer:
            plt.scatter(c.state[0], c.state[1], alpha=0.1)
        else:
            plt.scatter(c.state[0], c.state[1], alpha=0.9)

    # set Voronoi
    # vor = Voronoi(np.array(points))
    # voronoi_plot_2d(vor, show_vertices = False, line_colors='blue', ax=ax)
    
    ax.set_xlim((-1, 11))
    ax.set_ylim((-1, 11))


    plt.show()

if __name__ == "__main__":

    serv1 = Server([4, 1])
    serv2 = Server([2, 2])
    # serv3 = Server([5, 6])
    # serv4 = Server([3, 4])
    servers = [serv1, serv2, serv3, serv4]

    client1 = Client([3,3], True)
    client2 = Client([1,1])
    client3 = Client([9,9])
    client4 = Client([8,3], True)
    clients = [client1, client2, client3, client4]

    env = Environment(10, 10, 0.1, servers, clients)

    run_grid(env, 5)