import numpy as np
import matplotlib.pyplot as plt

# this returns a bunch of weights - sets a mean alpha level, then randomly generates alphas around the means
class AlphaWeights(object):

    def __init__(self, mean_alpha, std = 0.1):

        self.std = std
        self.iter = 1

        # assuming fully connected - this is an adjancency matrix
        # assume the vector is [legitimate, spoofed], assume it's diagraph
        self.means = mean_alpha
        self.betas = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

    # Calculate the updates fo the betas used in updating W
    # be sure to cap alpha at -0.5 and 0.5 at all times
    # this function should generate a random set of alphas based on the means and then update beta accordingly
    def update_betas(self):

        alpha = self.means + np.random.randn(self.means.shape[0], self.means.shape[1]) * self.std

        # cap alphas at 0.5 because thats the way it is
        thresh = alpha > 0.5
        alpha[thresh] = 0.5
        thresh = alpha < -0.5
        alpha[thresh] = -0.5

        # update beta because it's a running sum of alphas
        self.betas += alpha
        
        self.iter += 1


# define the simulation environment
class Environment(object):

    def __init__(self, leg, spoof, Theta, Omega):

        self.leg = leg         # legitimate nodes n_l
        self.spoof = spoof     # spoofed nodes n_s

        self.leg_len = leg.shape[0]
        self.full_len = leg.shape[0] + spoof.shape[0]

        self.Theta = Theta              # transition for spoofed based on leg - n_s x n_l   (row, col)
        self.Omega = Omega              # transition for spoofed based on spoofed - n_s x n_s

        # transition for legitimate based on spoof and leg - n_l x (n_l + n_s)
        # first n_l columns are the legit part W_L
        self.W = np.zeros((self.leg_len, self.full_len))

    # updates according to the system dynamics given
    def update(self):
        self.leg = np.matmul(self.W, np.concatenate((self.leg, self.spoof), axis=0))
        self.spoof = np.matmul(self.Theta, self.leg) + np.matmul(self.Omega, self.spoof)


    # set the transitions of W
    # call alphaweights to get an updated beta, then use that to update W.
    # the code will call update to progress the simulation
    def transition_W(self, alphaweights):
        #tempW = self.W.copy()
        for i in range(self.leg_len):
            for j in range(self.full_len):
                beta = alphaweights.betas[i][j]
                if beta >= 0:
                    middle = 1 - np.exp(-beta / 2)
                    self.W[i][j] = middle / self.leg_len 
                else:
                    middle = np.exp(beta)
                    self.W[i][j] = middle / (2 * self.leg_len)
                    
        # overwrite 
        for i in range(self.leg_len):
            self.W[i][i] += (1 - self.W[i].sum(axis=0))
        

# it plots the states - basically the same function from HW 2
def plot_states(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)

    # plot the average
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]))
    line.set_label("average")

    
    plt.legend()
    plt.show()

def plot_states_2(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)

    # plot the average
    line, = ax.plot(steps, np.ones(len(steps)) * np.average(node_states[0, :]))
    line.set_label("average")

    
    plt.legend()
    plt.show()



if __name__ == "__main__":

    # assume everything is in 1-D and fully connected
    
    leg = np.array([1, 2, 1.1, 1.9, 1.4, 2.3, 0.7, 2,1,2,1,2,1,0.5,0.8,1.5,1,2,1,2])
    spoof = np.array([4., 4., 4., 4.])
    
    # Uncomment for part 2b 
    # np.random.seed(286) # for reproducible results 
    # mean = 1.5
    # std = 0.1 # also change for 1, 3, 5
    # leg = std * np.random.randn(20) + mean 
    
    
    # Setting up a new matrix of inputs for the dynamics
    alphas = np.ones((leg.shape[0] + spoof.shape[0], leg.shape[0] + spoof.shape[0]))
    alphas = 0.4 * alphas
    alphas[:, -spoof.shape[0]:] = -1 * alphas[:, -spoof.shape[0]:] 

    alphas = AlphaWeights(alphas)

    theta = np.zeros((spoof.shape[0], leg.shape[0]))
    omega = np.eye(spoof.shape[0])

    # Define the environment
    env = Environment(leg, spoof, theta, omega)

    iter = 50

    # Run the simulation and plot
    leg_states = []
    leg_states.append(env.leg)
    spoof_states = [] 
    spoof_states.append(env.spoof)
    
    curr_spoof_val = 4.
    
    for i in range(iter):
        # Uncomment for part 2e 
        # Uncomment for sin pattern 
        # sin_spoof = np.sin((2. * np.pi * i / 50.) + 10) + 4.
        # env.spoof.fill(sin_spoof)
        
        # Uncomment for exponential pattern
        # env.spoof.fill(curr_spoof_val * (1.01))
        # curr_spoof_val *= 1.01 
        
        alphas.update_betas()
        env.transition_W(alphas)        # update W at every iteration
        env.update()

        leg_states.append(env.leg)
        spoof_states.append(env.spoof)
    print(leg_states[-1])
    plot_states(np.array(leg_states))
    
    # Uncomment to plot graphs for 2b, 2c, 2d 
    # std_devs = [0.1, 1, 3, 5]
    # final_vals = [1.83653285, 1.87401386, 1.95730544, 2.04059575]
    
    
    # _, ax = plt.subplots()
    # line = ax.scatter(std_devs, final_vals)

    # plt.xlabel("Standard Deviations")
    # plt.ylabel("Final Convergence Values")
    # plt.title("Comparing Converged Value for Standard Deviation")
    # plt.show()
    
    # num_spoofer = [2, 4, 8, 20]
    # final_vals = [1.70887816, 1.86829893, 2.16990608, 2.83404374]

    # _, ax = plt.subplots()
    # line = ax.scatter(num_spoofer, final_vals)

    # plt.xlabel("Number of Spoofers")
    # plt.ylabel("Final Convergence Values")
    # plt.title("Comparing Converged Value for Number of Spoofers")
    # plt.show()

    


    

    