import numpy as np
import matplotlib.pyplot as plt

# nodes for storing information
class Node(object):

    def __init__(self, init_state):

        self._prev_state = init_state
        self._next_state = init_state

    # store the state update
    def update(self, update):
        self._next_state += update

    # push the state update
    def step(self):
        self._prev_state = self._next_state

    @property
    def state(self):
        return self._prev_state


# adversarial node
class AdversaryNode(object):

    def __init__(self, init_state, target, epsilon_adv = 0.01):

        self._prev_state = init_state
        self._next_state = init_state
        
        self._target = target
        self._epsilon = epsilon_adv

    # store the state update
    def update(self, update):
    
        raise NotImplementedError

    # push the state update
    def step(self):
        self._prev_state = self._next_state

    @property
    def state(self):
        return self._prev_state


# Graph for connecting nodes
class Graph(object):

    def __init__(self, node_list, adj_matrix, epsilon = 0.2, threshold = 0, sigma = 0):

        self.node_list = node_list
        self.adj_matrix = adj_matrix


        self._epsilon = epsilon

        self._finished = False      # bool determining when we've reached a threshold
        self._threshold = threshold
        self._sigma = sigma

    # update the graph
    def update_graph(self):

        raise NotImplementedError

    # return the state of the nodes currently - you can disable print here
    def node_states(self):
        string = ""
        out = []
        for node in self.node_list:
            string = string + node.state.astype('str') + "\t"
            out.append(node.state)
        # print(string)

        return out

    # check if the graph has reached consensus somehow, even if there are adversaries
    def is_finished(self):
        
        raise NotImplementedError

    @property
    def finished(self):
        # add your code here
        return self._finished


# return a random adjacency matrix
def rand_adj(dim, p):

    raise NotImplementedError

# return the Fiedler value to show strong connection of the array
def fiedler(adj_mat):

    raise NotImplementedError


# plots the development of node values in the consensus problem over time
def plot_states(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)
    
    plt.legend()
    plt.show()


if __name__ == "__main__":

    node_list = [Node(4.0), Node(2.0), Node(-1.0), Node(3.0), Node(0.0)]

    # linear formation
    adj_matrix = np.array([[0, 1, 0, 0, 0],
                            [1, 0, 1, 0, 0],
                            [0, 1, 0, 1, 0],
                            [0, 0, 1, 0, 1],
                            [0, 0, 0, 1, 0]])

    # circular formation
    # adj_matrix = np.array([[0, 1, 0, 0, 1],
    #                         [1, 0, 1, 0, 0],
    #                         [0, 1, 0, 1, 0],
    #                         [0, 0, 1, 0, 1],
    #                         [1, 0, 0, 1, 0]])

    # fully connected
    # adj_matrix = np.array([[0, 1, 1, 1, 1],
    #                         [1, 0, 1, 1, 1],
    #                         [1, 1, 0, 1, 1],
    #                         [1, 1, 1, 0, 1],
    #                         [1, 1, 1, 1, 0]])

    graph = Graph(node_list, adj_matrix)
    node_states = []
    for _ in range(10):
        graph.update_graph()
        node_states.append(graph.node_states())

    node_states = np.array(node_states)
    plot_states(node_states)
    


