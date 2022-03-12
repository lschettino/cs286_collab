import numpy as np
import matplotlib.pyplot as plt
from scipy.sparse import csgraph

# nodes for storing information
class Node(object):

    def __init__(self, init_state):

        self._prev_state = init_state
        self._next_state = init_state

    # store the state update
    def update(self, update):
        # Adding the update instead of setting equal 
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
        #Check if the epilson is negative to adjust when to stop updating 
        if self._epsilon > 0:
            if self._prev_state < self._target:
                self._next_state += self._epsilon
        else:
            if self._prev_state >= self._target:
                self._next_state += self._epsilon

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
        self._sigma = sigma ** 0.5
        self._consensus = 0

    # update the graph
    def update_graph(self):
        # Loop through each node in the grpah
        for node in self.node_list:
            # If node is an adversary just push the update
            if isinstance(node, AdversaryNode):
                node.update(0)
            else:
                #Initialize value to add to node state
                val = 0.0 
                # Loop through other nodes in graph
                for node2 in self.node_list:
                    # Get the edge value from the adjacency, used for when weights are doubled 
                    aij = self.adj_matrix[self.node_list.index(node)][self.node_list.index(node2)]
                    # B) Generate noise for reported values
                    noise = np.random.normal(0,self._sigma)
                    noise2 = np.random.normal(0,self._sigma)
                    #Implement consensus equation with noise for reported state
                    val += aij * ((node2.state + noise2) - (node.state + noise))
                val *= self._epsilon

                # Add value to current node state
                node.update(val)

        #After all next states are updated, push to previous state 
        for node in self.node_list:
            node.step()

        # G) Change the adjacency matrix randomly 
        self.adj_matrix = rand_adj(len(self.adj_matrix), 0.1)


    # return the state of the nodes currently - you can disable print here
    def node_states(self):
        string = ""
        out = []
        for node in self.node_list:
            string = string + str(node.state) + "\t"
            out.append(node.state)
        # print(string)
        return out

    # check if the graph has reached consensus somehow, even if there are adversaries
    def is_finished(self, node_states):
        # whether or not to stop iteration 
        con_bool = True

        for node in self.node_list:
            # Apply a threshold for consensus value to allow for small diifferences
            if node.state >= self._consensus + self._threshold or node.state <= self._consensus - self._threshold:
                con_bool = False
        return con_bool

    # Helper function to get consensus value
    def get_consensus_value(self):
        consensus = 0
        # Consensus value calculated as the average of the initial states of the nodes
        for node in self.node_list:
            consensus += node.state
        consensus /= len(self.node_list)
        self._consensus = consensus


    @property
    def finished(self):
        # add your code here
        return self._finished


# return a random adjacency matrix
def rand_adj(dim, p):
    #Initialize Matrix
    adj = np.zeros([dim, dim])
    # Loop for length of the dimension
    for i in range(dim):
        for j in range(dim):
                if j>i:
                    # Randomly add an edge
                    if np.random.uniform() <= p:
                        adj[i][j] = 1
                        adj[j][i] = 1
                    else:
                        adj[i][j] = 0
    return adj

# return the Fiedler value to show strong connection of the array
def fiedler(adj_mat):
    #Laplacian function from scipy
    lap = csgraph.laplacian(adj_mat)
    # Get the eigenvalues 
    eigs = np.linalg.eigvals(lap)
    #Sort the eigenvalues from smallest to largest
    eigs.sort()
    #Fiedler value is the second smallest 
    return eigs[1]


# plots the development of node values in the consensus problem over time
def plot_states(node_states):

    steps = np.arange(len(node_states))

    _, ax = plt.subplots()
    for i in range(node_states.shape[1]):
        line, = ax.plot(steps, node_states[:, i])
        line.set_label(i)
    plt.legend()
    # plt.xticks(steps)
    plt.show()


if __name__ == "__main__":
    node_list = [Node(4.0), Node(2.0), Node(-1.0), Node(3.0), Node(1.0)]
    # node_list = [Node(4.0), Node(2.0), Node(-1.0), Node(3.0), AdversaryNode(1.0, 2)]


    # linear formation
    # adj_matrix = np.array([[0, 1, 0, 0, 0],
    #                         [1, 0, 1, 0, 0],
    #                         [0, 1, 0, 1, 0],
    #                         [0, 0, 1, 0, 1],
    #                         [0, 0, 0, 1, 0]])

    # circular formation
    adj_matrix = np.array([[0, 1, 0, 0, 1],
                            [1, 0, 1, 0, 0],
                            [0, 1, 0, 1, 0],
                            [0, 0, 1, 0, 1],
                            [1, 0, 0, 1, 0]])

    # fully connected
    # adj_matrix = np.array([[0, 1, 1, 1, 1],
    #                         [1, 0, 1, 1, 1],
    #                         [1, 1, 0, 1, 1],
    #                         [1, 1, 1, 0, 1],
    #                         [1, 1, 1, 1, 0]])


    # linear double weight
    # adj_matrix = np.array([[0, 2, 0, 0, 0],
    #                         [2, 0, 2, 0, 0],
    #                         [0, 2, 0, 2, 0],
    #                         [0, 0, 2, 0, 2],
    #                         [0, 0, 0, 2, 0]])



    # circular formation double weight
    # adj_matrix = np.array([[0, 2, 0, 0, 2],
    #                         [2, 0, 2, 0, 0],
    #                         [0, 2, 0, 2, 0],
    #                         [0, 0, 2, 0, 2],
    #                         [2, 0, 0, 2, 0]])


    # fully connected double weight
    # adj_matrix = np.array([[0, 2, 1, 1, 2],
    #                         [2, 0, 2, 1, 1],
    #                         [1, 2, 0, 2, 1],
    #                         [1, 1, 2, 0, 2],
    #                         [2, 1, 1, 2, 0]])


    #test 1
    # adj_matrix = np.array([[0, 1, 1, 0, 0],
    #                         [1, 0, 1, 1, 1],
    #                         [1, 1, 0, 1, 1],
    #                         [0, 1, 1, 0, 0],
    #                         [0, 1, 1, 0, 0,]])

    #test 2
    # adj_matrix = np.array([[0, 0, 1, 0, 0],
    #                         [0, 0, 0, 0, 1],
    #                         [1, 0, 0, 1, 1],
    #                         [0, 0, 1, 0, 1],
    #                         [0, 1, 1, 1, 0]])

    #test 3
    # adj_matrix = np.array([[0, 1, 0, 1, 0],
    #                         [1, 0, 0, 1, 1],
    #                         [0, 0, 0, 0, 1],
    #                         [1, 1, 0, 0, 1],
    #                         [0, 1, 1, 1, 0]])

    # print(fiedler(adj_matrix))

    graph = Graph(node_list, adj_matrix, epsilon = 0.2, threshold = 0.005, sigma = 0)
    # Calculate the consensus value
    graph.get_consensus_value()
    node_states = [graph.node_states()]
    #Check for consensus 
    while graph.is_finished(node_list) == False:
    #Loop used for when consensus cannot be reached 
    # for i in range(20):
        graph.update_graph()
        node_states.append(graph.node_states())
    print(graph._consensus)
    node_states = np.array(node_states)
    plot_states(node_states)
    


