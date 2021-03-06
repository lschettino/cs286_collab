import numpy as np
import copy, sys, itertools, os,csv
from base_policy import base_policy
sys.path.append('../util/')
from graphics import plot_trajectory, visualize_requests

class standard_rollout:
    def get_cartesian_product(self, list_of_all_control_component_sets):
        # returns list of controls where each element comes from a cartesian product
        # for example if list_of_all_control_component_sets = [[1, 2, 3],[4,5,6]]
        # the function should return [(1, 4), (1, 5), (1, 6), (2, 4), (2, 5), (2, 6), (3, 4), (3, 5), (3, 6)]
        ################################# Begin your code ###############################

        return None
        ################################# End your code ###############################


    def expectation_for_minimization(self, taxi_state_object, control):
        expected_cost = 0
        for num_simu in range(10):
            state_object = copy.deepcopy(taxi_state_object) #use state_object for further operations inside this function
            # update expected_cost using the 1) the next_state with the given control 2) future cost using the average_MC_simulation using base policy
            ################################# Begin your code ###############################

            ################################# End your code ###############################
        expected_cost /= 10.0
        return expected_cost

    def minimization_of_expectations(self, control_cost):
        # For the single agent case, if control_cost = {(0,): 23.64, (1,): 23.99, (2,): 24.92, (4,): 25.89}
        # This funtion should return (0,)
        # For the 2 agents case, if control_cost = {(0, 0): 23.47, (0, 2): 22.35, (0, 4): 20.11, (1, 0): 21.59, (1, 2): 17.96, (1, 4): 19.27}
        # This funtion should return (1, 2)
        ################################# Begin your code ###############################

        return 0
        ################################# End your code ###############################

    def get_control(self, taxi_state_object):
        #print('get_control() standard_rollout')
        min_control = 0
        list_of_all_control_component_sets = [[] for ell in range(taxi_state_object.g.m)]
        control_cost = {}  # A dictionary with <key: control, value: expected sum of the stage cost and expected future cost>
        # Populate list_of_all_control_component_sets using available_control_agent for all agents
        # Create cartesian product from list_of_all_control_component_sets
        # Populate control_cost using expectation_for_minimization for all controls given by the cartesian product
        # return min_control which is the minimizing control in control_cost dictionary
        ################################# Begin your code ###############################

        ################################# End your code ###############################
        return min_control


class one_at_a_time_rollout:

    def expectation_for_minimization(self, taxi_state_object, control):
        expected_cost = 0
        for num_simu in range(10):
            state_object = copy.deepcopy(taxi_state_object) #use state_object for further operations inside this function
            # update expected_cost using the 1) the next_state with the given control 2) future cost using the average_MC_simulation using base policy
            ################################# Begin your code ###############################

            ################################# End your code ###############################
        expected_cost /= 10.0
        return expected_cost

    def minimization_of_expectations(self, control_cost):
        # if control_cost = {0: 21.7, 1: 23.13, 2: 18.02, 3: 19.86, 4: 19.04}
        # The function should return 2
        ################################# Begin your code ###############################

        return 0
        ################################# End your code ###############################

    def get_control(self, taxi_state_object):
        one_at_a_time_rollout_control = None
        # construct one_at_a_time_rollout_control sequentially from agent 0 to taxi_state_object.g.m-1
        # for an agent ell set the constructed_control using the base policy and for all agents from ell+1 to taxi_state_object.g.m-1
        # and set the constructed_control using one_at_a_time_rollout_control and for all agents from 0 to ell-1
        # Find all controls for agent ell using available_control_agent
        # Populate a dictionary where <key: agent ell's availble control, corresponding stage and future cost using expectation_for_minimization
        # Set the minimizing control component for agent ell in one_at_a_time_rollout_control from the dictionary
        ################################# Begin your code ###############################

        ################################# End your code ###############################
        return one_at_a_time_rollout_control



def MC_simulation(taxi_state_object, policy_name='base_policy', do_print=True):
    state_object = copy.deepcopy(taxi_state_object)
    trajectory_cost = 0
    if (policy_name == 'standard_rollout_policy'):
        policy_ = standard_rollout()
    elif (policy_name == 'one_at_a_time_rollout'):
        policy_ = one_at_a_time_rollout()
    else:
        policy_ = base_policy()

    init_time = taxi_state_object.time
    trajectory_string = state_object.print_state() + "\n"
    for time in range(init_time, taxi_state_object.g.N):
        control = policy_.get_control(state_object)
        trajectory_cost += state_object.next_state(control)
        if do_print:
            trajectory_string += state_object.print_state() + "\n"
            #print(control)
            #state_object.print_state()
            None
    if (do_print):
        print('Inside MC_simulation: Trejectory')
        print(trajectory_string)
        print('trajectory_cost: ',trajectory_cost)
        #print('Inside MC_simulation: trajectory_cost ',trajectory_cost)
        #exit()
        None
    return trajectory_cost, trajectory_string

def average_MC_simulation(taxi_state_object, policy_name='base_policy', do_print=True):
    trajectory_cost_average = 0
    for num_simu in range(10):
        trajectory_cost_, _ = MC_simulation(taxi_state_object, policy_name, do_print)
        trajectory_cost_average += trajectory_cost_#+term_cost
    trajectory_cost_average /=10.0
    if (do_print):
        print('Inside average_MC_simulation: average_MC_simulation ', trajectory_cost_average)
    return trajectory_cost_average


def Run_a_trajectory(taxi_state_object, policy_name='base_policy'):
    state_object = copy.deepcopy(taxi_state_object)
    trajectory_cost = 0
    if (policy_name == 'standard_rollout_policy'):
        policy_ = standard_rollout()
    elif (policy_name == 'one_at_a_time_rollout'):
        policy_ = one_at_a_time_rollout()
    else:
        policy_ = base_policy()

    init_time = taxi_state_object.time
    all_requests = read_saved_requests(taxi_state_object)
    trajectory_string = state_object.print_state() + "\n"
    #print(trajectory_string)
    for time in range(init_time, taxi_state_object.g.N):
        control = policy_.get_control(state_object)
        trajectory_cost += state_object.next_state(control, all_requests[time+1])
        trajectory_string += state_object.print_state() + "\n"
        #print(trajectory_string)
    term_cost = state_object.terminal_cost()

    return trajectory_cost, trajectory_string, term_cost

def create_and_save_requests(taxi_state_object):
    foldername = os.getcwd() + '/../data/trajectory_file/'
    if not os.path.exists(foldername):
        try:
            os.mkdir(foldername)
        except:
            print('could not create', foldername)
    f = open(os.getcwd() + '/../data/trajectory_file/requests.csv', 'w')
    horizon = taxi_state_object.g.N
    all_new_requests = [[] for time in range(horizon)]
    for time in range(horizon):
        new_requests = taxi_state_object.sample_new_requests()
        for request in new_requests:
            f.write(str(time) + ',' + str(request[0]) + ',' + str(request[1])+ ',' + str(request[2])+ ',' + str(request[3]) + '\n')
            all_new_requests[time].append((request[0],request[1]))
    #print(all_new_requests)
    visualize_requests(all_new_requests)
    f.close()

def read_saved_requests(taxi_state_object):
    all_requests = [[] for time in range(taxi_state_object.g.N+1)]
    with open(os.getcwd() + '/../data/trajectory_file/requests.csv', mode='r') as infile:
        reader = csv.reader(infile)
        for rows in reader:
            time = int(rows[0])
            request_details = (int(rows[1]),int(rows[2]),int(rows[3]),int(rows[4]))
            all_requests[time].append(request_details)
    return all_requests

def visualize_trajectory(taxi_state_object, policy_name='base_policy', label = ""):
    #print(policy_name)
    trajectory_cost_, trajectory_string_,_ = Run_a_trajectory(taxi_state_object, policy_name)
    print(trajectory_string_)
    print('cost:', trajectory_cost_)
    plot_trajectory(trajectory_string_, policy_name, label, taxi_state_object.g.m)