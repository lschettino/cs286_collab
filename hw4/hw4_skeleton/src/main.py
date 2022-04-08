import sys
sys.path.append('../util/')
from state import state
from base_policy import base_policy
from globals import globals
from get_disturbance import find_pickup_probability, find_dropoff_probability, plot_distribution
from algorithms import MC_simulation, average_MC_simulation, visualize_trajectory, create_and_save_requests


def main_question_1():
    find_pickup_probability()

def main_question_2():
    find_dropoff_probability()

def main_question_3():
    s1 = state()
    create_and_save_requests(s1)
    # it creates data/trajectory_file/requests.csv
    # Each line is the following form
    # time, pickup_x, pickup_y, dropoff_x, dropoff_y

def main_question_4(s1):
    visualize_trajectory(s1, 'base_policy', 'base_policy_1')

def main_question_5(s2):
    visualize_trajectory(s2, 'base_policy','base_policy_2')

def main_question_6(s1):
    visualize_trajectory(s1, 'standard_rollout_policy', 'standard_rollout_policy_1')

def main_question_7(s2):
    visualize_trajectory(s2, 'standard_rollout_policy','standard_rollout_policy_2')

def main_question_8(s2):
    visualize_trajectory(s2, 'one_at_a_time_rollout','one_at_a_time_rollout_2')



if __name__ == '__main__':

    print('starting main code here')
    # main_question_1()

    # main_question_2()

    # main_question_3()

    # s1 = state(None, 1)  # states for 1 agent. Use it in questions 4, 6
    # If you want to compare trajectories given by questions 4, 6, you should use the same instance of the state s1

    s2 = state(None, 2)  # states for 2 agents. Use it in questions 5, 7, 8
    # If you want to compare trajectories given by questions 5, 7, 8, you should use the same instance of the state s2

    # main_question_4(s1)

    main_question_5(s2)

    #main_question_6(s1)

    #main_question_7(s2)

    #main_question_8(s2)







