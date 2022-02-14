'''
Harvard CS 286 Spring 2022
'''

import numpy as np

def solve(N, R, Q, C):
    tasks = np.zeros(N)
    robots = np.zeros(R)
    utility = np.array(Q) - np.array(C) 

    # more tasks than robots 
    if R > N: 
        for i in range(N): 

            # Negate utility because argsort sorts in increasing order 
            # (when decreasing utility is what is needed)
            optimal_bots = np.argsort(-utility[i])
            counter = 0 
            while robots[optimal_bots[counter]] == 1: 
                counter += 1 

            # found optimal unassigned robot, assign it to a task 
            robots[optimal_bots[counter]] = 1 
            tasks[i] = 1
            print(f'r_{optimal_bots[counter]} assigned to n_{i} at a utility of {utility[i][optimal_bots[counter]]}')

    else: 
        print('This is an NP-hard problem')
        pass 


def unit_test():

    #
    # Unit test 0
    #
    print('\n Unit Test 0')
    N_0 = 2
    R_0 = 6
    Q_0 = [[1,40,1,40,9,3],
           [1,20,1,6,9,10]]

    C_0 = [[1,1,1,1,1,1],
           [1,1,1,1,1,1]]

    results_0 = solve(N_0, R_0, Q_0, C_0)

    # The optimal solution is for task 0 to be assigned to robot 3 and task 1 to be assigned to robot 1
    opt_sol_0 = np.array([[0,3],[1,1]])

    assert np.array_equal(results_0, opt_sol_0), f'Failed unit test 0. Solution is {results_0} \nOptimal solution is \n {opt_sol_0}'

    #
    # Unit test 1 
    # Testing:
    # - negative value utilities
    # - Ties in utility for a given task
    #
    print('\n Unit Test 1')
    N_1 = 4
    R_1 = 6
    Q_1 = [[1,40,1,40,1,1],
           [1,20,1,1,1,60],
           [1,40,1,1,1,1],
           [30,1,1,1,1,100]]
    C_1 = [[1,1,1,1,1,1],
           [1,1,1,1,1,1],
           [1,1,1,1,1,1],
           [1,1,1,1,1,200]]

    results_1 = solve(N_1, R_1, Q_1, C_1)
    opt_sol_1 = np.array([[0,3],[1,5],[2,1],[3,0]])

    assert np.array_equal(results_1, opt_sol_1), f'Failed unit test 1.\n Solution is {results_1}\n Optimal solution is \n {opt_sol_1}'






if __name__ == "__main__":
    
    # print("Input N , R")
    # N, R = map(int, input().split())

    # print("Input Q")
    # Q = []
    # for i in range(N):
    #     Q.append(list(map(int, input().rstrip().split())))

    # print("Input C")
    # C = []
    # for i in range(N):
    #     C.append(list(map(int, input().rstrip().split())))

    N = 2
    R = 4
    Q = [[1, 3, 5, 4],
         [3, 2, 6, 4]]
    
    C = [[0, 1, 0, 2],
         [1, 0, 1, 2]]



    print("Solving...\n")
    solve(N, R, Q, C)
    
    unit_test()



