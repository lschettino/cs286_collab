'''
Harvard CS 286 Spring 2022
'''

import numpy as np

def solve(N, R, Q, C):
    # Array to keep track of whether a task has already been assigned to a robot
    tasks = np.zeros(N)
    
    # Array to keep track of whether a robot has already been assigned to a task
    robots = np.zeros(R)

    # Utility matrix
    U = np.array(Q) - np.array(C)
    
    # Array to keep track of all the assignments: (task, robot) tuples
    assignments = []
    
    # num_assignable_tasks accounts for the fact that there might not be enough robots for the number of tasks.
    # If there are enough robots for all the tasks, then all the tasks will be assigned to a robot
    # Else, if there are less robots than tasks it will assign all the robots to the first R tasks (leaving N-R tasks unassigned) 
    num_assignable_tasks = N if R >= N else R

    # Loop over the number of tasks that can be assigned 
    for i in range(num_assignable_tasks): 
        # Negate utility because argsort sorts in increasing order 
        # (when decreasing utility is what is needed)
        optimal_bots = np.argsort(-U[i])
        counter = 0 
        while robots[optimal_bots[counter]] == 1: 
            counter += 1 

        # found optimal unassigned robot, assign it to a task 
        robots[optimal_bots[counter]] = 1 
        tasks[i] = 1
        print(f'r_{optimal_bots[counter]} assigned to n_{i} at a utility of {U[i][optimal_bots[counter]]}')
        assignments.append([i,optimal_bots[counter]])

    return assignments

    
    

def unit_test():

    #
    # Unit test 0
    #
    print('\n\n\nUnit Test 0: Simple Allocation')
    N_0 = 2
    R_0 = 6
    Q_0 = [[1,40,1,40,9,3],
           [1,20,1,6,9,10]]

    C_0 = [[1,1,1,1,1,1],
           [1,1,1,1,1,1]]

    results_0 = solve(N_0, R_0, Q_0, C_0)

    # The optimal solution is for task 0 to be assigned to robot 3 and task 1 to be assigned to robot 1
    opt_sol_0 = np.array([[0,3],[1,1]])

    expected_sol_0 = np.array([[0,1],[1,5]])

    if np.array_equal(results_0, expected_sol_0):
        print('\tUnit test 1 passed')
        if np.array_equal(results_0, opt_sol_0):
            print('\t\twith optimal solution')
        else: 
            print('\t\twith suboptimal solution')
    else:
        print('\tUnit test 1 FAILED')
        print(f'\tExpected solution is \n {expected_sol_0}')

    #
    # Unit test 1 
    # Testing:
    # - negative value utilities
    # - Ties in utility for a given task
    #
    print('\nUnit Test 1: Allocation with negative utilities and ties')
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

    expected_sol_1 = np.array([[0,1],[1,5],[2,0],[3,2]])

    if np.array_equal(results_1, expected_sol_1):
        print('\tUnit test 1 passed')
        if np.array_equal(results_1, opt_sol_1):
            print('\t\twith optimal solution')
        else: 
            print('\t\twith suboptimal solution')
    else:
        print('\tUnit test 1 FAILED')
        print(f'\tExpected solution is \n {expected_sol_1}')
    


    #
    # Unit test 2 
    # Testing:
    # - Less robots than tasks available (N > R)
    #

    print('\nUnit Test 2: Allocation with N>R')
    N_1 = 4
    R_1 = 2
    Q_1 = [[1,40],
           [1,20],
           [1,40],
           [30,1]]
    C_1 = [[1,1],
           [1,1],
           [1,1],
           [1,1]]

    results_2 = solve(N_1, R_1, Q_1, C_1)

    # Expected solution if the algorithm works correctly
    expected_sol_2 = np.array([[0,1],[1,0]])

    # It is hard to define an optimal solution in this case and it doesn't make that much sense
    # Is it in terms of total utility? Which task should be left unassigned.
    # opt_sol_2 is set to be the expected solution
    opt_sol_2 = np.array([[0,1],[1,0]])


    if np.array_equal(results_2, expected_sol_2):
        print('\tUnit test 1 passed')
        if np.array_equal(results_2, opt_sol_2):
            print('\t\twith optimal solution')
        else: 
            print('\t\twith suboptimal solution')
    else:
        print('\tUnit test 1 FAILED')
        print(f'\tExpected solution is \n {expected_sol_2}')
    print('\n\n')




if __name__ == "__main__":
    unit_test()
    
    print("Input N , R")
    N, R = map(int, input().split())

    print("Input Q")
    Q = []
    for i in range(N):
        Q.append(list(map(int, input().rstrip().split())))

    print("Input C")
    C = []
    for i in range(N):
        C.append(list(map(int, input().rstrip().split())))


    print("Solving...\n")
    solve(N, R, Q, C)



