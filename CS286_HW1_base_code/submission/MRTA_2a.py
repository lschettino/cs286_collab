'''
Harvard CS 286 Spring 2022

Requires installing Mixed-Integer linear programming packages cvxpy and cvxopt
pip install cvxpy
pip install cvxopt
'''

import numpy as np 
import cvxpy as cp

def solve(N, R, Q, C):
    if R < N:
        print("This linear optimization algorithm doesn't have a solution: \n\t-There are more tasks than robots")
        return 0
    
    # Utility matrix
    U = np.array(Q) - np.array(C) 

    assignments = cp.Variable(shape=(N,R), boolean=True)

    # Maximize sum of elementwise product of assignments and utility
    objective_fct = cp.Maximize(cp.sum(cp.multiply(assignments, U)))

    constraint_list = []

    # Constraint: no more than one task per robot
    constraint_list.append(cp.sum(assignments, axis=0) <= 1) 

    # Constraint: Every task must have one assigned robot
    constraint_list.append(cp.sum(assignments, axis=1) == 1) 

    problem = cp.Problem(objective_fct, constraint_list)

    problem.solve(solver="GLPK_MI", verbose=False)

    if problem.status in ["optimal", "optimal_inaccurate"]:
        # Binarize solutions (Opimizer might not return boolean variables but rather a range from 0 to 1)
        assignments = (np.array(assignments.value) > 0.5) * 1.0

        # Get assigned tuples of (task, robot) 
        results = np.argwhere(assignments==1)

        # Print results
        for task, robot in results:
            print(f'\tn_{task} assigned to r_{robot} at a utility of q-c={U[task][robot]}')
        return results
    else:
        print('The Optimizer did not find a solution')
        return 0
    
  
def unit_test():

    #
    # Unit test 0
    #
    print('\nUnit Test 0: Simple allocation')
    N_0 = 2
    R_0 = 6
    Q_0 = [[1,40,1,40,9,3],
           [1,20,1,6,9,10]]

    C_0 = [[1,1,1,1,1,1],
           [1,1,1,1,1,1]]

    results_0 = solve(N_0, R_0, Q_0, C_0)

    # The optimal solution is for task 0 to be assigned to robot 3 and task 1 to be assigned to robot 1
    opt_sol_0 = np.array([[0,3],[1,1]])

    if np.array_equal(results_0, opt_sol_0):
        print('\tUnit test 0 passed')
    else:
        print('\tUnit test 1 FAILED')
        print(f'Optimal solution is \n {opt_sol_0}')
   

    #
    # Unit test 1 
    # Testing:
    # - negative value utilities
    # - Ties in utility for a given task
    #
    print('\nUnit Test 1: Allocation with negative utility and ties')
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


    if np.array_equal(results_1, opt_sol_1):
        print('\tUnit test 1 passed')
    else:
        print('\tUnit test 1 FAILED')
        print(f'Optimal solution is \n {opt_sol_1}')

    

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

    # Expected solution if the algorithm works correctly. 
    # In this case, there should not be a solution
    expected_sol_2 = 0

    # No solution
    opt_sol_2 = 0


    if results_2 == expected_sol_2:
        print('\tUnit test 1 passed')
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


    print("Solving...")
    solve(N, R, Q, C)
