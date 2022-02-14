'''
Harvard CS 286 Spring 2022

Requires installing Mixed-Integer linear programming packages cvxpy and cvxopt
pip install cvxpy
pip install cvxopt
'''

import numpy as np 
import cvxpy as cp


def solve(N, R, Q, C):
    tasks = np.zeros(N)
    robots = np.zeros(R)
    utility = np.array(Q) - np.array(C) 

    # more tasks than robots 
    if R > N: 
        for i in range(N): 
           optimal_bots = np.flip(np.argsort(utility[i]))

           counter = 0 
           while robots[optimal_bots[counter]] == 1: 
               counter += 1 

           # found optimal unassigned robot, assign it to a task 
           robots[optimal_bots[counter]] = 1 
           tasks[i] = 1
           print(optimal_bots[counter], i, utility[i][optimal_bots[counter]])

    else: 
        print('This is an NP-hard problem')
        pass 


def solve_CVXPY(N, R, Q, C):
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
        print('The Optimizer found a solution')

        # Binarize solutions (Opimizer might not return boolean variables but rather a range from 0 to 1)
        assignments = (np.array(assignments.value) > 0.5) * 1.0

        # Get assigned tuples of (task, robot) 
        results = np.argwhere(assignments==1)

        # Print results
        for task, robot in results:
            print(f'n_{task} assigned to r_{robot} at a utility of {U[task][robot]}')
        return results
    else:
        print('The Optimizer did not find a solution')
    
  
def unit_test():

    #
    # Unit test 0
    #
    N_0 = 2
    R_0 = 6
    Q_0 = [[1,40,1,40,9,3],
           [1,20,1,6,9,10]]

    C_0 = [[1,1,1,1,1,1],
           [1,1,1,1,1,1]]

    results_0 = solve_CVXPY(N_0, R_0, Q_0, C_0)

    # The optimal solution is for task 0 to be assigned to robot 3 and task 1 to be assigned to robot 1
    opt_sol_0 = np.array([[0,3],[1,1]])

    if np.array_equal(results_0, opt_sol_0):
        print('Unit test 0 passed')
    else:
        print('Unit test 1 FAILED')
        

    #
    # Unit test 1 
    # Testing:
    # - negative value utilities
    # - Ties in utility for a given task
    #

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

    results_1 = solve_CVXPY(N_1, R_1, Q_1, C_1)

    opt_sol_1 = np.array([[0,3],[1,5],[2,1],[3,0]])


    if np.array_equal(results_1, opt_sol_1):
        print('Unit test 1 passed')
    else:
        print('Unit test 1 FAILED')
        print(f'Expected solution is \n {opt_sol_1}')

    

       

# print("Input N , R")
# N, R = map(int, input().split())

# print("Input Q")
# Q = []
# for i in range(N):
# 	Q.append(list(map(int, input().rstrip().split())))

# print("Input C")
# C = []
# for i in range(N):
# 	C.append(list(map(int, input().rstrip().split())))

N = 2 
R = 5
Q = [[1,3,5,2,1],[49,3,5,3,1]]
C = [[1,1,1,5,1],[9,6,78,2,1]]




print(Q)
print(C)
print("Solving...")
print(solve_CVXPY(N, R, Q, C))
print(unit_test())
#print(solve(N, R, Q, C))
