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
#print(solve(N, R, Q, C))
