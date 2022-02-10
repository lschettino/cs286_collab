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
           optimal_bots = np.flip(np.argsort(utility[i]))
           counter = 0 
           while robots[optimal_bots[counter]] == 1: 
               counter += 1 
           # found optimal unassigned robot, assign it to a task 
           robots[optimal_bots[counter]] = 1 
           tasks[i] = 1 
           print(optimal_bots[counter], i, utility[i][optimal_bots[counter]])

    else: 
        pass 


       
            

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

print(Q)
print(C)
print("Solving...")
print(solve(N, R, Q, C))
