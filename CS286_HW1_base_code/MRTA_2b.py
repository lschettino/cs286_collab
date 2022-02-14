'''
Harvard CS 286 Spring 2022
'''

used = set()


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

    results_0 = solve(N_0, R_0, Q_0, C_0)

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

    results_1 = solve(N_1, R_1, Q_1, C_1)

    opt_sol_1 = np.array([[0,3],[1,5],[2,1],[3,0]])


    if np.array_equal(results_1, opt_sol_1):
        print('Unit test 1 passed')
    else:
        print('Unit test 1 FAILED')
        print(f'Expected solution is \n {opt_sol_1}')






if __name__ == "__main__":
    
    print("Input N , R")
    N, R = map(int, input().split())
    result = []

    for i in range(N):
        print("Input Q")
        Q = (list(map(int,input().rstrip().split())))
        
        print("Input C")
        C = (list(map(int,input().rstrip().split())))
        
        print("Solution for task ", str(i))
        cur_result = solve(1, R, Q, C)
        result.append(cur_result)
        used.add(cur_result[0])
        print(result)
        print(used)




