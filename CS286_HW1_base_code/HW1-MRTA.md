# HW1-MRTA - Question 2

## (b)

## (c)
No, the algorithm that we provided did not achieve the optimal solution. Since we are selecting the tasks greedily, we will assign the first robot with the highest utility to the task. The optimal solution requires looking forward and knowing that same robot might be better at the next task than any other robot. As an example, if we have Q = [[1,20,20], [1,20,5]] and C = [[1,1,1],[1,1,1]] our algorithm would select robot 1 for task 0 and robot 2 for task 1. Yet the optimal solution would be to assign robot 1 to task 1 and robot 2 for task 0. This is because both robots have the same utility for task 0, but robot 1 has a much higher utility for task 0 than robot 2 has. Thus robot 1 should be assigned to task 1 and robot 2 to task 0 to get the highest utility overall. The lower bound on the algorithm is 2-competitiveas stated in the paper for the BLE algorithm

## (d)
The single robot tasks are generally simpler than the multi robot task assignment. The single robot tasks can be solved in polynomial time since it is in the form of the optimal assignment problem. On the other hand, multi robot task assignment is shown to be an instance of the set partitioning problem which is strongly np-hard

## (e)
A key component of why a decentralized allocation system is better than a centralized one is the amount of time it takes to propagate those messages through the system. This is especially apparent in larger systems rather since the impact of this overhead becomes more apparent with more nodes. Another valuable part of decentralized is that, for a centralized system, if leaders fail the whole system is ruined. With decentralization the risk is spread out and has redundancy in case of failure. 