'''
Harvard CS 286 Spring 2022
'''

used = set()
def solve(N, R, Q, C):
	tempmax = 0
	index = -1
	'''
	Your code here
	'''
	return(index, tempmax)

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




