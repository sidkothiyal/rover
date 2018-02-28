import numpy as np 

def dynamic_path_finder(maps, goal=None, route=None):
	if goal is None:
		goal = (0, len(maps[0])-1)
	if route is None:
		route = np.full_like(maps, 9999)
		route[goal[0]][goal[1]] = 0
	
	for i in range(-1, 2):
		for j in range(-1, 2):
			if goal[0] + i >= 0 and goal[0] + i < len(maps) and goal[1] + j >= 0 and goal[1] + j < len(maps[0]):
				if route[goal[0]+i][goal[1]+j] > route[goal[0]][goal[1]] + 1 and maps[goal[0]+i][goal[1]+j] != 1:
					route[goal[0]+i][goal[1]+j] = route[goal[0]][goal[1]] + 1
					route = dynamic_path_finder(maps, (goal[0] + i, goal[1] + j), route)
	return route

if __name__ == '__main__':
	
	a = np.array([[0, 1, 0, 0, 0],
				[0, 1, 0, 0, 0],
				[0, 1, 1, 1, 0],
				[0, 0, 0, 0, 0],
				[0, 1, 0, 0, 0]])
	print(dynamic_path_finder(a, (2, 2)))