from pickle import NONE
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import shadowcasting
from helper import line, walk_grid, _is_frontier, at_least_1_is_open, _adjacent_points, generate_video
from skimage.draw import polygon, polygon2mask
from constants import *
import cv2
import PIL
from astar import astar
import os



class WiFi_BEACON():
    def __init__(self, initial_position, env):
        assert env.map[initial_position[0]][initial_position[1]] != OBSTACLE_VAL, "The beacon cannot be initialized at a position with an obstacle"
        self.loc = initial_position
        self.env = env
        self.noise = 1

        # Variable that determines whether the beacon is in sight of a robot (in visible space ) 
        # This variable is updated when shadowcasting and a beacon is revealed
        self.is_hidden = True
    

class ROBOT():
    def __init__(self, initial_position, env, initial_orientation=0):
        
        # Class type map
        self.env = env

        assert self.env.map[initial_position[0]][initial_position[1]] != OBSTACLE_VAL, "The robot cannot be initialized at a position with an obstacle"


        self.loc = initial_position
        self.orientation = initial_orientation
        self.env.map[initial_position[0]][initial_position[1]] = ROBOT_VAL


        # Robot's perception map to be overlayed with the actual physical map
        self.robot_map = np.array([[UNKNOWN_VAL_R] * env.dimensions[1] for row in range(env.dimensions[0])])
        self.robot_map[self.loc_tuple()] = ROBOT_VAL_R
        self.frontiers = []

        self.frontier_exploration_map = []

        self.AOA_measurement = None
        self.occupied = False 
        self.beacon = None
        self.beacon_found = False 
        # Initalize the map with the first shadowcast, and frontier measurement
        self.shadowcast()
        self.get_frontiers()


    def assign_beacon(self,beacon):
        self.beacon = beacon
        self.env.map[beacon.loc[0]][beacon.loc[1]] = BEACON_VAL
        self.measure_AOA()

    def loc_tuple(self):
        """
        Helper function to return tuple version of the location (is an array) 
        such that it can be used in numpy indexing
        """
        return tuple(self.loc)

    def move_step(self,dx,dy):
        """
        Move robot in 1 time step to either of its 8 neighbors
        """
        assert dx in {-1,0,1} and dy in {-1,0,1}

        # Set old location in map as visible in robot map
        self.robot_map[self.loc_tuple()] = VISIBLE_VAL_R
        self.env.map[self.loc_tuple()] = EMPTY_VAL



        if 0 < self.loc[0] + dx and  (self.loc[0] + dx < self.env.dimensions[0]):
            self.loc[0] = self.loc[0] + dx
        if 0 < self.loc[1] + dy and  (self.loc[1] + dy < self.env.dimensions[1]):
            self.loc[1] = self.loc[1] + dy

        # Set new location in map as robot location in robot map
        self.robot_map[self.loc_tuple()] = ROBOT_VAL_R
        self.env.map[self.loc_tuple()] = ROBOT_VAL

        # Shadowcast as robot moves
        self.shadowcast()
        # Get frontiers as it moves
        self.get_frontiers()

    
    def move_jump(self, x,y):
        """
        Move robot to location in map (taking a jump)
        """
        assert 0 <= x and x < self.env.dimensions[0] and  0 <= y and y < self.env.dimensions[1]
        # Set old location to visible
        self.robot_map[self.loc_tuple()] = VISIBLE_VAL_R
        # Modify old environment tile to be emptye
        self.env.map[self.loc_tuple()] = EMPTY_VAL

        self.loc = [x,y]
    
        # set new location as robot location
        self.robot_map[self.loc_tuple()] = ROBOT_VAL_R
        # Modify new environment tile to be robot's
        self.env.map[self.loc_tuple()] = ROBOT_VAL
        
        # Shadowcast as robot moves
        self.shadowcast()
        # Get frontiers as it moves
        self.get_frontiers()

    
    def shadowcast(self):
        def is_blocking(x, y):
            return self.env.map[x][y] == OBSTACLE_VAL

        def reveal(x, y):
            # Reveal tile that is visible to the robot.

            if self.loc == [x,y]:
                self.robot_map[x][y] = ROBOT_VAL_R
            # if the tile is an obstacle, the robot identifies it as an obstacle
            elif self.env.map[x][y] == OBSTACLE_VAL:
                self.robot_map[x][y] = OBSTACLE_VAL_R

            elif self.env.map[x][y] == BEACON_VAL:
                # Update the perception map of the robot to indicate the presence of a beacon at x,y
                self.robot_map[x][y] = BEACON_VAL_R

                # Set the beacon state to be visible by the robot such that robot directly plans its route to it (removing frontier exploration)
                self.beacon.is_hidden = False

            elif self.env.map[x][y] in [EMPTY_VAL] :
                self.robot_map[x][y] = VISIBLE_VAL_R

        shadowcasting.compute_fov(self.loc, is_blocking, reveal, PERCEPTION_DEPTH)
     

    def get_frontiers(self):
        """
        Implemented Wavefront Frontier Detector based on https://arxiv.org/pdf/1806.03581.pdf
        Points are classified between 
        - Map-open-list : MOL
        - Map-close-list : MCL
        - Frontier-Open-list : FOL
        - Frontier-close-list : FCL

        Output:
        Modifies robots perception map to include frontiers
        """

        self.frontiers = []

        # map data structure for frontier exploration
        frontier_exploration_map = np.array([[-1] * self.env.dimensions[1] for row in range(self.env.dimensions[0])])

        # Empty Queue M
        queue_m = []
        
        # ENQUEUE(queue_m, pose)
        queue_m.append(self.loc_tuple())

        # Mark pose as "Map-Open-List"
        frontier_exploration_map[self.loc_tuple()] = state_dict['MOL']

        
        while len(queue_m) > 0:
            # p <-- 
            p = queue_m.pop(0)

            if frontier_exploration_map[p] == state_dict['MCL']:
                continue

            if _is_frontier(p, self.robot_map, self.env.dimensions):
                queue_f = []
                new_frontier = []
                queue_f.append(p)

                frontier_exploration_map[p] = state_dict['FOL']

                while len(queue_f) > 0:
                    q = queue_f.pop(0)
                    if frontier_exploration_map[q] in [state_dict['MCL'], state_dict['FCL']]:
                        continue

                    if _is_frontier(q, self.robot_map, self.env.dimensions) :
                        new_frontier.append(q)

                        for w in _adjacent_points(q, self.env.dimensions):
                            if frontier_exploration_map[q] not in [state_dict['FOL'], state_dict['FCL'], state_dict['MCL']]:
                                queue_f.append(w)
                                # mark w as "Frontier Open List"
                                frontier_exploration_map[w] == state_dict['FOL']
                    
                    frontier_exploration_map[q] == state_dict['FCL']

                # Save data of New Frontier

                self.frontiers.append(new_frontier)
  
                # Mark all points of new frontier as Map-Closed-List
                for point in new_frontier:
                    frontier_exploration_map[point] = state_dict['MCL']

            for v in _adjacent_points(p, self.env.dimensions):
                # if v not marked as {Map-Open-List, Map-Closed-List} and v has at least one Map-Open-Space neighbor
                if ((frontier_exploration_map[v] not in [state_dict['MOL'], state_dict['MCL']])

                    and at_least_1_is_open(v, self.robot_map, self.env.dimensions) ):
                    # ENQUEUE(queue_m,v)
                    queue_m.append(v)
                    # Mark v as Map-Open-List
                    frontier_exploration_map[v] = state_dict['MOL']
            
            # Mark p as Map-Closed-List
            frontier_exploration_map[p] = state_dict['MCL']


        # Add all found frontiers to the map
        for frontier in self.frontiers:
            for point in frontier:
                self.robot_map[point] = FRONTIER_VAL_R


    def measure_AOA(self):
        self.AOA_measurement = line(self.loc, self.beacon.loc)

        
    
    def print_map(self, with_AOA=False):
        for r, row in enumerate(self.robot_map):
            print()
            for c, col in enumerate(row):  
                if with_AOA:
                    print(str(col).rjust(2), end=' ')
                else :
                    print(str(col).rjust(2), end=' ')

        print()
    
    def is_in_AOA_trajectory(self, x,y):
        """
        Helper method that checks if a point is in the trajectory (line) between the beacon and the robot
        Uses the values self.AOA_measurement
        """
        return (x,y) in self.AOA_measurement

    def print_map_PIL(self, with_AOA=False):
        """
        Print PIL image of the map

        with_AOA: Flag to plot the tiles that are in the AOA trajectory
        """
        rgb_map = []
        for r, row in enumerate(self.robot_map):
            row_vals = []
            for c, cell in enumerate(row):
                if with_AOA and self.is_in_AOA_trajectory(r,c):
                    row_vals.append(COLOR_MAP_R[BEACON_VAL_R])
                else:
                    row_vals.append(COLOR_MAP_R[cell])
            rgb_map.append(row_vals)

        rgb_map = np.array(rgb_map)

        plt.imshow(rgb_map)
        plt.show()
    
    def save_map_frame(self, filename, header ,with_AOA=False):
        rgb_map = []
        for r, row in enumerate(self.robot_map):
            row_vals = []
            for c, cell in enumerate(row):
                if with_AOA and self.is_in_AOA_trajectory(r,c):
                    row_vals.append(COLOR_MAP_R[BEACON_VAL_R])
                else:
                    row_vals.append(COLOR_MAP_R[cell])
            rgb_map.append(row_vals)

        rgb_map = np.array(rgb_map)
        
        hor_mult_factor = int(FRAME_SIZE[0] / self.env.dimensions[0])
        vert_mult_factor = int(FRAME_SIZE[1] / self.env.dimensions[1])

        rgb_map = np.repeat(rgb_map, hor_mult_factor, axis=1)
        rgb_map = np.repeat(rgb_map, vert_mult_factor, axis=0)

        

        fig, ax = plt.subplots()
        ax.imshow(rgb_map, origin='lower')
        ax.invert_yaxis() # seaborn shows the first row at the top
        ax.set_title(header)

        plt.savefig(os.getcwd() + '/recordings/frames/'+filename+".png")
        

    def path_planning(self, destination, algorithm='ASTAR'):
        assert algorithm in ['ASTAR']

        if algorithm == 'ASTAR':
            maze = self.robot_map
            print(self.robot_map)
            start = self.loc_tuple()
            end = destination
            
            # NOTE for ASTAR algorithm to work, input start and destination must be tuples
            assert type(start) is tuple
            assert type(end) is tuple

            path = astar(maze, start, end)

        else: # Not working yet
            from rrt_custom.src.rrt.rrt import RRT
            from rrt_custom.src.search_space.search_space import SearchSpace

            X_dimensions = np.array([(0, self.env.dimensions[0]), (0, self.env.dimensions[1])])  # dimensions of Search Space
            # obstacles
            Obstacles = np.array([(0, 20, 15, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80), (10, 10, 20, 20)])
            x_init = (0, 0)  # starting location
            x_goal = (100, 100)  # goal location

            Q = np.array([(8, 4)])  # length of tree edges
            r = 1  # length of smallest edge to check for intersection with obstacles
            max_samples = 1024  # max number of samples to take before timing out
            prc = 0.1  # probability of checking for a connection to goal

            # create search space
            X = SearchSpace(X_dimensions, Obstacles)

            # create rrt_search
            rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
            path = rrt.rrt_search()

        return path

    def run_SLAM(self):
        iter_count = 0
        frame_counter = 0

        while not self.beacon_found and iter_count < 30:
            print('Done is run slam')
            print(self.frontiers)
            # self.shadowcast()
            # self.get_frontiers()

            self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count))
            frame_counter+=1

            self.measure_AOA()

            self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count), with_AOA=True)
            frame_counter+=1

            self.print_map_PIL()

            if not self.beacon.is_hidden:
                # If the beacon is in the visible space of the robot
                #     - drop frontier exploration
                #     - drop AOA guiding
                #     - pathplan directly towards the beacon

                path = self.path_planning(tuple(self.beacon.loc))  

                for idx, point in enumerate(path):
                    self.move_jump(point[0], point[1])
                    self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count) + "   Path execution: " + str(idx))
                    frame_counter+=1           

                
            else :
                print('Beacon is still hidden')
                print(set(self.AOA_measurement))
                flatten_frontiers = []
                for frontier in self.frontiers:
                    for point in frontier:
                        flatten_frontiers.append(point)

                intersection = set(flatten_frontiers).intersection(self.AOA_measurement)
                print(tuple(intersection)[0])
                path = self.path_planning(tuple(intersection)[0])
                
                for idx, point in enumerate(path):
                    self.move_jump(point[0], point[1])
                    self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count) + "   Path execution: " + str(idx))
                    frame_counter+=1
                
            # Execute computed path
            
            if self.loc == self.beacon.loc:
                self.beacon_found = True
                self.occupied = False
            iter_count+=1

        frames_path = os.getcwd() + '/recordings/frames/'
        video_path = os.getcwd() + '/recordings/videos/video.mp4'
        # frames per second
        fps = 5
        generate_video(frames_path, video_path, fps)

            

class Robot_System():

    def __init__(self, env,robots, tasks):  
        self.tasks = tasks
        self.robots = robots 
        self.env = env
        self.total_tasks = len(tasks)
        self.robots_map = np.array([[UNKNOWN_VAL_R] * env.dimensions[1] for row in range(env.dimensions[0])])
        

    def build_map(self):
        for robot in self.robots:
            self.env.map[robot.loc[0]][robot.loc[1]] = ROBOT_VAL
            self.robots_map[robot.loc_tuple] = ROBOT_VAL_R

    def assign_robots(self):
        for robot in self.robots:
            if robot.occupied == False:
                for task in self.tasks:
                    robot.assign_beacon(self.tasks.pop(0))

    def combine_maps(self):
        for r, row in enumerate(self.robots_map):
            for c, cell in enumerate(row):
                for robot in self.robots:
                    if robot.robot_map[r][c] == 1 or robot.robot_map[r][c] == 10:
                        self.map = robot.robot_map[r][c]
        
        for robot in self.robots:
            robot.robot_map = self.robots_map

    def print_map_PIL(self, with_AOA=False):
        """
        Print PIL image of the map

        with_AOA: Flag to plot the tiles that are in the AOA trajectory
        """
        
        rgb_map = []
        for r, row in enumerate(self.robots_map):
            row_vals = []
            for c, cell in enumerate(row):
                if with_AOA and self.robots[0].is_in_AOA_trajectory(r,c):
                    row_vals.append(COLOR_MAP_R[BEACON_VAL_R])
                else:
                    row_vals.append(COLOR_MAP_R[cell])
            rgb_map.append(row_vals)

        rgb_map = np.array(rgb_map)

        plt.imshow(rgb_map)
        plt.show()
    
    def save_map_frame(self, filename, header ,with_AOA=False):
        rgb_map = []
        for r, row in enumerate(self.robots_map):
            row_vals = []
            for c, cell in enumerate(row):
                if with_AOA and self.robots[0].is_in_AOA_trajectory(r,c):
                    row_vals.append(COLOR_MAP_R[BEACON_VAL_R])
                else:
                    row_vals.append(COLOR_MAP_R[cell])
            rgb_map.append(row_vals)

        rgb_map = np.array(rgb_map)
        
        hor_mult_factor = int(FRAME_SIZE[0] / self.env.dimensions[0])
        vert_mult_factor = int(FRAME_SIZE[1] / self.env.dimensions[1])

        rgb_map = np.repeat(rgb_map, hor_mult_factor, axis=1)
        rgb_map = np.repeat(rgb_map, vert_mult_factor, axis=0)

        

        fig, ax = plt.subplots()
        ax.imshow(rgb_map, origin='lower')
        ax.invert_yaxis() # seaborn shows the first row at the top
        ax.set_title(header)

        plt.savefig(os.getcwd() + '/recordings/frames/'+filename+".png")
        

    def run_slams(self):
        self.build_map
        iter_count = 0
        frame_counter = 0
        all_robots_done = 0 
        while all_robots_done < self.total_tasks and iter_count < 10:

            self.assign_robots()
            # self.shadowcast()
            # self.get_frontiers()
            
            self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count))
            frame_counter+=1

            for robot in self.robots:
                if robot.occupied == True:
                    robot.measure_AOA()
                    self.combine_maps

            self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count), with_AOA=True)
            frame_counter+=1

            self.print_map_PIL()


            for robot in self.robots:
                if robot.occupied == True:
                    if not robot.beacon.is_hidden:
                    
                        # If the beacon is in the visible space of the robot
                        #     - drop frontier exploration
                        #     - drop AOA guiding
                        #     - pathplan directly towards the beacon

                        path = robot.path_planning(tuple(self.beacon.loc))  
                        robot.move_jump(path[0], path[1])
                    else:
                        flatten_frontiers = []
                        for frontier in robot.frontiers:
                            for point in frontier:
                                flatten_frontiers.append(point)

                        intersection = set(flatten_frontiers).intersection(self.AOA_measurement)
                        path = robot.path_planning(tuple(intersection)[0])
                        robot.move_jump(path[0], path[1])

                    if robot.loc == robot.beacon.loc:
                        robot.beacon_found = True
                        robot.occupied = False
                        all_robots_done += 1
            self.combine_maps()
            self.save_map_frame(filename="" + str(frame_counter), header="Iter: "+ str(iter_count))
            frame_counter+=1           
            iter_count+=1        
            # Execute computed path

        frames_path = os.getcwd() + '/recordings/frames/'
        video_path = os.getcwd() + '/recordings/videos/video.mp4'
        # frames per second
        fps = 5
        generate_video(frames_path, video_path, fps)
        
    
class SLAM_MAP():
    def __init__(self, map_dimensions, poly_obstacles, line_obstacles):
        
        self.dimensions = map_dimensions
        self.map = []
        # map with obstacles
        self.build_map(*map_dimensions, poly_obstacles, line_obstacles)
    

    def build_map(self, width=10, height=10, poly_obstacles=[], line_obstacles=[]):

        # Dimensionality checks
        for obstacle in poly_obstacles:
            for vertex in obstacle:
                assert 0 <= vertex[0] and vertex[0] < width and 0 <= vertex[1] and vertex[1] < height, f"{vertex}"
        
        # Dimensionality checks
        for obstacle in line_obstacles:
            for vertex in obstacle:
                assert 0 <= vertex[0] and vertex[0] < width and 0 <= vertex[1] and vertex[1] < height, f"{vertex}"    
    

        for i in range(height):
            self.map.append([])
            for j in range(width):
                if (i == 0) or (i == height - 1) or (j == 0) or (j == width - 1):
                    val = OBSTACLE_VAL
                else:
                    val = EMPTY_VAL
                self.map[i].append(val)
        
        # convert to numpy array for masking
        self.map = np.array(self.map)

        for obstacle in poly_obstacles:
            self._add_polygone_obstacle((width,height), obstacle)
        
        for line in line_obstacles:
            self._add_line_obstacle(line)
    

    def _add_polygone_obstacle(self, shape, vertices):
        mask = polygon2mask(shape, vertices)
        self.map[mask] = OBSTACLE_VAL

    def _add_line_obstacle(self, vertices):
        assert len(vertices)>=2 , "A line is defined by at least 2 vertices"
        for i in range(len(vertices)-1):
            for point in line(vertices[i], vertices[i+1]):
                self.map[point] = OBSTACLE_VAL

    def print_map(self):
        for r, row in enumerate(self.map):
            print()
            for c, col in enumerate(row):
                print(col, end=' ')
        print()

    def print_map_PIL(self, with_AOA=False):
        rgb_map = [[COLOR_MAP_M[cell] for cell in row] for row in self.map]
        plt.imshow(rgb_map)
        plt.show()




if __name__ == '__main__':

    map_dimensions = (30,30)
    # Obstacles are polygones defined by at LEAST 3 vertices and the space between 
    # the vertices is filled with obstacle
    obstacles_poly = [
            [(0,0), (0,8), (5,8)],
            [(15,15), (15,27), (18,22)],    
        ]
    
    # Draw obstacle lines from at least 2 vertices
    obstacles_line = [
        [(3,2),(3,6)]
    ]
    

    slam_map = SLAM_MAP(map_dimensions , obstacles_poly, obstacles_line)

    beacon_pos = [20,3]
    beacon1 = WiFi_BEACON(beacon_pos, slam_map)

    tasks = [beacon1]

    robot_pos = [1, 1]
    robot = ROBOT(robot_pos, slam_map)
    
    robot_pos_2 = [25, 25]
    robot2 = ROBOT(robot_pos_2, slam_map)
    robots = [robot, robot2]
    robot_system = Robot_System(slam_map,robots, tasks)

    robot_system.run_slams()
    # robot2.run_SLAM()

