import numpy as np
from constants import *
import cv2
import glob
import cv2
import numpy as np
import os
from os.path import isfile, join

##################################
# Helper Functions to draw lines
##################################

def line(p0, p1):
    points = []
    N = _diagonal_distance(p0, p1)
    for step in range(N+1):
        t =  0.0 if N==0 else step / N
        points.append(_round_point(_lerp_point(p0, p1, t)))
    return points

def walk_grid(p0, p1):

    dx = p1[0]-p0[0]
    dy = p1[1]-p0[1]
    
    nx = abs(dx)
    ny = abs(dy)
    sign_x = 1 if dx > 0 else -1, 
    sign_y = 1 if dy > 0 else -1
    print(nx)
    print(ny)

    p = (p0[0], p0[1])
    points = [(p[0], p[1])]
    ix = 0
    iy = 0
    while (ix < nx) or (iy < ny):
        if (((0.5+ix) / nx) < ((0.5+iy) / ny)):
            # next step is horizontal
            p[0] += sign_x
            ix+=1
        else:
            ## next step is vertical
            p[1] += sign_y
            iy += 1
        
        points.append((p[0], p[1]))
    
    return points

def _diagonal_distance(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    return max(abs(dx), abs(dy))


def _round_point(p):
    return (round(p[0]), round(p[1]))


def _lerp_point(p0, p1, t):
    return (_lerp(p0[0], p1[0], t),
            _lerp(p0[1], p1[1], t))


def _lerp(start, end, t):
    return start + t * (end-start)




###########################################
# Helper Functions to Frontier Exploration
###########################################

def _is_frontier(point, explored_map, map_dims):
    if explored_map[point] == UNKNOWN_VAL_R:
        for pnt in _adjacent_points(point, map_dims):
            if explored_map[pnt] == VISIBLE_VAL_R:
                return True 

    return False



def at_least_1_is_open(point, explored_map, map_dims):
    assert 0 <= point[0] and point[0] < map_dims[0] and 0 <= point[1] and point[1] < map_dims[1], "_is_open : Invalid point coordinates " + str(point) + " for map of dimensions " + str(map_dims) +""

    for neighbor in _adjacent_points(point, map_dims):
        if explored_map[neighbor] == VISIBLE_VAL_R: 
            return True 
    return False


def _adjacent_points(point, map_dims):
    assert 0 <= point[0] and point[0] < map_dims[0] and 0 <= point[1] and point[1] < map_dims[1], "_adjacent_points : Invalid point coordinates " + str(point) + " for map of dimensions " + str(map_dims) +""
    pnts = []
    for row_off in (-1,0,1):
        for col_off in (-1,0,1):
            
            if row_off != 0 or col_off != 0: # check not to add the current point 
                if (0 <= point[0]+row_off) and (point[0]+row_off < map_dims[0]) and  (0 <= point[1]+col_off) and (point[1]+col_off < map_dims[1]):
                    pnts.append((point[0]+row_off, point[1]+col_off))
    return pnts

###########################################
# Helper Functions to Write video
###########################################

def generate_video(pathIn, pathOut,fps=5, frame_ext='png'):
    frame_array = []
    files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f)) and f.endswith(frame_ext)]
    #for sorting the file names properly
    files.sort(key = lambda x: x[:-4])
    for i in range(len(files)):
        filename=pathIn + files[i]
        #reading each files
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        #inserting the frames into an image array
        frame_array.append(img)

    out = cv2.VideoWriter(pathOut, cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
    
    for i in range(len(frame_array)):
        # writing to a image array
        out.write(frame_array[i])
    out.release()
   



def play_video(filename):
    
    # importing libraries

    # Create a VideoCapture object and read from input file
    cap = cv2.VideoCapture(filename)

    # Check if camera opened successfully
    if (cap.isOpened()== False):
        print("Error opening video file")

    # Read until video is completed
    while(cap.isOpened()):
        
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:

            # Display the resulting frame
            cv2.imshow('Frame', frame)

            # Press Q on keyboard to exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else:
            break

    # When everything done, release
    # the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()

    

if __name__ == '__main__':
    print(_adjacent_points((2,1), (20,20)))
    p0=(0,1)
    p1=(0,10)

    