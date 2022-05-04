############################################################################################
# CS286 Final Project (April 2022): Constants file
############################################################################################


# Values used in map
OBSTACLE_VAL = ' #'
EMPTY_VAL = ' .'
ROBOT_VAL = ' @'
BEACON_VAL = ' B'

COLOR_MAP_M = {
    EMPTY_VAL : [255,255,255],
    OBSTACLE_VAL : [0,0,0],
    ROBOT_VAL : [255,0,0],
    BEACON_VAL : [0,255,0]
}



# Potentials used in robot map
VISIBLE_VAL_R = 1
ROBOT_VAL_R = 0
UNKNOWN_VAL_R = -1
OBSTACLE_VAL_R = -9
FRONTIER_VAL_R = 2
BEACON_VAL_R = 10


COLOR_MAP_R = { 
    UNKNOWN_VAL_R: [128,128,128],
    ROBOT_VAL_R: [255,0,0],
    VISIBLE_VAL_R: [255,255,255],
    FRONTIER_VAL_R: [0,0,255],
    OBSTACLE_VAL_R: [0,0,0],
    BEACON_VAL_R: [0,255,0]
}


# Frontier exploration cell states
state_dict = {
            'MOL':4,
            'MCL':5,
            'FOL':6,
            'FCL':7
        }


# Parameters for sensor perception
PERCEPTION_DEPTH = 10 # i.e how far (in how many tiles) can the robot see. used to cap recursive step



# Video rendering
FRAME_SIZE = (600, 600)