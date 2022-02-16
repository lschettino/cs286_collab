'''
Harvard CS 286 Spring 2022
'''

from ninja_turtles import NinjaTurtles
import rospy
import numpy as np
import matplotlib.pyplot as plt
from turtlesim.msg import Pose 

'''
Import the module to use msg corresponding to the turtles pose topic
'''

plt.style.use('seaborn-whitegrid')

class NinjaTurtlesViz:
    def __init__(self):
        self.don_data = []
        self.leo_data = []
        self.ralph_data = []
        self.mike_data = []


    def position_cb_don(self,msg):
        '''
        Store the position data in the msg in self.don_data
        '''
        self.don_data.append((msg.x, msg.y))
        


    def position_cb_leo(self,msg):
        '''
        Store the position data in the msg in self.leo_data
        '''
        self.leo_data.append((msg.x, msg.y))


    def position_cb_ralph(self,msg):
        '''
        Store the position data in the msg in self.ralph_data
        '''
        self.ralph_data.append((msg.x, msg.y))


    def position_cb_mike(self,msg):
        '''
        Store the position data in the msg in self.mike_data
        '''
        self.mike_data.append((msg.x, msg.y))


    def get_plot(self):
        '''
        Generate the final plot that show the trace of all robot positions
        '''
        
        fig = plt.figure()
        ax = plt.axes()

        '''
        Your code to generate plot here
        '''
        don_x, don_y = [i for i, j in self.don_data], [j for i, j in self.don_data]

        leo_x, leo_y = [i for i, j in self.leo_data], [j for i, j in self.leo_data]

        ralph_x, ralph_y = [i for i, j in self.ralph_data], [j for i, j in self.ralph_data]

        mike_x, mike_y = [i for i, j in self.mike_data], [j for i, j in self.mike_data]
        
        ax.plot(don_x, don_y, marker = 'o', color = "red", alpha = 0.8)
        ax.plot(leo_x, leo_y, marker = 'o', color = "blue", alpha = 0.8)
        ax.plot(ralph_x, ralph_y, marker = 'o', color = "green", alpha = 0.8 )
        ax.plot(mike_x, mike_y, marker = 'o', color = "magenta", alpha = 0.8)
        plt.show()
        plt.savefig('turtlesim_viz.png')
        

def main():
    print("Starting")
    rospy.init_node("Cowabunga_plots")
    viz = NinjaTurtlesViz()
    
    '''
    Create subscribers for each turtle's pose topic and use the approaporiate callback functions
    '''
    don_subscriber = rospy.Subscriber('/t1/pose', Pose, viz.position_cb_don)
    leo_subscriber = rospy.Subscriber('/t2/pose', Pose, viz.position_cb_leo)
    ralph_subscriber = rospy.Subscriber('/t3/pose', Pose, viz.position_cb_ralph)
    mike_subscriber = rospy.Subscriber('/t4/pose', Pose, viz.position_cb_mike)
    
    print("Press CTRL+C once the simulation ends to generate the plot")

    rospy.spin()
    viz.get_plot()

if __name__ == "__main__":
    main()
