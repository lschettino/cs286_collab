'''
Harvard CS 286 Spring 2022
'''

from ninja_turtles import NinjaTurtles
import rospy
import numpy as np
import matplotlib.pyplot as plt

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
        pass

    def position_cb_leo(self,msg):
        '''
        Store the position data in the msg in self.leo_data
        '''
        pass


    def position_cb_ralph(self,msg):
        '''
        Store the position data in the msg in self.ralph_data
        '''
        pass


    def position_cb_mike(self,msg):
        '''
        Store the position data in the msg in self.mike_data
        '''
        pass


    def get_plot(self):
        '''
        Generat the final plot that show the trace of all robot positions
        '''
        
        fig = plt.figure()
        ax = plt.axes()

        '''
        Your code to generate plot here
        '''
        plt.show()
        

def main():
    print("Starting")
    rospy.init_node("Cowabunga_plots")
    viz = NinjaTurtlesViz()
    
    '''
    Create subscribers for each turtle's pose topic and use the approaporiate callback functions
    '''
    
    print("Press CTRL+C once the simulation ends to generate the plot")
    rospy.spin()
    viz.get_plot()

if __name__ == "__main__":
    main()
