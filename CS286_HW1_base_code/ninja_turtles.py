'''
Harvard CS 286 Spring 2022
'''

#import rospy

'''
Import the modules required for using ROS turtlesim services.
'''

class NinjaTurtles:
    def __init__(self, x, y, name):
        self.x = x
        self.y = y
        self.name = name

    def remove_bot(self,name='turtle1'):
        '''
        Use the turtlesim ROS package service that will remove the default turtle in turtlesim
        '''
        pass

    def add_bot(self):
        '''
        Use the turtlesim ROS package service that will spawn a new turtle
        '''
        pass


    def go_to(self,new_x=0, new_y=0):
        '''
        Use the turtlesim ROS package service that enables a turtle to 
        directly teleport to a new location (new_x,new_y)
        '''
        pass
    
if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    t1 = NinjaTurtles(1,4,'t1')
    t1.remove_bot('turtle1')
    t1.add_bot()
    t1.go_to(5,5)
    


