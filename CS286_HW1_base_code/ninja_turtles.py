'''
Harvard CS 286 Spring 2022
'''

import rospy
import turtlesim 
from turtlesim.srv import Kill, Spawn, TeleportAbsolute

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
        try:
            self.kill_turtle = rospy.ServiceProxy('/kill', turtlesim.srv.Kill)
            self.kill_turtle(name)
        except rospy.ServiceException, rospy.ROSInterruptException as e:
            print(e) 
        
    def add_bot(self):
        '''
        Use the turtlesim ROS package service that will spawn a new turtle
        '''
        
        try:
            self.spawn_turtle = rospy.ServiceProxy('/spawn', turtlesim.srv.Spawn)
            print("turtle spawned")
            self.spawn_turtle(self.x, self.y, 0, self.name)
        except rospy.ServiceException as e:
            print("Service call failes: %s"%e)


    def go_to(self,new_x=0, new_y=0):
        '''
        Use the turtlesim ROS package service that enables a turtle to 
        directly teleport to a new location (new_x,new_y)
        '''

        service_name = "/{name}/teleport_absolute"
        self.move_turtle = rospy.ServiceProxy(service_name.format(name = self.name), turtlesim.srv.TeleportAbsolute)
        self.move_turtle(new_x, new_y, 0)
    
if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    t2 = NinjaTurtles(1,4,'t2')
    t2.add_bot()
    t2.remove_bot('t2')
    t3 = NinjaTurtles(1,1,'t3')
    t3.add_bot()
    t3.go_to(5, 5)
    
    
   


