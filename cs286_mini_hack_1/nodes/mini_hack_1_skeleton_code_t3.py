#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, isnan
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import math
import matplotlib.pyplot as plt


#New===
import argparse
from sensor_msgs.msg import LaserScan, JointState 
from nav_msgs.msg import Odometry
#Import the package corresponding to the LiDAR data from sensor_msgs
#Import the package corresponding to the message type of the position_topic from nav_msgs
#New===

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
LIMIT_RANGE=2

print_msg = """
control your Turtlebot3 for Waypoint!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""


class GotoPoint():
    def __init__(self, position_topic, velocity_topic, scan_topic):
        print(position_topic)
        rospy.init_node('pointop_key', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.r = rospy.Rate(10)
        
        #======New=====
        self.scan_topic = scan_topic
        self.cmd_vel = rospy.Publisher(velocity_topic, Twist, queue_size=5)
    
        '''
        Create a subscriber for the position_topic make sure to use the correct message type.        
        Reference: "Writing a subscriber" section in http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29"
        '''
        self.sub_pos = rospy.Subscriber(position_topic, Odometry, self.get_position_cb)
        
        self.x_coordinates = [] 
        self.y_coordinates = [] 
        self.rot = Quaternion()
        self.curr_position = Point()
        self.curr_rotation = 0
        self.waypoints = [[2.2, 0, 0], [2.2, -2.2, -90] , [0, -2.2, -180], [0, 0, 90]]
 #These are the rough dimensions of the testbed
        self.waypoint_counter = 0
        #=====New=====
                
    def get_scan(self):
        '''
    Use the code in *_obstacle.py to update the scan_filter value which is returned.
        '''
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  
        samples_view = 2        
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf') or scan_filter[i] == 0 or scan_filter[i] > LIMIT_RANGE :
                scan_filter[i] = LIMIT_RANGE
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter


    def go_to_waypoint(self):
        position = Point()
        move_cmd = Twist()

        print("Starting new waypoint")
        position = self.curr_position
        rotation = self.curr_rotation
        last_rotation = 0
        linear_speed = 1 
        angular_speed = 1
        
        (goal_x, goal_y, goal_z) = self.getWaypoint()
        time.sleep(1)
        print("Got waypoints")
            
        if goal_z > 180 or goal_z < -180:
                print("you input wrong z range.")
                self.shutdown()
        
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        print("initial distance: ", distance)
        print(position)
        print(goal_x, goal_y, goal_z)

        while distance > 0.05:
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
	    rospy.loginfo("Min distance : %f",min_distance)
            

            if min_distance < SAFE_STOP_DISTANCE and min_distance > 0:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                rospy.loginfo('Obstacle detected!')
                turtlebot_moving = False 
                print("while 1 if: ", rotation, last_rotation) 
            else: 
                turtlebot_moving = True 
                

                position = self.curr_position
                rotation = self.curr_rotation

                print("while 1 else: ", rotation, last_rotation)

                x_start = position.x
                y_start = position.y
                path_angle = atan2(goal_y - y_start, goal_x- x_start)
            
                if path_angle < -pi/4 or path_angle > pi/4:
                    if goal_y < 0 and y_start < goal_y:
                        path_angle = -2*pi + path_angle
                    elif goal_y >= 0 and y_start > goal_y:
                        path_angle = 2*pi + path_angle

                if last_rotation > pi-0.1 and rotation <= 0:
                    rotation = 2*pi + rotation
                elif last_rotation < -pi+0.1 and rotation > 0:
                    rotation = -2*pi + rotation
                print("\t updated rotation: ", rotation) 
                move_cmd.angular.z = angular_speed * path_angle-rotation

                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
                move_cmd.linear.x = min(linear_speed * distance, 0.1)

                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
            position = self.curr_position
            rotation = self.curr_rotation


        while abs(rotation - goal_z) > 0.05:
            print("while 2") 
            position = self.curr_position
            rotation = self.curr_rotation
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()

        rospy.loginfo("Stopping the robot...") 
        self.cmd_vel.publish(Twist())
       

    def getkey(self):
        x, y, z = raw_input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z


    def getWaypoint(self):
        if self.waypoint_counter == len(self.waypoints): 
            self.shutdown()
        val = self.waypoints[self.waypoint_counter]
        self.waypoint_counter+=1
        
        x = val[0]
        y = val[1]
        z = val[2]

        return x, y, z


    def get_position_cb(self, msg):
        '''
        Update the self.curr_position and self.curr_rotation in this callback function to the position_topic.
        Reference: 
        1. get_odom() function in the *_pointop_key.py
        2. "Writing a subscriber" section in http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29"
        3. HW1 turtles_assemble.py     
        '''      

        rot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
        self.rot = rot 
        temp = euler_from_quaternion(rot) 
        yaw_angle = temp[2] 
        self.curr_rotation = yaw_angle 
        self.curr_position = Point(msg.pose.pose.position.x, msg.pose.pose.position.y, 0)
        self.x_coordinates.append(msg.pose.pose.position.x)
        self.y_coordinates.append(msg.pose.pose.position.y)

    def get_plot(self):
        '''
        Generate the final plot that show the trace of all robot positions
        '''
        
        fig = plt.figure()
        ax = plt.axes()

        '''
        Your code to generate plot here
        '''
        
        ax.plot(self.x_coordinates, self.y_coordinates, marker = 'o', color = "red", alpha = 0.8)
        plt.savefig('task3.png')

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        self.get_plot()
        exit(1)
    
    
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--position_topic', type=str)
    parser.add_argument('--velocity_topic', type=str)
    parser.add_argument('--scan_topic', type=str)
    args=parser.parse_args()

    try:
        obj = GotoPoint(args.position_topic, args.velocity_topic, args.scan_topic)
        while not rospy.is_shutdown():
            obj.go_to_waypoint()
    except Exception as e: 
        print(e)
        rospy.loginfo("shutdown program now.")

