import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import argparse

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
PI = 3.14

class RobotSLAM_Nav:
    def __init__(self, goal_topic, position_topic,bot):
        rospy.init_node("move_base_tester")
        self.client = actionlib.SimpleActionClient(goal_topic,MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        
        if(bot == 1):
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/locobot/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/locobot/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/locobot/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
            clear_costmap()
        else:
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmap()


        #Create the actionlib server
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        rospy.Subscriber(position_topic, Odometry, self.odom_cb)
        rospy.Subscriber('aoa_topic', Float32, self.aoa_cb_dummy)
        self.gotAOA = False
        self.current_position = Point()
        self.current_ori = Quaternion()

    def aoa_cb_dummy(self, msg):
        self.move_direction = msg.data 
        self.gotAOA = True
        print("Got new AOA")

    def move(self):

        while True:
            print(msg)
            
            try:
                x, y, z = raw_input("| x | y | z |\n").split()
            except ValueError:
                rospy.loginfo("Shutting Down")
                rospy.signal_shutdown("Shutting Down")
                break

            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = float(x)
            self.goal.target_pose.pose.position.y = float(y)
            
            q = quaternion_from_euler(0,0,float(z))

            self.goal.target_pose.pose.orientation.x = q[0]
            self.goal.target_pose.pose.orientation.y = q[1]
            self.goal.target_pose.pose.orientation.z = q[2]
            self.goal.target_pose.pose.orientation.w = q[3]
            
            rospy.loginfo("Attempting to move to the goal")
            self.client.send_goal(self.goal)
            wait=self.client.wait_for_result(rospy.Duration(self.timeout))

            if not wait:
                rospy.loginfo("Timed-out after failing to reach the goal.")
                self.client.cancel_goal()
                rospy.loginfo("Please provide a new goal position")
            else:
                rospy.loginfo("Reached goal successfully")


    def odom_cb(self,msg):
	#Update this callback function to get the positions of the robot from its odometery.
	#Use the variables self.current_position and self.current_ori to store the position and orientation values.
	self.current_position.x = msg.pose.pose.position.x
	self.current_position.y = msg.pose.pose.position.y 
        rot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
	self.current_ori = rot 
		
	

    def move_along_direction(self):
        rospy.loginfo("Waiting for AOA...")
        while not rospy.is_shutdown():    
            if(self.gotAOA):
		angle = self.move_direction / 180 * math.pi # (to convert to radians) 
		delta_x = self.step_size * math.cos(angle) 
		delta_y = self.step_size * math.sin(angle) 
		# Add your code here to update x and y based on the AOA direction and step-size
		x = self.current_position.x + delta_x 
                y = self.current_position.y + delta_y
		# use AOA to get angle 
		# math.cosine(theta) * self.step_size 		
                print("Moving to next location x = ",x, ", y =",y)
                
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = x
                self.goal.target_pose.pose.position.y = y

                self.goal.target_pose.pose.orientation.x = self.current_ori[0]
                self.goal.target_pose.pose.orientation.y = self.current_ori[1]
                self.goal.target_pose.pose.orientation.z = self.current_ori[2]
                self.goal.target_pose.pose.orientation.w = self.current_ori[3]
                
                rospy.loginfo("Attempting to move to the goal")
                self.client.send_goal(self.goal)
                wait=self.client.wait_for_result(rospy.Duration(self.timeout))

                if not wait:
                    rospy.loginfo("Timed-out after failing to reach the goal.")
                    self.client.cancel_goal()
                    rospy.loginfo("Please provide a new goal position")
                else:
                    rospy.loginfo("Reached goal successfully")

                self.gotAOA = False

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Get the inputs.')
    parser.add_argument('--goal_topic', type=str)
    parser.add_argument('--position_topic', type=str)
    parser.add_argument('--bot', type=int)
    args = parser.parse_args()
    obj = RobotSLAM_Nav(args.goal_topic, args.position_topic, args.bot)
    
    #obj.move()
    obj.move_along_direction()
