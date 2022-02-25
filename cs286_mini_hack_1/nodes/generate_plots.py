import rosbag 
import matplotlib.pyplot as plt 


task1_bag = rosbag.Bag("task1.bag") 
task2_bag = rosbag.Bag("task2.bag") 
task3_bag = rosbag.Bag("task3.bag")

task_1 = [], [] 
task_2 = [], []
task_3 = [], []

for _, msg, _ in task1_bag.read_messages(topics=['/odom']):
    task_1[0].append(msg.pose.pose.position.x)
    task_1[1].append(msg.pose.pose.position.y)
task1_bag.close()

for _, msg, _ in task2_bag.read_messages(topics=['/camera/odom/sample']):
    task_2[0].append(msg.pose.pose.position.x)
    task_2[1].append(msg.pose.pose.position.y)
task2_bag.close()

for _, msg, _ in task3_bag.read_messages(topics=['/odom']):
    task_3[0].append(msg.pose.pose.position.x)
    task_3[1].append(msg.pose.pose.position.y)
task3_bag.close()

plt.plot(task_1[0], task_1[1], label = "Task 1")
plt.plot(task_2[0], task_2[1], label = "Task 2")
plt.plot(task_3[0], task_3[1], label = "Task 3")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Comparing robot coordinates traversing a square in all 3 tasks") 
plt.legend()
plt.savefig('all_Tasks.png')
