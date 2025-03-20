#!/usr/bin/env python3
import rospy, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def save_goalpos(msgg):
    if(min_front > 10.0 and min_right > 5 and min_left > 5): 
        global is_goal_visited
        global pos
        pos = msgg.pose.pose
        if(is_goal_visited == 0):
            goal_pose = msgg.pose.pose 
        else: #stop the exploration
            command.linear.x = 0.0
            command.angular.z = 0.0
            cmd_vel_pub.publish(command)
            print('stop exploration')

def scan_callback(msg):
    global range_front
    global range_right
    global range_left
    global ranges
    global min_front,i_front, min_right,i_right, min_left ,i_left
    
    ranges = msg.ranges
    # get the range of a few points
    # in front of the robot (between 5 to -5 degrees)
    range_front[:5] = msg.ranges[5:0:-1]  
    range_front[5:] = msg.ranges[-1:-5:-1]
    # to the right (between 300 to 345 degrees)
    range_right = msg.ranges[300:345]
    # to the left (between 15 to 60 degrees)
    range_left = msg.ranges[60:15:-1]
    # get the minimum values of each range 
    # minimum value means the shortest obstacle from the robot
    min_range,i_range = min( (ranges[i_range],i_range) for i_range in range(len(ranges)) )
    min_front,i_front = min( (range_front[i_front],i_front) for i_front in range(len(range_front)) )
    min_right,i_right = min( (range_right[i_right],i_right) for i_right in range(len(range_right)) )
    min_left ,i_left  = min( (range_left [i_left ],i_left ) for i_left  in range(len(range_left )) )
    
        
    
# Initialize all variables
range_front = []
range_right = []
range_left  = []
min_front = 0
i_front = 0
min_right = 0
i_right = 0
min_left = 0
i_left = 0
goal_pose = 0.0
pos = 0.0

# Create the node
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) # to move the robot
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)   # to read the laser scanner
sodom = rospy.Subscriber('odom', Odometry, save_goalpos)
rospy.init_node('maze_explorer')

command = Twist()
command.linear.x = 0.0
command.angular.z = 0.0
        
rate = rospy.Rate(10)
time.sleep(1) # wait for node to initialize

near_wall = 0 # start with 0, when we get to a wall, change to 1, 3 for outer area/exit

command.angular.z = 0.9 #left
command.linear.x = 0.2
cmd_vel_pub.publish(command)
time.sleep(2)
       
while not rospy.is_shutdown():
    if(min_front > 10.0 and min_right > 5 and min_left > 5 and near_wall == 1 ): 
            near_wall = 3
            command.linear.x = -0.5
            for i in range(4):
                cmd_vel_pub.publish(command)
                rospy.sleep(0.15)
            command.angular.z = -2.5 #-3.17 #-1.57
            
            for i in range(4):
                cmd_vel_pub.publish(command)
                rospy.sleep(0.25) 
            
            command.angular.z = 0.0
            cmd_vel_pub.publish(command)
            command.angular.x = 0.2
            is_goal_visited = 1 

    while(near_wall == 0 and not rospy.is_shutdown()): #1
        is_goal_visited = 0
        if(min_front > 0.2 and min_right > 0.2 and min_left > 0.2):    
            command.angular.z = -0.1    # if nothing near, go forward
            command.linear.x = 0.15
        elif(min_left < 0.2):           # if wall on left, start tracking
            near_wall = 1           
        else:
            command.angular.z = -0.25   # if not on left, turn right 
            command.linear.x = 0.0

        cmd_vel_pub.publish(command)
        
    else:   # left wall detected
        if near_wall == 3:
            command.linear.x = 0.10
            if(min_left < 0.2):
                near_wall = 1
            else:
                command.angular.z = 0.0
                
        else:
            if(min_front > 0.2): #2
                if(min_left < 0.15 and min_front < 0.10):    #3
                    print(("Range: {:.2f}m - Too close. Backing up.".format(min_left)))
                    command.angular.z = -0.5
                    command.linear.x = -0.1
                elif(min_left < 0.15):
                    command.angular.z = -0.5
                    command.linear.x = 0.1
                elif(min_left > 0.20):  #4
                    print(("Range: {:.2f}m - Wall-following; turn left.".format(min_left)))
                    command.angular.z = 0.3
                    command.linear.x = 0.15
                else:
                    print(("Range: {:.2f}m - Wall-following; turn right.".format(min_left)))
                    command.angular.z = -0.3
                    command.linear.x = 0.15
                    
            else:   #5
                #print("Front obstacle detected. Turning away.")
                command.angular.z = -1.0
                command.linear.x = 0.0
                cmd_vel_pub.publish(command)
                while(min_front < 0.3 and not rospy.is_shutdown()):      
                    pass
                
        # publish command 
        cmd_vel_pub.publish(command)
    # wait for the loop
    rate.sleep()
    