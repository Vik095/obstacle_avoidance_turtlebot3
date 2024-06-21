#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point
import numpy as np


curr_pos = Point()


target = Point()

target.x= 1
target.y=0
target.z=0


def controlUGV(u_lin, u_ang):
    twist =Twist()
    twist.linear.x = u_lin
    twist.angular.z=u_ang
    pub.publish(Twist())



def callback_scan(dt):
    
    print ('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print ('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print ('Range data at 345 deg: {}'.format(dt.ranges[345]))
    min_angle = dt.angle_min
    max_angle = dt.angle_max
    step = dt.angle_increment

    thr1 = 0.8 # Laser scan range threshold
    thr2 = 0.8
    if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: 
        move.linear.x = 0.5 
        move.angular.z = 0.0
    else:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.5
        if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
            move.linear.x = 0.5
            move.angular.z = 0.0
    pub.publish(move) 

def callback_odom(dt):
    curr_pos.x = dt.pose.pose.position.x
    curr_pos.y= dt.pose.pose.position.y
    curr_pos.z = dt.pose.pose.position.z
    if(target.x==1):
        controlUGV(0.5,0.5)


move = Twist() 
rospy.init_node('obstacle_avoidance_node') 
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
#sub = rospy.Subscriber("/scan", LaserScan, callback_scan) 
pos_sub = rospy.Subscriber("/odom", Odometry, callback_odom)


rospy.spin() 

