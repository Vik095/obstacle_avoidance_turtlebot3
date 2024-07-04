#!/usr/bin/env python3
# Author: Vik Kanagavel Chithra
# Units: Meters
import math
import rospy  # Python library for ROS
from sensor_msgs.msg import LaserScan  # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from tf.transformations import euler_from_quaternion

class Nav:
    def __init__(self):
        self.curr_pos = Point()
        self.target_pos = Point()

        self.target_pos.x = 1.0
        self.target_pos.y = 0.0
        self.target_pos.z = 0.0

        self.target_orientation = Quaternion()
        self.curr_orientation = Quaternion()
        self.orient_tol = 0.087

        self.scan_data = []
        self.lidar_thr = 2
        self.clearance_width = 0.3
        self.current_yaw = 0.0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.Subscriber("/odom", Odometry, self.callback_odom)

        self.obstacle_detected = False

    def quaternion_to_euler(self, dir):
        x, y, z, w = dir.x, dir.y, dir.z, dir.w
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        print(yaw)
        return roll, pitch, yaw
    def control_UGV(self, u_lin, u_ang):
        twist = Twist()
        twist.linear.x = u_lin
        twist.angular.z = u_ang
        self.pub.publish(twist)
    def turn(self, angle):
        initial_or = self.current_yaw
        target_yaw = initial_or + angle
        
        if target_yaw < 360:
            while self.current_yaw < target_yaw:
                self.control_UGV(0, 0.1)
        else:
            while self.current_yaw < target_yaw % 360:
                self.control_UGV(0, 0.1)

    def callback_scan(self, dt):
        self.scan_data = dt.ranges
        if dt.ranges[0] < self.lidar_thr and dt.ranges[3] < self.lidar_thr and dt.ranges[-3] < self.lidar_thr:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        while(self.scan_data[0]<10000):
            self.turn(2)
        
        

    def callback_odom(self, dt):
        self.curr_pos.x = dt.pose.pose.position.x
        self.curr_pos.y = dt.pose.pose.position.y
        self.curr_pos.z = dt.pose.pose.position.z
        self.curr_orientation = dt.pose.pose.orientation
        orientation_q = dt.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    nav = Nav()
    rospy.spin()
