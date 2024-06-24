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


class Nav:
    def __init__(self):
        self.curr_pos = Point()
        self.target_pos = Point()

        self.target_pos.x = 1.0
        self.target_pos.y = 0.0
        self.target_pos.z = 0.0

        self.target_orientation = Quaternion()
        self.curr_orientation = Quaternion()
        self.orient_tol = 0.02

        self.scan_data = []
        self.lidar_thr = 0.8
        self.clearance_width = 0.3

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.Subscriber("/odom", Odometry, self.callback_odom)

        self.obstacle_detected = False

    def quaternion_to_euler(self, dir):
        x, y, z, w = dir.x, dir.y, dir.z, dir.w
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def angle_off(self):
        delta_x = self.target_pos.x - self.curr_pos.x
        delta_y = self.target_pos.y - self.curr_pos.y
        target_angle = math.atan2(delta_y, delta_x)

        (roll, pitch, yaw) = self.quaternion_to_euler(self.curr_orientation)
        angle_diff = target_angle - yaw

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        

        return angle_diff

    def orient(self):
        target_angle = self.angle_off()

        while abs(target_angle) > self.orient_tol:
            angular_velocity = max(min(target_angle, 0.1), -0.1)  # Limit angular velocity

            self.control_UGV(0, angular_velocity)
            rospy.sleep(0.1)

            target_angle = self.angle_off()

        self.control_UGV(0, 0)  # Stop the robot once orientation is achieved

    def control_UGV(self, u_lin, u_ang):
        twist = Twist()
        twist.linear.x = u_lin
        twist.angular.z = u_ang
        self.pub.publish(twist)

    def move(self):
        initial_x = self.curr_pos.x
        initial_y = self.curr_pos.y

        while True:
            self.control_UGV(0.1, 0)
            rospy.sleep(0.1)

            current_x = self.curr_pos.x
            current_y = self.curr_pos.y

            distance_moved = math.sqrt((current_x - initial_x)**2 + (current_y - initial_y)**2)

            if distance_moved >= 0.2:  # 20 cm in meters
                break

    def callback_scan(self, dt):
        self.scan_data = dt.ranges
        if dt.ranges[0] < self.lidar_thr:
            self.obstacle_detected = True
            self.control_UGV(0, 0.2)  # Adjust for obstacle avoidance
        else:
            self.obstacle_detected = False
            self.move()
            self.orient()

    def callback_odom(self, dt):
        self.curr_pos.x = dt.pose.pose.position.x
        self.curr_pos.y = dt.pose.pose.position.y
        self.curr_pos.z = dt.pose.pose.position.z
        self.curr_orientation = dt.pose.pose.orientation


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    nav = Nav()
    rospy.spin()
