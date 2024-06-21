#!/usr/bin/env python3
import rospy  # Python library for ROS
from sensor_msgs.msg import LaserScan  # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np


class Nav:
    def __init__(self):
        self.curr_pos = Point()
        self.target = Point()
        self.orientation = Quaternion()

        self.target.x = 1
        self.target.y = 0
        self.target.z = 0

        self.move = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        rospy.Subscriber("/odom", Odometry, self.callback_odom)

        self.obstacle_detected = False

    def controlUGV(self, u_lin, u_ang):
        twist = Twist()
        twist.linear.x = u_lin
        twist.angular.z = u_ang
        self.pub.publish(twist)

    def callback_scan(self, dt):
        print('Range data at 0 deg:   {}'.format(dt.ranges[0]))
        print('Range data at 15 deg:  {}'.format(dt.ranges[15]))
        print('Range data at 345 deg: {}'.format(dt.ranges[345]))

        thr1 = 0.8  # Laser scan range threshold
        if dt.ranges[0] < thr1:
            self.obstacle_detected = True
            self.move.linear.x = -1
            self.move.angular.z = 0.0
        else:
            self.obstacle_detected = False

    def callback_odom(self, dt):
        self.curr_pos.x = dt.pose.pose.position.x
        self.curr_pos.y = dt.pose.pose.position.y
        self.curr_pos.z = dt.pose.pose.position.z

        if not self.obstacle_detected and self.curr_pos.x < 1:
            if (self.target.x - self.curr_pos.x > 0.2):
                self.controlUGV(1, 0)
            else:
                self.controlUGV(0, 0)
        else:
            self.controlUGV(0, 0)


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    nav = Nav()
    rospy.spin()
