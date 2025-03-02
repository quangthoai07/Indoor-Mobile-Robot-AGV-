#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from localizer_dwm1001.msg import Tag
import csv
import time
from math import sin, cos

class DataPlayer:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/vel_pub', Twist, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Vector3, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom_ekf', Odometry, queue_size=10)
        self.uwb_pub = rospy.Publisher('/uwb_data', Tag, queue_size=10)

        self.vel_data = self.load_csv('vel_data.csv')
        self.imu_data = self.load_csv('imu_data.csv')
        self.odom_data = self.load_csv('odom_data.csv')
        self.uwb_data = self.load_csv('uwb_data.csv')

        self.vel_timer = rospy.Timer(rospy.Duration(1.0/75), self.vel_callback)
        self.imu_timer = rospy.Timer(rospy.Duration(1.0/75), self.imu_callback)
        self.odom_timer = rospy.Timer(rospy.Duration(1.0/50), self.odom_callback)
        self.uwb_timer = rospy.Timer(rospy.Duration(1.0/2), self.uwb_callback)

        self.vel_idx = 0
        self.imu_idx = 0
        self.odom_idx = 0
        self.uwb_idx = 0

        self.start_time = rospy.get_time()

    def load_csv(self, filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            return list(reader)

    def vel_callback(self, event):
        if self.vel_idx < len(self.vel_data):
            current_time = rospy.get_time() - self.start_time
            if float(self.vel_data[self.vel_idx][0]) <= current_time:
                msg = Twist()
                msg.linear.x = float(self.vel_data[self.vel_idx][1])
                msg.angular.z = float(self.vel_data[self.vel_idx][2])
                self.vel_pub.publish(msg)
                self.vel_idx += 1

    def imu_callback(self, event):
        if self.imu_idx < len(self.imu_data):
            current_time = rospy.get_time() - self.start_time
            if float(self.imu_data[self.imu_idx][0]) <= current_time:
                msg = Vector3()
                msg.z = float(self.imu_data[self.imu_idx][1])
                self.imu_pub.publish(msg)
                self.imu_idx += 1

    def odom_callback(self, event):
        if self.odom_idx < len(self.odom_data):
            current_time = rospy.get_time() - self.start_time
            if float(self.odom_data[self.odom_idx][0]) <= current_time:
                msg = Odometry()
                msg.pose.pose.position.x = float(self.odom_data[self.odom_idx][1])
                msg.pose.pose.position.y = float(self.odom_data[self.odom_idx][2])
                yaw = float(self.odom_data[self.odom_idx][3])
                msg.pose.pose.orientation.z = sin(yaw/2)
                msg.pose.pose.orientation.w = cos(yaw/2)
                self.odom_pub.publish(msg)
                self.odom_idx += 1

    def uwb_callback(self, event):
        if self.uwb_idx < len(self.uwb_data):
            current_time = rospy.get_time() - self.start_time
            if float(self.uwb_data[self.uwb_idx][0]) <= current_time:
                msg = Tag()
                msg.x, msg.y = map(float, self.uwb_data[self.uwb_idx][1:3])
                self.uwb_pub.publish(msg)
                self.uwb_idx += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('data_player')
    player = DataPlayer()
    player.run()
