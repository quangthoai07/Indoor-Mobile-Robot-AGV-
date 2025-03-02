#!/usr/bin/env python3

import rospy
import message_filters
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from localizer_dwm1001.msg import Tag
import csv
import time
from math import atan2

class DataLogger:
    def __init__(self):
        self.vel_sub = rospy.Subscriber('/vel_pub', Twist, self.vel_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Vector3, self.imu_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom_ekf', Odometry, self.odom_callback, queue_size=1)
        self.uwb_sub = rospy.Subscriber('/dwm1001/tag', Tag, self.uwb_callback, queue_size=1)

        self.vel_file = open('vel_data.csv', 'w')
        self.imu_file = open('imu_data.csv', 'w')
        self.odom_file = open('odom_data.csv', 'w')
        self.uwb_file = open('uwb_data.csv', 'w')

        self.vel_writer = csv.writer(self.vel_file)
        self.imu_writer = csv.writer(self.imu_file)
        self.odom_writer = csv.writer(self.odom_file)
        self.uwb_writer = csv.writer(self.uwb_file)

        # Headers
        self.vel_writer.writerow(['timestamp', 'linear_x', 'angular_z'])
        self.imu_writer.writerow(['timestamp', 'z'])
        self.odom_writer.writerow(['timestamp', 'x', 'y', 'yaw'])
        self.uwb_writer.writerow(['timestamp', 'x', 'y'])

    def vel_callback(self, msg):
        self.vel_writer.writerow([rospy.Time.now().to_sec(), msg.linear.x, msg.angular.z])

    def imu_callback(self, msg):
        self.imu_writer.writerow([rospy.Time.now().to_sec(), msg.z])

    def odom_callback(self, msg):
        yaw = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odom_writer.writerow([rospy.Time.now().to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def uwb_callback(self, msg):
        self.uwb_writer.writerow([rospy.Time.now().to_sec(), msg.x, msg.y])

    def close_files(self):
        self.vel_file.close()
        self.imu_file.close()
        self.odom_file.close()
        self.uwb_file.close()

if __name__ == '__main__':
    rospy.init_node('data_logger')
    logger = DataLogger()
    rospy.spin()
    logger.close_files()
