#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import pow, sqrt
import threading

class RobotController:
    def __init__(self):
        
        # Khởi tạo các biến vị trí
        self.robot_x = 0
        self.robot_y = 0
        
        # Biến kiểm soát
        self.is_active = False
        self.radius = 0
        
        # Đăng ký các publisher và subscriber
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Khởi tạo vị trí góc trên bản đồ
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = 0  # Đặt vị trí trung tâm của khu vực
        self.goal_pose.pose.position.y = 0
        self.goal_pose.pose.orientation.w = 1.0

        # Khởi động thread kiểm tra
        self.check_thread = threading.Thread(target=self.check_position)
        self.check_thread.start()

    # Callback function để lấy vị trí hiện tại của robot
    def odom_callback(self, data):
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y

    # Kiểm tra xem robot có nằm trong khu vực hay không
    def is_within_area(self):
        distance_to_center = sqrt(pow(self.robot_x - self.goal_pose.pose.position.x, 2) + 
                                  pow(self.robot_y - self.goal_pose.pose.position.y, 2))
        return distance_to_center <= self.radius

    # Hàm để kiểm tra liên tục vị trí của robot
    def check_position(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.is_active and not self.is_within_area():
                self.move_to_center()
            rate.sleep()

    # Hàm để di chuyển robot đến vị trí trung tâm của khu vực 
    def move_to_center(self):
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_publisher.publish(self.goal_pose)
        rospy.loginfo("Robot out of bounds. Moving to center.")

    # Hàm để bắt đầu giới hạn
    def start_limiting(self, radius):
        self.radius = radius
        self.is_active = True
        rospy.loginfo(f"Started limiting with radius {radius}")

    # Hàm để dừng giới hạn
    def stop_limiting(self):
        self.is_active = False
        rospy.loginfo("Stopped limiting")

# Khởi tạo controller global
controller = None

def initialize():
    global controller
    controller = RobotController()
    rospy.loginfo("RobotController initialized and ready.")

def start_limit(radius):
    global controller
    if controller:
        controller.start_limiting(radius)
    else:
        rospy.logerr("Controller not initialized. Call initialize() first.")

def stop_limit():
    global controller
    if controller:
        controller.stop_limiting()
    else:
        rospy.logerr("Controller not initialized. Call initialize() first.")

if __name__ == '__main__':
    try:
        initialize()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
