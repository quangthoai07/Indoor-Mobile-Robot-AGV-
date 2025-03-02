#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
import paho.mqtt.client as mqtt
import subprocess
import os
import math
from nav_msgs.msg import Odometry
import yaml

# Hàm callback khi nhận được thông điệp từ MQTT cho lệnh
def on_message(client, userdata, msg):
    command = msg.payload.decode()
    rospy.loginfo("Received command: %s", command)
    
    if command.startswith("save_map"):
        # Phân tách lệnh và tên tệp từ thông điệp
        parts = command.split(',')
        if len(parts) > 1:
            map_name = parts[1]
        else:
            # Nếu không có tên tệp, sử dụng tên mặc định
            map_name = "test"
        
        rospy.loginfo("Executing command: save_map")
        # Di chuyển đến thư mục chứa bản đồ
        os.chdir(os.path.expanduser("~/catkin_ws/src/ros_mobile_robot/maps"))
        
        # Thực thi lệnh lưu bản đồ với tên tệp đã chỉ định
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", map_name])
        subprocess.call(["rosnode", "kill", "/node_teleop"])
        
    elif command.startswith("rename"):
        # Phân tách lệnh và tên tệp từ thông điệp
        parts = command.split(',')
        if len(parts) > 1:
            new_map_name = parts[1]
        else:
            # Nếu không có tên tệp, sử dụng tên mặc định
            new_map_name = "test0"

        rospy.loginfo(f"Renaming map to: {new_map_name}")

        # Di chuyển đến thư mục chứa bản đồ
        os.chdir(os.path.expanduser("~/catkin_ws/src/ros_mobile_robot/maps"))

        # Thay đổi tên file ảnh trong nội dung YAML
        yaml_file = "name_map.yaml"
        with open(yaml_file, 'r') as file:
            content = file.read()

        # Thay đổi tên file trong nội dung YAML
        content = content.replace("test.pgm", f"{new_map_name}.pgm")

        with open(yaml_file, 'w') as file:
            file.write(content)
        
    elif command == "slam":
        rospy.loginfo("Executing command: start_slam")
        # Thực thi lệnh khởi động SLAM
        subprocess.Popen(['gnome-terminal', '--', 'roslaunch', 'ros_mobile_robot', 'slam_new.launch'])   
    
    elif command == "stop_slam":
        rospy.loginfo("Stopping Slam and Rviz")
        # Dừng các node liên quan đến navigation
        subprocess.call(["rosnode", "kill", "/rviz"])
        subprocess.call(["rosnode", "kill", "/my_robot_slam_gmapping"])
        subprocess.call(["rosnode", "kill", "/odom_pub"])
        subprocess.call(["rosnode", "kill", "/robot_state_publisher"])
        subprocess.call(["rosnode", "kill", "/joint_state_publisher"])                     
   
    elif command == "stop_navi":
        rospy.loginfo("Stopping Navigation and Rviz")
        # Dừng các node liên quan đến navigation
        subprocess.call(["rosnode", "kill", "/rviz"])

    elif command == "navi":
        rospy.loginfo("Starting Navigation")
        # Chay navigation
        subprocess.Popen(['gnome-terminal', '--', 'roslaunch', 'ros_mobile_robot', 'navigation.launch'])
            
    elif command.startswith("update"):
        # Phân tách lệnh và tên tệp từ thông điệp
        parts = command.split(',')
        if len(parts) == 3:
            try:
                max_x = float(parts[1])
                min_x = float(parts[2])
                update_yaml_file(max_x, min_x)
        # Sau đó khởi chạy lại file move_base_test.launch
                move_base_script = os.path.join(os.path.dirname(__file__), '..', 'launch', 'move_base_test.launch')
                subprocess.Popen(['roslaunch', move_base_script])
                rospy.loginfo("Updated YAML file with max_vel_x=%f, min_vel_x=%f", max_x, min_x)
            except ValueError:
                rospy.logerr("Invalid velocity values: %s", command)            
    else:
        rospy.logwarn("Unknown command received: %s", command)

# Đường dẫn đến file YAML
YAML_FILE_PATH = os.path.expanduser("~/catkin_ws/src/ros_mobile_robot/config/param/teb_local_planner_params.yaml")

def update_yaml_file(max_x, min_x):
    # Đọc nội dung file YAML
    with open(YAML_FILE_PATH, 'r') as file:
        data = yaml.safe_load(file)
    
    # Cập nhật giá trị
    data['TrajectoryPlannerROS']['max_vel_x'] = max_x
    data['TrajectoryPlannerROS']['min_vel_x'] = min_x

    # Ghi lại nội dung vào file YAML
    with open(YAML_FILE_PATH, 'w') as file:
        yaml.safe_dump(data, file, default_flow_style=False)


# Hàm callback khi nhận được thông điệp từ MQTT cho vận tốc
def on_velocity_message(client, userdata, msg):
    try:
        # Giả sử thông điệp MQTT có dạng "linear,angular"
        linear, angular = map(float, msg.payload.decode().split(','))
        
        # Tạo thông điệp Twist
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular

        # Xuất bản thông điệp lên topic /cmd_vel
        velocity_publisher.publish(velocity_msg)
        rospy.loginfo(f"Published velocity: linear={linear}, angular={angular}")
    except Exception as e:
        rospy.logerr(f"Failed to process velocity message: {e}")

# Hàm callback khi nhận được thông điệp từ MQTT cho điểm đích
def on_goal_message(client, userdata, msg):
    try:
        # Giả sử thông điệp MQTT có dạng "x,y,heading"
        x, y, heading = map(float, msg.payload.decode().split(','))
        
        # Chuyển đổi góc từ độ sang radian
        heading_radian = math.radians(heading)
        
        # Tạo thông điệp PoseStamped
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = math.sin(heading / 2)  # Chuyển đổi heading thành quaternion
        goal_msg.pose.orientation.w = math.cos(heading / 2)
        
        # Xuất bản thông điệp lên topic /move_base_simple/goal
        goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Published goal: x={x}, y={y}, heading={heading}")
    except Exception as e:
        rospy.logerr(f"Failed to process goal message: {e}")

def odom_callback(msg):
    # Lấy dữ liệu từ Odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    # Chuyển đổi quaternion thành góc Euler
    _, _, yaw = euler_from_quaternion(orientation)
    
    # Chuyển đổi radian thành độ
    yaw_degrees = math.degrees(yaw)
    # Làm tròn giá trị đến 1 chữ số thập phân
    x_rounded = round(x, 1)
    y_rounded = round(y, 1)
    degree_rounded = round(yaw_degrees, 1)
    # Tạo thông điệp để gửi qua MQTT
    odom_message = f"{x_rounded},{y_rounded},{degree_rounded}"
    
    # Xuất bản thông điệp lên MQTT
    mqtt_client.publish("robot/odom", odom_message)
    rospy.loginfo(f"Published odom data: {odom_message}")

def main():
    global velocity_publisher, goal_publisher

    # Khởi tạo node ROS
    rospy.init_node('mqtt_to_ros')

    # Tạo publisher cho topic /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Tạo publisher cho topic /move_base_simple/goal
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Thiết lập MQTT
    mqtt_client = mqtt.Client()
    mqtt_client.message_callback_add("robot/commands", on_velocity_message)
    mqtt_client.message_callback_add("robot/goal", on_goal_message)
    mqtt_client.message_callback_add("robot/msg", on_message)
       
    # Đăng ký subscriber cho topic /odom
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    # Kết nối tới broker MQTT################################################################################################
    mqtt_client.connect("192.168.1.3", 1883, 60)
#    mqtt_client.connect("192.168.100.67", 1883, 60)
    # Đăng ký các topic cần lắng nghe
    mqtt_client.subscribe("robot/commands")
    mqtt_client.subscribe("robot/goal")
    mqtt_client.subscribe("robot/msg")
    #mqtt_client.subscribe("robot/odom")
    # Bắt đầu lắng nghe thông điệp từ MQTT
    mqtt_client.loop_start()

    # Duy trì node ROS
    rospy.spin()

if __name__ == '__main__':
    main()

