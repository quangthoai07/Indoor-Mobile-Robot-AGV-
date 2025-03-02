/*
 * mainpp.cpp
 *
 *  Created on: Dec 27, 2023
 *      Author: MTQ
 */



#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <std_msgs/Float32.h>
#include <rospy_tutorials/Floats.h>
#include <geometry_msgs/Twist.h>



#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;

//publisher
std_msgs::Int32 num_msg;
std_msgs::String str_msg;

sensor_msgs::Imu imu;

//std_msgs::Float32 vel_msg;
uint8_t nowTick=0;
uint8_t pastTick=0;

int num_test=0;



geometry_msgs::Twist vel_msg;

ros::Publisher num_pub("num", &num_msg);
ros::Publisher chatter("chatter",&str_msg);

ros::Publisher vel_pub("vel_pub",&vel_msg);

ros::Publisher imu_pub("imu", &imu);
char hello[]="Hello World";

//subscriber
void led_cb(const std_msgs::Empty &msg ){
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}




ros::Subscriber<std_msgs::Empty>led_sub("toggle_led",&led_cb);
//ros::Subscriber<std_msgs::Float32>vel_sub("/set_velocity",&velocityCallback);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART6){
		nh.getHardware()->flush();
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART6){
		nh.getHardware()->reset_rbuf();
	}
}

void setup()
{
	nh.initNode();

	nh.advertise(chatter);
	nh.subscribe(led_sub);



}
void loop()
{
	//publish message

	str_msg.data=hello;
	chatter.publish(&str_msg);








//	imu.angular_velocity.x=;
//	imu.angular_velocity.y=;
//	imu.angular_velocity.z=;
//	imu.linear_acceleration.x=imu_data_tx.x;
//	imu.linear_acceleration.y=imu_data_tx.y;
//	imu.linear_acceleration.z=imu_data_tx.z;


//	nowTick=HAL_GetTick();
//	if(nowTick-pastTick>100){
//
//
//		vel_msg.linear.x=vel_data_tx.v;
//		vel_msg.angular.z=vel_data_tx.w;
//		vel_pub.publish(&vel_msg);
//		pastTick=nowTick;
//	}

//	nowTick=HAL_GetTick();
//	if(nowTick-pastTick>100){
//		num_msg.data=num_test;
//		num_pub.publish(&num_msg);
//
//		nowTick=pastTick;
//
//		num_test++;
//	}
	nh.spinOnce();
	HAL_Delay(1000);
}

