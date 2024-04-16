/*
 * rosHandler.cpp
 *
 *  Created on: Feb 12, 2024
 *      Author: daffa
 */

#include "rosHandler.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <kin.h>

float xtarget;
float ytarget;
float thtarget;
float Aksendbg[3];
float errorPub;
float InvTarget[3];
float msg_imu[10];
float msg_odom[7];
bool stateInv;
int16_t rawYaw;

void kinCallback(const geometry_msgs::Vector3 &data){
	xtarget = data.x;
	ytarget = data.y;
	thtarget = data.z;
}

void invkinCallback(const geometry_msgs::Vector3 &data){
	InvTarget[0] = data.x;
	InvTarget[1] = data.y;
	InvTarget[2] = data.z;
}

void stateInverseKin(const std_msgs::Bool &data){
	stateInv = data.data;
}

ros::NodeHandle nh;

geometry_msgs::Vector3 kinMsg;
geometry_msgs::Vector3 KinTarget_msg;
geometry_msgs::Vector3 aksenMsg;
geometry_msgs::Quaternion sensMsg;

std_msgs::Bool stateInv_msg;
std_msgs::Float32 error_arr_msg;
//std_msgs::Float32MultiArray imuData;
//std_msgs::Float32MultiArray odomData;

ros::Subscriber<geometry_msgs::Vector3> kinematic("robot/target_kinematic", &kinCallback);
ros::Subscriber<geometry_msgs::Vector3> invKinematic("robot/inv_target_kinematic", &invkinCallback);
ros::Subscriber<std_msgs::Bool> stateInv_Sub("robot/stateInv", &stateInverseKin);
ros::Publisher errorArr("robot/Error_Aksen", &error_arr_msg);
//ros::Publisher imu_pub("robot/imu", &imuData);
//ros::Publisher odom_pub("robot/odom", &odomData);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void errorArrPublish();
void imuPublish();
void odomPublish();

void setup(void) {
	nh.initNode();
	nh.advertise(errorArr); // error arrived
//	nh.advertise(imu_pub);
//	nh.advertise(odom_pub);
	nh.subscribe(invKinematic); // inverse kinematic
	nh.subscribe(kinematic); // forward kinematic
	nh.subscribe(stateInv_Sub); // diaktifkan apabila menggunakan inverse kinematic
//	nh.negotiateTopics();
	HAL_Delay(100);
}

void loop(){
	errorArrPublish();
//	imuPublish();
//	odomPublish();
	nh.spinOnce();
	HAL_Delay(10);
}

void errorArrPublish(){
	error_arr_msg.data = errorPub;
	errorArr.publish(&error_arr_msg);
}

/*

void imuPublish(bno055_vector_t* quat, bno055_vector_t* line, bno055_vector_t* gyro){
	msg_imu[0] = quat->w;
	msg_imu[1] = quat->x;
	msg_imu[2] = quat->y;
	msg_imu[3] = quat->z;

	msg_imu[4] = line->x;
	msg_imu[5] = line->y;
	msg_imu[6] = line->z;

	msg_imu[7] = gyro->x;
	msg_imu[8] = gyro->y;
	msg_imu[9] = gyro->z;

	imuData.data = msg_imu;
	imuData.data_length = 10;
	imu_pub.publish(&imuData);
}

void odomPublish(){
	msg_odom[0] = Aksen[0];
	msg_odom[1] = Aksen[1];
	msg_odom[2] = Aksen[2];

	msg_odom[3] = velo[0];
	msg_odom[4] = velo[1];
	msg_odom[5] = velo[2];

	msg_odom[6] = rawYaw;

	odomData.data = msg_odom;
	odomData.data_length = 7;
	odom_pub.publish(&odomData);
}

*/
