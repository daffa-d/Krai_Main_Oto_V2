/*
 * rosHandler.cpp
 *
 *  Created on: Feb 12, 2024
 *      Author: daffa
 */

#include "rosHandler.h"
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <kin.h>

float xtarget;
float ytarget;
float thtarget;
float Aksendbg[3];
float errorPub;
float InvTarget[3];
bool stateInv;

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
ros::Subscriber<geometry_msgs::Vector3> kinematic("robot/target_kinematic", &kinCallback);
ros::Subscriber<geometry_msgs::Vector3> invKinematic("robot/inv_target_kinematic", &invkinCallback);
ros::Subscriber<std_msgs::Bool> stateInv_Sub("robot/stateInv", &stateInverseKin);
ros::Publisher errorArr("robot/Error_Aksen", &error_arr_msg);
//ros::Publisher AksenPub("robot/aksen", &aksenMsg);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void AksenPublish();
void errorArrPublish();

void setup(void) {
	nh.initNode();
//	nh.advertise(AksenPub);
	nh.advertise(errorArr);
	nh.subscribe(invKinematic);
	nh.subscribe(kinematic);
	nh.subscribe(stateInv_Sub);
//	nh.negotiateTopics();
	HAL_Delay(100);
}

void loop(){
	errorArrPublish();
//	AksenPublish();
	nh.spinOnce();
	HAL_Delay(10);
}

void AksenPublish(){
	aksenMsg.x = Aksendbg[0];
	aksenMsg.y = Aksendbg[1];
	aksenMsg.z = Aksendbg[2];
//	AksenPub.publish(&aksenMsg);
}

void errorArrPublish(){
	error_arr_msg.data = errorPub;
	errorArr.publish(&error_arr_msg);
}
