/*
 * kin.cpp
 *
 *  Created on: Dec 18, 2023
 *      Author: DaffaD
 */

#include <kin.h>
#include <rosHandler.h>

int yawVal;
int out[4];
int16_t rpmExt[3];
float outDot[3];
float Aksen[3];
unsigned long prevTime;
double last_outDot[3];
float error_arrived; // Variable only kin.cpp
//float scale1;
//float scale2;

//void ForwardKin(){
//	outDot[0] = cos(d2r(ENC_1)) * rpmExt[0] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[2]; // X
//	outDot[1] = sin(d2r(ENC_1)) * rpmExt[0] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[2]; // Y
//	outDot[2] = lengthAlpha * rpmExt[0] + lengthAlpha * rpmExt[1] + lengthAlpha * rpmExt[2]; // TH
//
//	if(outDot[0] >= 0) scale1 = 0.00000438;
//	else scale1 = 0.00000438;
//	if(outDot[1] >= 0) scale2 = 0.00000214;
//	else scale2 = 0.0000021;
//
//	last_outDot[0] = outDot[0];
//	last_outDot[1] = outDot[1];
//
//	outDot[0] = cos(d2r(yawVal)) * (scale1 * outDot[0]) + -sin(d2r(yawVal)) * (scale2 * outDot[1]);
//	outDot[0] = sin(d2r(yawVal)) * (scale1 * outDot[0]) + cos(d2r(yawVal)) * (scale2 * outDot[1]);
//
////	outDot[0] = cos(d2r(yawVal)) * (scale1 * (cos(d2r(ENC_1)) * rpmExt[0] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[2])) +
////					-sin(d2r(yawVal)) * (scale2 * (sin(d2r(ENC_1)) * rpmExt[0] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[2])); // X
////	outDot[1] = sin(d2r(yawVal)) * (scale1 * (cos(d2r(ENC_1)) * rpmExt[0] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[2])) +
////					cos(d2r(yawVal)) * (scale2 * (sin(d2r(ENC_1)) * rpmExt[0] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[2])); // Y
////	outDot[2] = lengthAlpha * rpmExt[0] + lengthAlpha * rpmExt[1] + lengthAlpha * rpmExt[2]; // TH
//
//
//	if(HAL_GetTick() - prevTime >= 100){
//		Aksen[0] = Aksen[0] + outDot[0] * 100; // X
//		Aksen[1] = Aksen[1] + outDot[1] * 100; // Y
//	//	Aksen[2] = (Aksen[2] + outDot[2] * 100) * scale3; // theta
//		Aksen[2] = yawVal; // theta diambil dari heading imu
//		prevTime = HAL_GetTick();
//	}
//}


vector3Kin ForwardKin(float xStar, float yStar, float thStar){
	vector3Kin calOut = {
			.x = 0, .y = 0, .th = 0
	};

	outDot[0] = cos(d2r(yawVal)) * (cos(d2r(ENC_1)) * rpmExt[0] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[2]) +
				-sin(d2r(yawVal)) * (sin(d2r(ENC_1)) * rpmExt[0] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[2]); // X
	outDot[1] = sin(d2r(yawVal)) * (cos(d2r(ENC_1)) * rpmExt[0] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[2]) +
				cos(d2r(yawVal)) * (sin(d2r(ENC_1)) * rpmExt[0] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[2]); // Y
	outDot[2] = lengthAlpha * rpmExt[0] + lengthAlpha * rpmExt[1] + lengthAlpha * rpmExt[2]; // TH

	if(HAL_GetTick() - prevTime >= 100){
		Aksen[0] = Aksen[0] + outDot[0] * 100 * scale1; // X
		Aksen[1] = Aksen[1] + outDot[1] * 100 * scale2; // Y
	//	Aksen[2] = (Aksen[2] + outDot[2] * 100) * scale3; // theta
		Aksen[2] = yawVal; // theta diambil dari heading imu
		prevTime = HAL_GetTick();
	}

	calOut.x = xStar - Aksen[0];
	calOut.y = yStar - Aksen[1];
	calOut.th = thStar - Aksen[2];
	return calOut;
}

MotorKin InverseKin(vector3Kin *calOut){
	MotorKin mtr = {
			.w1=0, .w2=0, .w3=0, .w4=0
	};

	error_arrived = sqrt(pow(calOut->x, 2) + pow(calOut->y, 2) + pow(calOut->th, 2));
	errorPub = error_arrived;

	if(error_arrived < 0.8)
	{
		calOut->x = 0;
		calOut->y = 0;
		calOut->th = 0;
	}
	else
	{
		kinMotor_V3(&mtr, calOut->x, calOut->y, calOut->th);
	}

	return mtr;
}

void kinMotor_V3(MotorKin *mtrKin, float Ex, float Ey, float Eth)
{
	mtrKin->w1 = (lambdaX * cos(d2r(135)) * (cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey)) +
				 (lambdaY * sin(d2r(135)) * (-sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey)) +
				 (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w2 = (lambdaX * cos(d2r(-135)) * (cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey)) +
				 (lambdaY * sin(d2r(-135)) * (-sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey)) +
				 (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w3 = (lambdaX * cos(d2r(-45)) * (cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey)) +
			     (lambdaY * sin(d2r(-45)) * (-sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey)) +
				 (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w4 = (lambdaX * cos(d2r(45)) * (cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey)) +
			     (lambdaY * sin(d2r(45)) * (-sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey)) +
			     (lambdaTH * alphaLengthMotor * Eth);

	if(mtrKin->w1 > Max_Cutoff_Mtr) mtrKin->w1 = Max_Cutoff_Mtr;
	else if(mtrKin->w1 < Min_Cutoff_Mtr) mtrKin->w1 = Min_Cutoff_Mtr;

	if(mtrKin->w2 > Max_Cutoff_Mtr) mtrKin->w2 = Max_Cutoff_Mtr;
	else if(mtrKin->w2 < Min_Cutoff_Mtr) mtrKin->w2 = Min_Cutoff_Mtr;

	if(mtrKin->w3 > Max_Cutoff_Mtr) mtrKin->w3 = Max_Cutoff_Mtr;
	else if(mtrKin->w3 < Min_Cutoff_Mtr) mtrKin->w3 = Min_Cutoff_Mtr;

	if(mtrKin->w4 > Max_Cutoff_Mtr) mtrKin->w4 = Max_Cutoff_Mtr;
	else if(mtrKin->w4 < Min_Cutoff_Mtr) mtrKin->w4 = Min_Cutoff_Mtr;
}

void kinMotor_V2(MotorKin *mtrKin, float Ex, float Ey, float Eth)
{
	mtrKin->w1 = (lambdaX * cos(d2r(135)) * cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey) +
				 (lambdaY * sin(d2r(135)) * -sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey) +
				 (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w2 = (lambdaX * cos(d2r(-135)) * cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey) +
				 (lambdaY * sin(d2r(-135)) * -sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey) +
			     (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w3 = (lambdaX * cos(d2r(-45)) * cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey) +
				 (lambdaY * sin(d2r(-45)) * -sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey) +
			     (lambdaTH * alphaLengthMotor * Eth);
	mtrKin->w4 = (lambdaX * cos(d2r(45)) * cos(d2r(yawVal)) * Ex + sin(d2r(yawVal)) * Ey) +
				 (lambdaY * sin(d2r(45)) * -sin(d2r(yawVal)) * Ex + cos(d2r(yawVal)) * Ey) +
				 (lambdaTH * alphaLengthMotor * Eth);

	if(mtrKin->w1 > Max_Cutoff_Mtr) mtrKin->w1 = Max_Cutoff_Mtr;
	else if(mtrKin->w1 < Min_Cutoff_Mtr) mtrKin->w1 = Min_Cutoff_Mtr;

	if(mtrKin->w2 > Max_Cutoff_Mtr) mtrKin->w2 = Max_Cutoff_Mtr;
	else if(mtrKin->w2 < Min_Cutoff_Mtr) mtrKin->w2 = Min_Cutoff_Mtr;

	if(mtrKin->w3 > Max_Cutoff_Mtr) mtrKin->w3 = Max_Cutoff_Mtr;
	else if(mtrKin->w3 < Min_Cutoff_Mtr) mtrKin->w3 = Min_Cutoff_Mtr;

	if(mtrKin->w4 > Max_Cutoff_Mtr) mtrKin->w4 = Max_Cutoff_Mtr;
	else if(mtrKin->w4 < Min_Cutoff_Mtr) mtrKin->w4 = Min_Cutoff_Mtr;
}

void kinMotor(MotorKin *mtrKin, float x, float y, float th) {
	mtrKin->w1 = (lambdaInv_X * cos(d2r(135)) * x) +
				 (lambdaInv_Y * sin(d2r(135)) * y) +
				 (lambdaInv_TH * lengthAlpha * th);
	mtrKin->w2 = (lambdaInv_X * cos(d2r(-135)) * x) +
				 (lambdaInv_Y * sin(d2r(-135)) * y) +
				 (lambdaInv_TH * lengthAlpha * th);
	mtrKin->w3 = (lambdaInv_X * cos(d2r(-45)) * x) +
			     (lambdaInv_Y * sin(d2r(-45)) * y) +
				 (lambdaInv_TH * lengthAlpha * th);
	mtrKin->w4 = (lambdaInv_X * cos(d2r(45)) * x) +
				 (lambdaInv_Y * sin(d2r(45)) * y) +
				 (lambdaInv_TH * lengthAlpha * th);

	if(mtrKin->w1 > Max_Cutoff_Mtr_Inv) mtrKin->w1 = Max_Cutoff_Mtr_Inv;
	else if(mtrKin->w1 < Min_Cutoff_Mtr_Inv) mtrKin->w1 = Min_Cutoff_Mtr_Inv;

	if(mtrKin->w2 > Max_Cutoff_Mtr_Inv) mtrKin->w2 = Max_Cutoff_Mtr_Inv;
	else if(mtrKin->w2 < Min_Cutoff_Mtr_Inv) mtrKin->w2 = Min_Cutoff_Mtr_Inv;

	if(mtrKin->w3 > Max_Cutoff_Mtr_Inv) mtrKin->w3 = Max_Cutoff_Mtr_Inv;
	else if(mtrKin->w3 < Min_Cutoff_Mtr_Inv) mtrKin->w3 = Min_Cutoff_Mtr_Inv;

	if(mtrKin->w4 > Max_Cutoff_Mtr_Inv) mtrKin->w4 = Max_Cutoff_Mtr_Inv;
	else if(mtrKin->w4 < Min_Cutoff_Mtr_Inv) mtrKin->w4 = Min_Cutoff_Mtr_Inv;
}

//int* calculateFKin(int xStar, int yStar, int thStar) {
//	static int valueCal[3];
//
//	outDot[0] = cos(d2r(ENC_1)) * rpmExt[2] + cos(d2r(ENC_2)) * rpmExt[1] + cos(d2r(ENC_3)) * rpmExt[0]; // X
//	outDot[1] = sin(d2r(ENC_1)) * rpmExt[2] + sin(d2r(ENC_2)) * rpmExt[1] + sin(d2r(ENC_3)) * rpmExt[0]; // Y
//	outDot[2] = lengthAlpha * rpmExt[2] + lengthAlpha * rpmExt[1] + lengthAlpha * rpmExt[0]; // TH
//
////	currTime = HAL_GetTick();
////	Aksen[0] = Aksen[0] + outDot[0] * (currTime - prevTime); // X
////	Aksen[1] = Aksen[1] + outDot[1] * (currTime - prevTime); // Y
////	Aksen[2] = Aksen[2] + outDot[2] * (currTime - prevTime); // TH
////	prevTime = currTime;
//
//	if (HAL_GetTick() - prevTime >= 100) {
//		Aksen[0] = Aksen[0] + outDot[0]; // X
//		Aksen[1] = Aksen[1] + outDot[1]; // Y
//		Aksen[2] = Aksen[2] + outDot[2];
//		prevTime = HAL_GetTick();
//	}
//
//	valueCal[0] = xStar - Aksen[0];
//	valueCal[1] = yStar - Aksen[1];
//	valueCal[2] = thStar - Aksen[2];
//
//	return valueCal;
//}