/*
 * kin.h
 *
 *  Created on: Dec 18, 2023
 *      Author: DaffaD
 */

#ifndef INC_KIN_H_
#define INC_KIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "stm32f4xx_hal.h"
#include "main.h"

#define lambda 1000 // yang ini gakepake

#define lambdaInv_X 1000
#define lambdaInv_Y 1000
#define lambdaInv_TH 500

#define lambdaX 1200
#define lambdaY 1200
#define lambdaTH 60

#define d2r(x) x*(M_PI/180) // degree to radians
#define lengthAlpha 0.18 // Panjang Encoder
#define alphaLengthMotor 0.26 // Panjang Motor

#define ENC_1 135 // Derajat Encoder
#define ENC_2 45
#define ENC_3 -90

#define scale1 0.00000336 //0.00000336
#define scale2 0.00000157 //0.00000157
#define scale3 1

#define Max_Cutoff_Mtr 800
#define Min_Cutoff_Mtr -800

#define Max_Cutoff_Mtr_Inv 400
#define Min_Cutoff_Mtr_Inv -400

// Encoder External 1
#define ENC1A_HIGH (HAL_GPIO_ReadPin(ENC_EXT1_A_GPIO_Port, ENC_EXT1_A_Pin) == 1)
#define ENC1A_LOW (HAL_GPIO_ReadPin(ENC_EXT1_A_GPIO_Port, ENC_EXT1_A_Pin) == 0)
#define ENC1B_HIGH (HAL_GPIO_ReadPin(ENC_EXT1_B_GPIO_Port, ENC_EXT1_B_Pin) == 1)
#define ENC1B_LOW (HAL_GPIO_ReadPin(ENC_EXT1_B_GPIO_Port, ENC_EXT1_B_Pin) == 0)

// Encoder External 2
#define ENC2A_HIGH (HAL_GPIO_ReadPin(ENC_EXT2_A_GPIO_Port, ENC_EXT2_A_Pin) == 1)
#define ENC2A_LOW (HAL_GPIO_ReadPin(ENC_EXT2_A_GPIO_Port, ENC_EXT2_A_Pin) == 0)
#define ENC2B_HIGH (HAL_GPIO_ReadPin(ENC_EXT2_B_GPIO_Port, ENC_EXT2_B_Pin) == 1)
#define ENC2B_LOW (HAL_GPIO_ReadPin(ENC_EXT2_B_GPIO_Port, ENC_EXT2_B_Pin) == 0)

// Encoder External 3
#define ENC3A_HIGH (HAL_GPIO_ReadPin(ENC_EXT3_A_GPIO_Port, ENC_EXT3_A_Pin) == 1)
#define ENC3A_LOW (HAL_GPIO_ReadPin(ENC_EXT3_A_GPIO_Port, ENC_EXT3_A_Pin) == 0)
#define ENC3B_HIGH (HAL_GPIO_ReadPin(ENC_EXT3_B_GPIO_Port, ENC_EXT3_B_Pin) == 1)
#define ENC3B_LOW (HAL_GPIO_ReadPin(ENC_EXT3_B_GPIO_Port, ENC_EXT3_B_Pin) == 0)

extern int16_t rpmExt[3];
extern int16_t yawVal;
extern float outDot[3];
extern float Aksen[3];
extern float error_arrived;
extern float InvTarget[3];
extern float velo[3];

typedef struct {
	float x;
	float y;
	float th;
} vector3Kin;

typedef struct {
	int32_t w1;
	int32_t w2;
	int32_t w3;
	int32_t w4;
} MotorKin;

vector3Kin ForwardKin(float xStar, float yStar, float thStar);
MotorKin InverseKin(vector3Kin*);
void kinMotor_V3(MotorKin*, float Ex, float Ey, float Eth); // aku pake yg ini
void kinMotor_V2(MotorKin*, float Ex, float Ey, float Eth);
void kinMotor(MotorKin *, float x, float y, float th); // yg ini untuk kamera

#ifdef __cplusplus
}
#endif

#endif /* INC_KIN_H_ */
