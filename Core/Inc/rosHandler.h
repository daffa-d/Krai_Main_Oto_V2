/*
 * rosHandler.h
 *
 *  Created on: Feb 12, 2024
 *      Author: daffa
 */

#ifndef INC_ROSHANDLER_H_
#define INC_ROSHANDLER_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "kin.h"

extern float xtarget;
extern float ytarget;
extern float thtarget;
extern float errorPub;
extern float InvTarget[3];
extern float msg_imu[10];
extern bool stateInv;

#ifdef __cplusplus

extern "C"{
#endif

void setup();
void loop();

#ifdef __cplusplus
}
#endif

#endif /* INC_ROSHANDLER_H_ */
