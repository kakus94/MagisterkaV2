/*
 * Home_Driver.h
 *
 *  Created on: 07.06.2019
 *      Author: kkarp
 */

#ifndef HOME_DRIVER_H_
#define HOME_DRIVER_H_

#include "main.h"
#include "Motor_Control.h"
#include "LED_Strip.h"

typedef struct{
	uint16_t finishImpulse;
	uint16_t leftFinsh;
	uint16_t rightFinsh;
}Maneuver_StructInit;

enum{
	False,
	True
};

enum{
	rotateLeft,
	rotateRight,
	leftStop,
	RightStop
};

uint8_t Home_checkCard(uint8_t* card1, uint8_t* card2);
LedStrip_Speed_InitTypeDef Home_motorControl(LedStrip_InitTypeDef* LedStript);
uint8_t Home_ManeuverRotate(Motor_InitTypeDef* LeftMotor,
		Motor_InitTypeDef* RightMotor,uint8_t action,Maneuver_StructInit* mane);



#endif /* HOME_DRIVER_H_ */
