/*
 * Home_Driver.c
 *
 *  Created on: 07.06.2019
 *      Author: kkarp
 */
#include "Home_Driver.h"

uint8_t Home_checkCard(uint8_t* card1, uint8_t* card2) {
	uint8_t result = False;
	uint8_t iterator = 0;
	for (int i = 0; i <= 3; i++)
		if (card1[i] == card2[i])
			iterator++;
		else
			return result;
	if (iterator == 4)
		return True;
	else
		return result;
}

LedStrip_Speed_InitTypeDef Home_motorControl(LedStrip_InitTypeDef* LedStript) {
	static LedStrip_Speed_InitTypeDef result;
	switch (LedStript->Led_StatusPin) {
	case LS_00001:
	case LS_00010:
	case LS_00011:
		result.Action = rotationInPlace_Right;
		result.LeftSpeed = -3;
		result.RightSpeed = -3;
		break;
	case LS_00100:
		result.Action = move_Forward;
		result.LeftSpeed = -2;
		result.RightSpeed = -2;
		break;
	case LS_10000:
	case LS_11000:
	case LS_01000:
		result.Action = rotationInPlace_Left;
		result.LeftSpeed = -3;
		result.RightSpeed = -3;
		break;
	default:
		break;
	}
	return result;
}

uint8_t Home_ManeuverRotate(Motor_InitTypeDef* LeftMotor,
		Motor_InitTypeDef* RightMotor, uint8_t action,
		Maneuver_StructInit* mane) {

	switch (action) {
	case rotateRight:
		vMotor_Control(LeftMotor, Forward);
		vMotor_Control(RightMotor, Back);
		break;
	case rotateLeft:
		vMotor_Control(LeftMotor, Back);
		vMotor_Control(RightMotor, Forward);
		break;
	default:
		break;
	}
	if (mane->leftFinsh > mane->finishImpulse) {
		vMotor_Control(LeftMotor, BreakeSoft);
	}
	if (mane->rightFinsh > mane->finishImpulse) {
		vMotor_Control(RightMotor, BreakeSoft);
	}
	if (mane->leftFinsh > mane->finishImpulse
			&& mane->rightFinsh > mane->finishImpulse)
		return 1;
	else
		return 0;
}
