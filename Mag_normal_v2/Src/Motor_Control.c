/*
 * Motor_Control.c
 *
 *  Created on: 29.03.2019
 *      Author: kkarp
 */
#include "Motor_Control.h"

uint16_t uGetCounterTim(TIM_TypeDef* tim) {
	return tim->CNT;
}

HAL_StatusTypeDef vClearCounter(TIM_TypeDef* tim) {
	HAL_StatusTypeDef Status = HAL_ERROR;
	if (NULL != tim) {
		tim->CNT = 0;
		Status = HAL_OK;
	}
	return Status;
}

void vMotor_init(Motor_InitTypeDef* Motor_InitStruct1,
		Motor_InitTypeDef* Motor_InitStruct2) {
	Motor_InitStruct1->Motor_GPIO_int1 = MotorLeft_GPIO_pin1;
	Motor_InitStruct1->Motor_GPIO_int2 = MotorLeft_GPIO_pin2;
	Motor_InitStruct1->Motor_Pin_int1 = MotorLeft_Pin1;
	Motor_InitStruct1->Motor_Pin_int2 = MotorLeft_Pin2;
	Motor_InitStruct1->number_turns = 0;
	Motor_InitStruct1->speed = 0;
	Motor_InitStruct1->Tim_Encoder = MotorLeft_Encoder;
	Motor_InitStruct1->Tim_PWM = MotorLeft_PWM;
	Motor_InitStruct1->Code = 0xFFAAFFAA;
	Motor_InitStruct1->CCRx = &MotorLeft_PWM->CCR1;
	Motor_InitStruct1->clear_PID = 0;

	Motor_InitStruct2->Motor_GPIO_int1 = MotorRight_GPIO_pin1;
	Motor_InitStruct2->Motor_GPIO_int2 = MotorRight_GPIO_pin2;
	Motor_InitStruct2->Motor_Pin_int1 = MotorRight_Pin1;
	Motor_InitStruct2->Motor_Pin_int2 = MotorRight_Pin2;
	Motor_InitStruct2->number_turns = 0;
	Motor_InitStruct2->speed = 0;
	Motor_InitStruct2->Tim_Encoder = MotorRight_Encoder;
	Motor_InitStruct2->Tim_PWM = MotorRight_PWM;
	Motor_InitStruct2->Code = 0xAABBEEFF;
	Motor_InitStruct2->CCRx = &MotorRight_PWM->CCR2;
	Motor_InitStruct2->clear_PID = 0;
}



void vMotor_Control(Motor_InitTypeDef* motor, uint8_t eBridgeControl) {
	switch (eBridgeControl) {
	case BreakeHard:
		HAL_GPIO_WritePin(motor->Motor_GPIO_int1, motor->Motor_Pin_int1,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->Motor_GPIO_int2, motor->Motor_Pin_int2,
				GPIO_PIN_RESET);
		motor->Tim_PWM->CCR1 = motor->Tim_PWM->ARR;
		break;
	case Back:
		HAL_GPIO_WritePin(motor->Motor_GPIO_int1, motor->Motor_Pin_int1,
				GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->Motor_GPIO_int2, motor->Motor_Pin_int2,
				GPIO_PIN_SET);
		motor->Tim_PWM->CCR1 = motor->dutyPWM;
		break;
	case Forward:
		HAL_GPIO_WritePin(motor->Motor_GPIO_int1, motor->Motor_Pin_int1,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->Motor_GPIO_int2, motor->Motor_Pin_int2,
				GPIO_PIN_RESET);
//		motor->Tim_PWM->CCR1 = motor->dutyPWM;
		*motor->CCRx = motor->dutyPWM;
		break;
	case BreakeSoft:
		HAL_GPIO_WritePin(motor->Motor_GPIO_int1, motor->Motor_Pin_int1,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->Motor_GPIO_int2, motor->Motor_Pin_int2,
				GPIO_PIN_SET);
		motor->Tim_PWM->CCR1 = 0;
		break;
	default:
		break;
	}
}

void vMotor_SetPWM(Motor_InitTypeDef* motor, uint8_t dutyPWM) {
	motor->dutyPWM = (motor->Tim_PWM->ARR * dutyPWM) / 100.0;
#ifndef ONE_TIM_TO_PWM
	motor->Tim_PWM->CCR1 = motor->dutyPWM;
#else
	if (motor->Code == 0xFFAAFFAA) {
		motor->Tim_PWM->CCR1 = motor->dutyPWM;
	} else {
		motor->Tim_PWM->CCR2 = motor->dutyPWM;
	}
#endif
}

void vMotorPID_init(MotorPID_InitTypeDef* PID1, MotorPID_InitTypeDef* PID2) {
	PID1->e_last = 0;
	PID1->e_sum = 0;
	PID1->e_sumMax = MaxPIDki;
	PID1->kp = KpValue1;
	PID1->ki = KiValue1;
	PID1->kd = KdValue1;
	PID1->ValueTask = 90;

	PID2->e_last = 0;
	PID2->e_sum = 0;
	PID2->e_sumMax = MaxPIDki;
	PID2->kp = KpValue2;
	PID2->ki = KiValue2;
	PID2->kd = KdValue2;
	PID2->ValueTask = 90;
}

void vMotorPID_clear(MotorPID_InitTypeDef* PID1, MotorPID_InitTypeDef* PID2){
	PID1->e_last = 0;
	PID1->e_sum = 0;
	PID1->e = 0;
	PID1->ExecutionValue =0;

	PID2->e_last = 0;
	PID2->e_sum = 0;
	PID2->e = 0;
	PID2->ExecutionValue =0;
}

void vMotorPID_Control(MotorPID_InitTypeDef* MotorPID, Motor_InitTypeDef* Motor) {

	if (Motor->clear_PID == 1) {
		Motor->clear_PID = 0;
		MotorPID->e_sum = 0;
	}

	uint16_t encoderCounter = uGetCounterTim(Motor->Tim_Encoder);
	if (encoderCounter < 500) {
		MotorPID->e = MotorPID->ValueTask - encoderCounter;
	} else {
		MotorPID->e = MotorPID->ValueTask - (1000 - encoderCounter);
	}
	MotorPID->e_sum += MotorPID->e;
	if (MotorPID->e_sum > MotorPID->e_sumMax)
		MotorPID->e_sum = MotorPID->e_sumMax;

	MotorPID->ExecutionValue = MotorPID->kp * MotorPID->e
			+ MotorPID->ki * MotorPID->e_sum
			+ MotorPID->kd * (MotorPID->e - MotorPID->e_last);

	if (MotorPID->ValueTask == 0) {
		MotorPID->e_sum = 0;
		MotorPID->ExecutionValue = 0;
	}
}

void vMotorAction_LedStrip(Motor_InitTypeDef* LeftMotor,
		Motor_InitTypeDef* RightMotor, uint8_t action) {
	switch (action) {
	case rotationInPlace_Left:
		vMotor_Control(LeftMotor, Back);
		vMotor_Control(RightMotor, Forward);
		break;
	case rotationInPlace_Right:
		vMotor_Control(LeftMotor, Forward);
		vMotor_Control(RightMotor, Back);
		break;
	case turn_Left:
		vMotor_Control(LeftMotor, BreakeSoft);
		vMotor_Control(RightMotor, Forward);
		break;
	case turn_Right:
		vMotor_Control(LeftMotor, Forward);
		vMotor_Control(RightMotor, BreakeSoft);
		break;
	case move_Back:
		vMotor_Control(LeftMotor, Back);
		vMotor_Control(RightMotor, Back);
		break;
	case move_Forward:
		vMotor_Control(LeftMotor, Forward);
		vMotor_Control(RightMotor, Forward);
		break;
	default:
		break;
	}
}

