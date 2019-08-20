/*
 * Robot.h
 *
 *  Created on: 17.08.2019
 *      Author: kkarp
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "main.h"
#include "nrf24.h"
#include "stdio.h"
#include "Motor_Control.h"
#include "PROTOCOL_driver.h"
#include "string.h"
#include "cardSystem.h"

#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

uint16_t RobotNrfTimer;

typedef struct {

} Status;

typedef struct {
	uint8_t state;
} Config;

typedef struct {
	uint8_t Id_card[4];
	uint8_t Id_sector[4];
	uint8_t iterator;
} Card_Data;

typedef struct {
	Status* status;
	Config* config;
	Card_Data* card_data;
	Stos_typeDef** stos;
	Stos_typeDef stosResult;
	Motor_InitTypeDef* MotorRight;
	Motor_InitTypeDef* MotorLeft;
	MotorPID_InitTypeDef* MotorPID_Left;
	MotorPID_InitTypeDef* MotorPID_Right;
	uint8_t PayloadRx[32];
	uint8_t PayloadTX[32];
	uint8_t payload_length;
	nRF24_RXResult pipe;
	nRF24_TXResult tx_res;
	uint8_t FirstCall;
	uint8_t ActionsPerformed;
	uint8_t stackIsEmpty;
} Robot_Data;

typedef enum{
	eRobotStop,eRobotMove
}enumRobot;

uint8_t Robot_CheckBufforNrf();
uint8_t Robot_PopBuffer(Robot_Data*);
uint8_t Robot_SendData(Robot_Data*);
void Robot_PerformAction(Robot_Data*);
void Robot_ECHO(Robot_Data* robot_data);
void Robot_ChangeTX();
void Robot_ChangeRX();

void Robot_IntToHex(uint8_t* result,uint8_t* data,uint8_t sizeSmallArray);





#endif /* ROBOT_H_ */

