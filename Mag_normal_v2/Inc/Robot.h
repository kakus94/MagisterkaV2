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

uint16_t RobotNrfTimer; //timer frequency check NRF

typedef struct {

} Status_InitTypeDef;

typedef struct {
	uint8_t state;
} Config_InitTypeDef;

typedef struct {
	uint8_t id_card[4];
	uint8_t id_sector[4];
	uint8_t iterator;
} CardData_InitTypeDef;

typedef struct{
	Motor_InitTypeDef* motorRight;
	Motor_InitTypeDef* motorLeft;
	MotorPID_InitTypeDef* motorPID_Left;
	MotorPID_InitTypeDef* motorPID_Right;
}Movment_InitTypeDef;

typedef struct {
	Status_InitTypeDef* status;
	Config_InitTypeDef* config;
	CardData_InitTypeDef* card_data;
	Movment_InitTypeDef* movment;
	Stos_typeDef** stos;
	Stos_typeDef stosResult;
	uint8_t payloadRx[32];
	uint8_t payloadTX[32];
	uint8_t payload_length;
	nRF24_RXResult pipe;
	nRF24_TXResult tx_res;
	uint8_t firstCall;
	uint8_t actionsPerformed;
	uint8_t stackIsEmpty;
} RobotData_InitTypeDef;

typedef enum{
	eRobotStop,eRobotMove
}enumRobot;

uint8_t Robot_CheckBufforNrf();
uint8_t Robot_PopBuffer(RobotData_InitTypeDef*);
uint8_t Robot_SendData(RobotData_InitTypeDef*);
void Robot_PerformAction(RobotData_InitTypeDef*);
void Robot_ECHO(RobotData_InitTypeDef* robot_data);
void Robot_ChangeTX();
void Robot_ChangeRX();

void Robot_IntToHex(uint8_t* result,uint8_t* data,uint8_t sizeSmallArray);





#endif /* ROBOT_H_ */

