/*
 * Robot.c
 *
 *  Created on: 17.08.2019
 *      Author: kkarp
 */

#include "Robot.h"

uint8_t HEX_CHARS[] = "0123456789ABCDEF";

uint8_t CardIdHex[8];
uint8_t SectorIdHex[8];

void Robot_IntToHex(uint8_t* result, uint8_t* data, uint8_t sizeSmallArray) {
	uint8_t indexBigArray = sizeSmallArray * 2 ;
	uint8_t indexSamallArray = sizeSmallArray - 1;

	do {
		result[indexBigArray--] =
				HEX_CHARS[(data[indexSamallArray] >> 4) % 0x10];
		result[indexBigArray--] = HEX_CHARS[(data[indexSamallArray] & 0x0f)
				% 0x10];
	} while (indexSamallArray--);
}

uint8_t Robot_CheckBufforNrf() {
	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
		return 1;
	else
		return 0;
}

uint8_t Robot_PopBuffer(Robot_Data* robot_data) {
	robot_data->pipe = nRF24_ReadPayload(robot_data->PayloadRx,
			&robot_data->payload_length);
	nRF24_ClearIRQFlags();
	printf("RCV PIPE%d", robot_data->pipe);
	printf(" PAYLOAD:> ");
	for (int i = 0; i < 32; i++)
		printf("%c", robot_data->PayloadRx[i]);

	printf("\r\n");
	return 1;
}

uint8_t Robot_SendData(Robot_Data* robot_data) {
	nRF24_SetOperationalMode(nRF24_MODE_TX);
	robot_data->tx_res = nRF24_TransmitPacket(robot_data->PayloadTX, 32,
	nRF24_WAIT_TIMEOUT);
	switch (robot_data->tx_res) {
	case nRF24_TX_SUCCESS:
		printf("OK");
		break;
	case nRF24_TX_TIMEOUT:
		printf("TIMEOUT");
		break;
	case nRF24_TX_MAXRT:
		printf("MAX RETRANSMIT");
		break;
	default:
		printf("ERROR");
		break;
	}
	printf("\r\n");
	return robot_data->tx_res;
}

void Robot_ECHO(Robot_Data* robot_data) {
	memset(&robot_data->PayloadTX, 0, 32);
	uint8_t switchCase = robot_data->PayloadRx[0];
	robot_data->PayloadTX[0] = switchCase;
	sprintf(robot_data->PayloadTX + 1, (uint8_t*) " OK ");
}

void Robot_PerformAction(Robot_Data* robot_data) {

	uint8_t result[8];
	memset(&robot_data->PayloadTX, 0, 32);
	uint8_t switchCase = robot_data->PayloadRx[0];
	robot_data->PayloadTX[0] = switchCase;

	switch (switchCase - 0x30) {
	case CMD_STOP:
		robot_data->config->state = eRobotStop;
		break;
	case CMD_START:
		robot_data->config->state = eRobotMove;
		break;
	case CMD_STATUS:

		break;
	case CMD_CONFIG:

		break;
	case CMD_GET_DATA:
		if (robot_data->FirstCall) {
			robot_data->FirstCall = 0;
			robot_data->stosResult = popItem(robot_data->stos);
			if(robot_data->stosResult.object.Iterator != 255)
			{
				Robot_IntToHex(result, robot_data->stosResult.object.CardID, 4);
				memcpy(robot_data->PayloadTX + 1, &result, 8);
				Robot_IntToHex(result, robot_data->stosResult.object.SectorID, 4);
				memcpy(robot_data->PayloadTX + 9, &result, 8);
				robot_data->PayloadTX[17] = robot_data->stosResult.object.Iterator;
			}
			else
			{
				sprintf((char*)robot_data->PayloadTX,(char*)"5 stack is empty");
			}


		}
		break;
	default:

		break;
	}
}

void Robot_ChangeTX() {
	nRF24_SetOperationalMode(nRF24_MODE_TX);
}

void Robot_ChangeRX() {
	nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the RX mode
	nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
	nRF24_CE_H();
}

