/*
 * DEVICE_driver.h
 *
 *  Created on: 16.04.2019
 *      Author: mprze
 */

#ifndef DEVICE_DRIVER_H_
#define DEVICE_DRIVER_H_

#include "PROTOCOL_driver.h"
#include "usart.h"

UART_HandleTypeDef  esp_uart;
HAL_StatusTypeDef PROTOCOL_GetDataFromFifo();
HAL_StatusTypeDef GetDataFromFifo();
HAL_StatusTypeDef PROTOCOL_CheckFrame(PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff);


#endif /* DEVICE_DRIVER_H_ */
