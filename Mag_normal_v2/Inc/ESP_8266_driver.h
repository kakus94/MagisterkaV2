/*
 * ESP_8266_driver.h
 *
 *  Created on: 04.04.2019
 *      Author: mprze
 */

#ifndef ESP_8266_DRIVER_H_
#define ESP_8266_DRIVER_H_

#include "main.h"
#include "usart.h"
#include "FIFO_driver.h"
#include "PROTOCOL_driver.h"

#define SERVER_PORT 333

UART_HandleTypeDef  esp_uart;

HAL_StatusTypeDef mySEP8266_Restore(uint8_t mode);
HAL_StatusTypeDef mySEP8266_SetMode(uint8_t mode);


void myESP_8266_InitServer(uint8_t mode,char *SSID, char* PASSWORD,uint16_t Port);
void myESP_8266_InitClient(uint8_t mode,char *SSID, char* PASSWORD,uint16_t Port);

void myESP_8266_PrepareToSending(uint32_t ServerPort);
void myESP8266_SendFrame(uint8_t* data, uint32_t Port);
void myESP8266_SendEnd();

void myESP8266_SendFrameFromServer(uint8_t* data, uint8_t LinkID , uint8_t Length);

void myESP8266_RxMode(uint8_t ServerPort);
uint8_t myESP8266_CheckStatus();
void myESP_8266_TCPconnect(uint8_t mode,char *SSID, char* PASSWORD,uint16_t Port);

#endif /* ESP_8266_DRIVER_H_ */
