/*
 * ESP_8266_driver.c
 *
 *  Created on: 04.04.2019
 *      Author: mprze
 */

#include "ESP_8266_driver.h"
#include "string.h"


uint8_t Command[256];
uint8_t map_size;

/**
 * Function set default configurations for ESP_8266
 */
HAL_StatusTypeDef mySEP8266_Restore(uint8_t mode)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	return status;
}

/**
 * Set mode in ESP_8266
 */
HAL_StatusTypeDef mySEP8266_SetMode(uint8_t mode)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	return status;
}


/**
 * Function init ESP_8266
/**
 *
 */
void myESP_8266_InitClient(uint8_t mode,char *SSID, char* PASSWORD,uint16_t Port)
 {
	/* Copy UART handle variable */
	memcpy(&esp_uart, &huart7, sizeof(huart7));

	/* TODO */
	HAL_GPIO_WritePin(WIFI_CHPD_GPIO_Port, WIFI_CHPD_Pin, SET);
	HAL_GPIO_WritePin(WIFI_RST_GPIO_Port, WIFI_RST_Pin, SET);

	/* Close connection  */
	printf("TX: AT+CIPCLOSE\r\n");
	map_size = sprintf(&Command, "AT+CIPCLOSE\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);


	/* Reset device */
	printf("TX: AT+RST\r\n");
	map_size = sprintf(&Command, "AT+RST\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);

	/* Set device mode */
	printf("TX: AT+CWMODE_DEF=%d\r\n",mode);
	map_size = sprintf(&Command, "AT+CWMODE_DEF=%d\r\n", mode);
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);

	/* Connection to AP */
	map_size = sprintf(&Command, "AT+CWJAP_DEF=\"%s\",\"%s\"\r\n", SSID, PASSWORD);
	printf(&map_size);
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(5000);
	/**/
	/* Get current IP, gateway and netmask */
	printf("TX: AT+CIFSR\r\n");
	map_size = sprintf(&Command, "AT+CIFSR\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);
	/**/

	/* Set current IP, gateway and netmask */
	printf("TX: AT+CIPSTA_DEF=\"192.168.1.11\",\"192.168.1.1\",\"255.255.255.0\"\r\n");
	map_size = sprintf(&Command,
					"AT+CIPSTA_DEF=\"192.168.1.11\",\"192.168.1.1\",\"255.255.255.0\"\r\n");//? 11? 10?
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);


//	/* tcp conect */
////	size = sprintf(&Command, "AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n", Port);
//	printf("TX: AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n");
//	map_size = sprintf(&Command, "AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n", Port);
//	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
//	HAL_Delay(1000);
}

void myESP_8266_TCPconnect(uint8_t mode,char *SSID, char* PASSWORD,uint16_t Port)
{
		/* tcp conect */
//		size = sprintf(&Command, "AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n", Port);
		printf("TX: AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n");
		map_size = sprintf(&Command, "AT+CIPSTART=\"TCP\",\"192.168.1.10\",%d\r\n", Port);
		HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
}

void myESP_8266_PrepareToSending(uint32_t ServerPort)
{

	map_size = sprintf(&Command, "AT+CIPMODE=1\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(200);

	/*  */
	map_size = sprintf(&Command, "AT+CIPSEND\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(200);
}

/**
 * Function send frame
 */
void myESP8266_SendFrame(uint8_t* data, uint32_t Port)
{
	myESP_8266_PrepareToSending(Port);
	HAL_Delay(10);
	map_size = sprintf(&Command, data);

	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);
//	myESP8266_SendEnd();
}

/**
 * Function ending connection with server
 */
void myESP8266_SendEnd()
 {
	map_size = sprintf(&Command, "+++");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);
	map_size = sprintf(&Command, "AT+CIPCLOSE\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(10);
}


/**
 *
 */
void myESP8266_SendFrameFromServer(uint8_t* data, uint8_t LinkID , uint8_t Length)
{
	map_size = sprintf(&Command, "AT+CIPSEND=%d,%d\r\n",LinkID,Length);
	HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
	HAL_Delay(100);
	map_size = sprintf(&Command, data);
	HAL_UART_Transmit(&esp_uart, Command, map_size,1000);
	HAL_Delay(50);
}

/**
 * Function switch to server mode and wait for data from client
 */
void myESP8266_RxMode(uint8_t ServerPort)
{
	map_size = sprintf(&Command, "AT+CIPMODE=0\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size,1000);
	HAL_Delay(300);
	map_size = sprintf(&Command, "AT+CIPMUX=1\r\n");
	HAL_UART_Transmit(&esp_uart, Command, map_size,1000);
	HAL_Delay(300);
	/* Create server on Port */
	map_size = sprintf(&Command, "AT+CIPSERVER=1,%d\r\n",ServerPort);
	HAL_UART_Transmit(&esp_uart, Command, map_size,1000);
	HAL_Delay(300);
}


// Funkcja wysy³aj¹ca podany ci¹g znaków przez interfejs UART

void uart_write_line(UART_HandleTypeDef * handler, char * text)
{

//                HAL_UART_Transmit(handler, text, strlen(text), 1000);
//
//                HAL_UART_Transmit(handler, "\r\n", 2, 100);

}

// Funkcja wysy³aj¹ca dane przez nawi¹zane po³¹czenie TCP

// i zamykaj¹ca to po³¹czenie

void esp_send_data_and_close(UART_HandleTypeDef * uart, char mux_id,
		char * content)
{
//
//		char cmd[17];
//
//		sprintf(cmd, "AT+CIPSEND=%c,%d", mux_id, strlen(content));
//
//		uart_write_line(uart, cmd);
//
//		HAL_Delay(20);
//
//		HAL_UART_Transmit(uart, content, strlen(content), 5000);
//
//		HAL_Delay(100);
//
//		sprintf(cmd, "AT+CIPCLOSE=%c", esp_recv_mux);
//
//		uart_write_line(esp_uart, cmd);

}

uint8_t myESP8266_CheckStatus()
 {
  FIFO_Clear(&FIFO_RX);
  PROTOCOL_LinBuffClr(&LinearBuffer);
  map_size = sprintf(&Command, "AT+CIPSTATUS\r\n");
  HAL_UART_Transmit(&esp_uart, Command, map_size, 1000);
  HAL_Delay(10);
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t data = 0;
  /* Copy UART handle variable */
  memcpy(&esp_uart, &huart7, sizeof(huart7));
  while (__HAL_UART_DISABLE_IT(&esp_uart, UART_IT_RXNE)
      && HAL_OK == FIFO_GetByte(&FIFO_RX, &data))
  {
    __HAL_UART_ENABLE_IT(&esp_uart, UART_IT_RXNE);
    PROTOCOL_LinBuffPutByte(&LinearBuffer, data);
  }
  __HAL_UART_ENABLE_IT(&esp_uart, UART_IT_RXNE);

  uint8_t recdata[32];
  if (NULL != strstr(((char*) LinearBuffer.p_lin_buffer), "\r\nSTATUS:"))
  {
    uint8_t offset = sizeof("AT+CIPSTATUS\r\r\nSTATUS:") - 1;
    memcpy(recdata, LinearBuffer.p_lin_buffer + offset,
        strlen(LinearBuffer.p_lin_buffer));
  }
    uint8_t status_key = recdata[0];
  switch (status_key - 0x30)
  {
  case 1:
    return 1;
    break;
  case 2:
    return 2;
    break;
  case 3:
    return 3;
    break;
  case 4:
    return 4;
    break;
  case 5:
    return 5;
    break;
  default:
    return 0;
    break;
  }
}
