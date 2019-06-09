/*
 * DEVICE_driver.c
 *
 *  Created on: 16.04.2019
 *      Author: mprze
 */
#include "main.h"
#include "DEVICE_driver.h"
#include "FIFO_driver.h"
#include "string.h"

/**
 * Function to get data from FIFO to LinearBuffer, start checking frame
 * @return HAL_StatusTypeDef  Status of the check frame
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef PROTOCOL_GetDataFromFifo()
{
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
	PROTOCOL_FrameTypeDef* pRecFrame =
			(PROTOCOL_FrameTypeDef*) LinearBuffer.p_lin_buffer;

	if (LinearBuffer.head
			> (offsetof(PROTOCOL_HeaderTypeDef, length)
					+ sizeof(pRecFrame->header.length))
			&& pRecFrame->header.length <= LinearBuffer.head)
	{
		status = HAL_OK;
	}
	if (LinearBuffer.head == 10 && 0x0D == LinearBuffer.p_lin_buffer[0]
			&& 0x3A == LinearBuffer.p_lin_buffer[9])
	{
		FIFO_Clear(&FIFO_RX);
		PROTOCOL_LinBuffClr(&LinearBuffer);
	}
	return status;
}

HAL_StatusTypeDef GetDataFromFifo()
{
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
    if(32 == LinearBuffer.head)
    {
      status = HAL_OK;
    }
return status;
}

/**
 * Function checks frame
 * @param  pChainComm         Pointer to the PROTOCOL_LinearBuffer_ApiTypeDef
 * @return HAL_StatusTypeDef  Status of the check frame
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef PROTOCOL_CheckFrame(
		PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	PROTOCOL_FrameTypeDef* pRecFrame =
			(PROTOCOL_FrameTypeDef*) LinearBuffer.p_lin_buffer;
	if (pRecFrame->header.dst_address == BROADCAST_ADDRESS
			|| pRecFrame->header.dst_address == DEVICE_ADDRESS)
	{
		status = HAL_OK;
	} else
	{
		FIFO_Clear(&FIFO_RX);
		PROTOCOL_LinBuffClr(pLinBuff);
	}
	return status;
}
