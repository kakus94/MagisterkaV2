/**
 * Copyright (c) 2018 by Surge Cloud
 */

/**
 * @file FIFO_driver.h
 * @author Marcin Przepiorkowski - eMPe
 * @brief  It contains driver of FIFO, declarations of the FIFO structure and FIFO functions
 */
 

#ifndef __FIFO_DRIVER_H__
#define __FIFO_DRIVER_H__

#include "main.h"
#include <stdint.h>

#define FIFO_BUF_SIZE 64

/**
 * @brief Linear buffer structure
 */
typedef struct PROTOCOL_LinearBuffer_ApiTypeDef
{
    char* const p_lin_buffer;  /*!< Pointer on FIFO buffer */
    uint16_t head;                /*!< Last added element */
    uint16_t size;                /*!< Size of buffer */
}__packed PROTOCOL_LinearBuffer_ApiTypeDef;

/**
 * @brief FIFO API structure
 */
typedef struct FIFO_ApiTypeDef
{
    volatile uint8_t* const buffer;   /*!< Pointer on FIFO buffer */
    volatile uint16_t head;           /*!< Last added element */
    volatile uint16_t tail;           /*!< First added element */
    volatile uint16_t length;         /*!< Current number of bytes in buffer */
    uint16_t size;                    /*!< Size of buffer */
} FIFO_ApiTypeDef;

HAL_StatusTypeDef FIFO_PutByte(FIFO_ApiTypeDef *pFifo, uint8_t Data);
HAL_StatusTypeDef FIFO_PutBytes(FIFO_ApiTypeDef *pFifo, uint8_t *pData,
        uint16_t Length);
HAL_StatusTypeDef FIFO_GetByte(FIFO_ApiTypeDef *pFifo, uint8_t *pData);
uint16_t FIFO_GetLength(FIFO_ApiTypeDef *pFifo);
HAL_StatusTypeDef FIFO2FIFO_GetBytes(FIFO_ApiTypeDef *pFifo, uint16_t Length,
        FIFO_ApiTypeDef *pDestq);
HAL_StatusTypeDef FIFO_GetBytes(FIFO_ApiTypeDef *pFifo, uint16_t Length,
        uint8_t *pTable);
HAL_StatusTypeDef FIFO_Clear(FIFO_ApiTypeDef *pFifo);

HAL_StatusTypeDef PROTOCOL_LinBuffPutByte(PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff,
    uint8_t Data);
HAL_StatusTypeDef PROTOCOL_LinBuffClr(PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff);

extern volatile uint8_t Fifo_Tx[FIFO_BUF_SIZE];
extern volatile uint8_t Fifo_Rx[FIFO_BUF_SIZE];
extern FIFO_ApiTypeDef FIFO_TX;
extern FIFO_ApiTypeDef FIFO_RX;
extern PROTOCOL_LinearBuffer_ApiTypeDef LinearBuffer;
extern PROTOCOL_LinearBuffer_ApiTypeDef TagDataStuct;

#endif /* __FIFO_DRIVER_H__ */
