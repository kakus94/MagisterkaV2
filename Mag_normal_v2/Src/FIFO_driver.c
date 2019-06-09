/**
 * @file FIFO_driver.c
 * @author Marcin Przepiorkowski - eMPe
 * @brief  It contains driver of FIFO, definitions of FIFO functions
 */
 
#include "FIFO_driver.h"

/**
 * Function to put byte to FIFO
 * @param  pFifo              Pointer to FIFO
 * @param  Data               Data to add to the FIFO buffer
 * @return HAL_StatusTypeDef  Status of the put byte to the FIFO operation
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef FIFO_PutByte(FIFO_ApiTypeDef *pFifo, uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint16_t head_temp = pFifo->head + 1; /* temporary variable */
  if (head_temp == pFifo->size) /* if  is it last element of array ==> back to begin */
  {
    head_temp = 0;
  }
  if (head_temp == pFifo->tail) /* check is buffer full? ==> if head+1 == tail */
  {
    status = HAL_ERROR; /* return -1 ==> buffer is full */
  }
  /* buffer is not full */
  if (status == HAL_OK)
  {
    pFifo->buffer[pFifo->head] = Data; /* put data to buffer */
    pFifo->head = head_temp; /* head ++ */
    pFifo->length = FIFO_GetLength(pFifo); /* update FIFO length */
  }
  return status; /* return status */
}



/**
 * Function to get byte from FIFO
 * @param  pFifo              Pointer to FIFO
 * @param  pData              Return byte
 * @return HAL_StatusTypeDef  Status of the get byte from FIFO operation
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef FIFO_GetByte(FIFO_ApiTypeDef *pFifo, uint8_t *pData)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint16_t next = pFifo->tail + 1; /* temporary variable */
  if (pFifo->head == pFifo->tail) /* check is buffer empty? ==> if head == tail */
  {
    status=HAL_ERROR;
  }
  else
  {
    if (next == pFifo->size) /* if  is it last element of array ==> back to begin */
    {
      next = 0;
    }
    /* buffer is not empty */
    *pData = pFifo->buffer[pFifo->tail]; /* read value */
    pFifo->tail = next; /* tail ++ */
    pFifo->length = FIFO_GetLength(pFifo); /* update FIFO length */
  }
   return status; /* return status */
}


/**
 * Function to get current number of bytes in the FIFO buffer
 * @param  pFifo      Pointer to FIFO
 * @return uint16_t   Number of bytes to send in FIFO
 */
uint16_t FIFO_GetLength(FIFO_ApiTypeDef *pFifo)
{
  uint16_t templen = 0;
  if (pFifo->head >= pFifo->tail)
  {
    templen = pFifo->head - pFifo->tail;
  }
  else
  {
    templen = pFifo->size - (pFifo->tail - pFifo->head);
  }
  return templen; /* return length */
}

/**
 * Function to get more than one byte from FIFO and put this byts to another FIFO
 * @param  pFifo              Pointer to source FIFO
 * @param  Length             Number of byte to get
 * @param  pDestq             Destination FIFO when byte will be put
 * @return HAL_StatusTypeDef  Status of the get byte from pFifo to pDestq FIFO operation
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef FIFO2FIFO_GetBytes(FIFO_ApiTypeDef *pFifo, uint16_t Length, FIFO_ApiTypeDef *pDestq)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data = 0;
  while (Length--)
  {
    if (!FIFO_GetByte(pFifo, &data)) /* if buffer not empty */
    {
      if (FIFO_PutByte(pDestq, data)) /* if buffer full */
      {
        status = HAL_ERROR; /* error ==> buffer is full */
      }
    }
    else
    {
      status = HAL_ERROR; /* error ==> buffer is empty */
    }
  }
  return status; /* return status */
}


/**
 * Function to get more than one byte from FIFO
 * @param  pFifo              Pointer to FIFO
 * @param  Length             Number of byte to get
 * @param  pTable             Pointer to the table when bytes will be put
 * @return HAL_StatusTypeDef  Status of the get more than one byte from pFifo operation
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef FIFO_GetBytes(FIFO_ApiTypeDef *pFifo, uint16_t Length, uint8_t *pTable)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data = 0;
  int i = 0;
  while (Length--)
  {
    if (!FIFO_GetByte(pFifo, &data))
    {
      pTable[i++] = data;
    }
    else
    {
      status = HAL_ERROR; /* buffer is empty */
    }
  }
  return status;
}

/**
 * Function to put string to FIFO buffer
 * @param  pFifo              Pointer to FIFO
 * @param  pData              Pointer to the data (it can be uint8, uint16, uint32, ...)
 * @param  Length             Length of Data
 * @return HAL_StatusTypeDef  Status of the put text to FIFO operation
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef FIFO_PutBytes(FIFO_ApiTypeDef *pFifo, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  while (Length--)
  {
    if (FIFO_PutByte(pFifo, *pData++) != HAL_OK)
    {
      status = HAL_ERROR; /* buffer is full next, data will be lost */
      break;
    }
  }
  return status; /* return 0 ==> is OK !! */
}

/**
 * Function to clear FIFO buffer
 * @param pFifo
 * @return
 */
HAL_StatusTypeDef FIFO_Clear(FIFO_ApiTypeDef *pFifo)
{
  HAL_StatusTypeDef status = HAL_OK;
  pFifo->head=pFifo->tail=0;
  pFifo->length = FIFO_GetLength(pFifo);
  return status;
}



/**
 * PROTOCOL_LinBuff_PutByte
 * @param  pLinBuff           Pointer to the PROTOCOL_LinearBuffer_ApiTypeDef
 * @param  Data               Data to put in linear buffer
 * @return HAL_StatusTypeDef  Status of the parser received data
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef PROTOCOL_LinBuffPutByte(PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff,
    uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
  if (pLinBuff->head < pLinBuff->size)
  {
    pLinBuff->p_lin_buffer[pLinBuff->head] = Data;
    pLinBuff->head++;
  }
  else
  {
    status = HAL_ERROR;
  }
  return status;
}

/**
 * Clear linear buffer - set head on zero index
 * @param  pLinBuff           Pointer to the linear buffer
 * @return HAL_StatusTypeDef  Status of clearing the linear buffer
 * @retval HAL_OK             On success
 * @retval HAL_ERROR          In case of error
 */
HAL_StatusTypeDef PROTOCOL_LinBuffClr(PROTOCOL_LinearBuffer_ApiTypeDef* pLinBuff)
{
  HAL_StatusTypeDef status = HAL_OK;
  pLinBuff->head = 0;
  memset(pLinBuff->p_lin_buffer, 0x00, sizeof(pLinBuff->size));
  return status;
}

///**
// * Update linear buffer
// * @param  pChainComm         Pointer to the COMMUNICATION structure
// * @return HAL_StatusTypeDef  Status of updating the linear buffer
// * @retval HAL_OK             On success
// * @retval HAL_ERROR          In case of error
// */
//HAL_StatusTypeDef PROTOCOL_LinBuffUpdate(PROTOCOL_ApiTypeDef* pChainComm)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//  pChainComm->p_linear_buffer->head = pChainComm->p_frame->header.length
//          - PROTOCOL_CMD_CRC_SIZE;
//  return status;
//}
