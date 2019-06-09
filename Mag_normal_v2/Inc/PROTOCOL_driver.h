/*
 * PROTOCOL_driver.h
 *
 *  Created on: 16.04.2019
 *      Author: mprze
 */

#ifndef PROTOCOL_DRIVER_H_
#define PROTOCOL_DRIVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "FIFO_driver.h"


#define PROTOCOL_CMD_CRC_SIZE 4
#define BROADCAST_ADDRESS     255
#define DEVICE_ADDRESS        100
extern bool timeouted;

/**
 * @brief PROTOCOL commans enumerator definition
 */
typedef enum PROTOCOL_CmdTypeDef
{
  CMD_STOP              = 1,   /*!< Command stop motor */
  CMD_START             = 2,   /*!< Command start motor */
  CMD_STATUS            = 3,   /*!< Command checks status device*/
  CMD_CONFIG            = 4,   /*!< Command set config device*/
  CMD_GET_DATA          = 5,   /*!< Command for harvesting data */
} PROTOCOL_CmdTypeDef;



/**
 * @brief Scan command request payload structure definition
 */
typedef struct PROTOCOL_CmdScan_ReqPayloadTypeDef
{

} __packed PROTOCOL_CmdScan_ReqPayloadTypeDef;

/* PROTOCOL_CmdScan_RespPayloadTypeDef is empty */

/**
 * @brief GetData command request payload structure definition
 */
typedef struct PROTOCOL_CmdGetData_ReqPayloadTypeDef
{

} __packed PROTOCOL_CmdGetData_ReqPayloadTypeDef;

/**
 * @brief GetData command response payload structure definition
 */
typedef struct PROTOCOL_CmdGetData_RespPayloadTypeDef
{
  uint8_t   tags_data[255];          /*!< Tag's data (UIDs etc.) */
} __packed PROTOCOL_CmdGetData_RespPayloadTypeDef;

/**
 * @brief Protocol frame request payload structure definition
 */
typedef struct PROTOCOL_ReqPayloadTypeDef
{
  union
  {
    PROTOCOL_CmdScan_ReqPayloadTypeDef        scan_payload;       /*!< Payload structure for CMD_SCAN request command */
    PROTOCOL_CmdGetData_ReqPayloadTypeDef     get_data_payload;   /*!< Payload structure for GET_DATA request command */
  };
} __packed PROTOCOL_ReqPayloadTypeDef;

/**
 * @brief Protocol frame response payload structure definition
 */
typedef struct PROTOCOL_RespPayloadTypeDef
{
  union
  {
    PROTOCOL_CmdGetData_RespPayloadTypeDef    get_data_payload;   /*!< Payload structure for CMD_GET_DATA response command */
  };
} __packed PROTOCOL_RespPayloadTypeDef;

/**
 * @brief Protocol Header structure definition
 */
typedef struct PROTOCOL_HeaderTypeDef
{
  uint8_t    dst_address;            /*!< Destination address */
  uint8_t    src_address;            /*!< Source address */
  uint8_t    length;                 /*!< Frame length */
  union
  {
    uint8_t   raw_command;    /*!< RAW command */
    struct
    {
      PROTOCOL_CmdTypeDef command   : 7;  /*!< 7-bit command */
      bool            response  : 1;  /*!< REQ / RESP indicator */
    };
  };
} __packed PROTOCOL_HeaderTypeDef;

/**
 * @brief PROTOCOL Frame structure definition
 */
typedef struct PROTOCOL_FrameTypeDef
{
  PROTOCOL_HeaderTypeDef        header;               /*!< PROTOCOL frame header */
  union
  {
    uint8_t         raw_data[FIFO_BUF_SIZE];          /*!< PROTOCOL frame RAW payload */
    PROTOCOL_ReqPayloadTypeDef  req_payload;          /*!< PROTOCOL request frame payload */
    PROTOCOL_RespPayloadTypeDef resp_payload;         /*!< PROTOCOL response frame payload */
  };
} __packed PROTOCOL_FrameTypeDef;

#endif /* PROTOCOL_DRIVER_H_ */
