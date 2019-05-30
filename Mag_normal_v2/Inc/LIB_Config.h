/**
  ******************************************************************************
  * @file    LIB_Config.h
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief     This file provides configurations for low layer hardware libraries.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USE_LIB_CONFIG_H_
#define _USE_LIB_CONFIG_H_
//Macro Definition

/* Includes ------------------------------------------------------------------*/
#include "MacroAndConst.h"
#include "main.h"

/*------------------------------------------------------------------------------------------------------*/
//SSD1306
#include "SSD1306.h"
#include "Fonts.h"

#define SH1106
//#define SSD1306

//#define INTERFACE_3WIRE_SPI   //3-wire SPI 
#define INTERFACE_4WIRE_SPI     //4-wire SPI 
//#define INTERFACE_IIC         //I2C

#define SSD1306_CS_PIN          OLED_CS_Pin//GPIO_PIN_2
#define SSD1306_CLK_PIN         GPIO_PIN_10
#define SSD1306_DIN_PIN         GPIO_PIN_12
#define SSD1306_RES_PIN         OLED_RES_Pin//GPIO_PIN_8
#define SSD1306_DC_PIN          OLED_DC_Pin//GPIO_PIN_3

#define SSD1306_CS_GPIO         OLED_CS_GPIO_Port
#define SSD1306_CLK_GPIO        GPIOC
#define SSD1306_DIN_GPIO        GPIOC
#define SSD1306_RES_GPIO        OLED_RES_GPIO_Port
#define SSD1306_DC_GPIO         OLED_DC_GPIO_Port

void SSD1306_CS_SET();
void SSD1306_CS_CLR();

void SSD1306_RES_SET();
void SSD1306_RES_CLR();

void SSD1306_DC_SET();
void SSD1306_DC_CLR();

void SSD1306_CLK_SET();
void SSD1306_CLK_CLR();

void SSD1306_DIN_SET();
void SSD1306_DIN_CLR();

void SSD1306_WRITE_BYTE( uint8_t DATA);



extern SPI_HandleTypeDef* hspi3;
/*------------------------------------------------------------------------------------------------------*/
/*
//I2C
#include "IIC.h"

#define IIC_SCL_PIN         GPIO_Pin_13
#define IIC_SDA_PIN         GPIO_Pin_15

#define IIC_SCL_GPIO        GPIOB
#define IIC_SDA_GPIO        GPIOB

#define __IIC_SCL_SET()     GPIO_WriteBit(IIC_SCL_GPIO, IIC_SCL_PIN, Bit_SET)
#define __IIC_SCL_CLR()     GPIO_WriteBit(IIC_SCL_GPIO, IIC_SCL_PIN, Bit_RESET)

#define __IIC_SDA_SET()		GPIO_WriteBit(IIC_SDA_GPIO, IIC_SDA_PIN, Bit_SET)
#define __IIC_SDA_CLR()     GPIO_WriteBit(IIC_SDA_GPIO, IIC_SDA_PIN, Bit_RESET)

#define __IIC_SDA_IN()     	do { \
								GPIO_InitTypeDef tGPIO; \
								tGPIO.GPIO_Pin = IIC_SDA_PIN; \
								tGPIO.GPIO_Speed = GPIO_Speed_50MHz; \
								tGPIO.GPIO_Mode = GPIO_Mode_IPU; \
								GPIO_Init(IIC_SDA_GPIO, &tGPIO); \
							}while(0)				

#define __IIC_SDA_OUT() 	do { \
								GPIO_InitTypeDef tGPIO; \
								tGPIO.GPIO_Pin = IIC_SDA_PIN; \
								tGPIO.GPIO_Speed = GPIO_Speed_50MHz; \
								tGPIO.GPIO_Mode = GPIO_Mode_Out_PP; \
								GPIO_Init(IIC_SDA_GPIO, &tGPIO); \
							}while(0)   


#define __IIC_SDA_READ()    GPIO_ReadInputDataBit(IIC_SDA_GPIO, IIC_SDA_PIN)
*/
/*------------------------------------------------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


#endif

/*-------------------------------END OF FILE-------------------------------*/

