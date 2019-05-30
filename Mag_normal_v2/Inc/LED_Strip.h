/*
 * LED_Strip.h
 *
 *  Created on: 04.04.2019
 *      Author: kkarp
 */

#ifndef LED_STRIP_H_
#define LED_STRIP_H_

#include "main.h"

/*              LED STRIP
 * **************************************
 * * LED1    LED2   LED3   LED4   LED5  *
 * **************************************
 * */

#define Led1_Gpio 	LEDS_OUT1_GPIO_Port
#define Led1_Pin 	LEDS_OUT1_Pin

#define Led2_Gpio 	LEDS_OUT2_GPIO_Port
#define Led2_Pin 	LEDS_OUT2_Pin

#define Led3_Gpio 	LEDS_OUT3_GPIO_Port
#define Led3_Pin 	LEDS_OUT3_Pin

#define Led4_Gpio  	LEDS_OUT4_GPIO_Port
#define Led4_Pin 	LEDS_OUT4_Pin

#define Led5_Gpio 	LEDS_OUT5_GPIO_Port
#define Led5_Pin    LEDS_OUT5_Pin

#define map_size 10

enum
{
//	LS_miss = 0x1F,
//	LS_all = 0x00,
//	LS_00001 = 0x1E,
//	LS_00011 = 0x1C,
//	LS_00010 = 0x1D,
//	LS_00110 = 0x19,
//	LS_00100 = 0x1B,
//	LS_01100 = 0x13,
//	LS_01000 = 0x17,
//	LS_11000 = 0x07,
//	LS_10000 = 0x0F,
//	LS_miss_L = 0xF0,
//	LS_miss_R = 0xF1
	LS_miss = 0x1F,
	LS_all = 0x00,
	LS_00001 = 0x0F,
	LS_00011 = 0x07,
	LS_00010 = 0x17,
	LS_00110 = 0x13,
	LS_00100 = 0x1B,
	LS_01100 = 0x19,
	LS_01000 = 0x1D,
	LS_11000 = 0x1C,
	LS_10000 = 0x1E,
	LS_miss_L = 0xF0,
	LS_miss_R = 0xF1
} LedStrip_StatusPinEnum;

typedef struct
{
	uint8_t history[10];
	uint8_t pivot;
	uint8_t MissLineError;
	union
	{
		uint8_t Led_StatusPin;
		struct
		{
			uint8_t Led1_StatusPin :1;
			uint8_t Led2_StatusPin :1;
			uint8_t Led3_StatusPin :1;
			uint8_t Led4_StatusPin :1;
			uint8_t Led5_StatusPin :1;
			uint8_t unused :3;
		};
	};
} LedStrip_InitTypeDef;

typedef struct
{
	int8_t LeftSpeed;
	int8_t RightSpeed;
	uint8_t Action;

} LedStrip_Speed_InitTypeDef;

void vLedStrip_ReadStatus(LedStrip_InitTypeDef* LedStript);
LedStrip_Speed_InitTypeDef vLed_control(LedStrip_InitTypeDef* LedStript,uint8_t tryb);
void vLedStrip_Init(LedStrip_InitTypeDef* ledStrip_InitTypeDef);

#endif /* LED_STRIP_H_ */
