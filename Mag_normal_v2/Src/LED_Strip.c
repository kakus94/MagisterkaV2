/*
 * LED_Strip.c
 *
 *  Created on: 04.04.2019
 *      Author: kkarp
 */

#include "LED_Strip.h"
#include "Motor_Control.h"
#include "main.h"

void vLedStrip_Init(LedStrip_InitTypeDef* ledStrip_InitTypeDef) {
  ledStrip_InitTypeDef->pivot = 0;
  ledStrip_InitTypeDef->Led_StatusPin = 0;
}

void vLedStrip_ReadStatus(LedStrip_InitTypeDef* LedStript) {
  if (NULL != LedStript)
  {
    LedStript->Led1_StatusPin = HAL_GPIO_ReadPin(Led1_Gpio, Led1_Pin);
    LedStript->Led2_StatusPin = HAL_GPIO_ReadPin(Led2_Gpio, Led2_Pin);
    LedStript->Led3_StatusPin = HAL_GPIO_ReadPin(Led3_Gpio, Led3_Pin);
    LedStript->Led4_StatusPin = HAL_GPIO_ReadPin(Led4_Gpio, Led4_Pin);
    LedStript->Led5_StatusPin = HAL_GPIO_ReadPin(Led5_Gpio, Led5_Pin);

    if (LedStript->Led_StatusPin != LedStript->history[LedStript->pivot])
    {
      LedStript->pivot++;
      if (LedStript->pivot >= map_size)
      {
        LedStript->pivot = 0;
      }
      LedStript->history[LedStript->pivot] = LedStript->Led_StatusPin;
    }
  }
}

LedStrip_Speed_InitTypeDef vLed_control(LedStrip_InitTypeDef* LedStript,
    uint8_t tryb) {
  static LedStrip_Speed_InitTypeDef Motor;
  if (!tryb)
  {
    switch (LedStript->Led_StatusPin)
    {
    case LS_miss:
      LedStript->MissLineError = 1;
      uint8_t TempPivot;
      uint8_t LastLine;
      TempPivot = LedStript->pivot - 1;
      if (TempPivot > map_size)
      {
        LastLine = LedStript->history[map_size - 1];
      } else
      {
        LastLine = LedStript->history[TempPivot];
      }
      if (LastLine == LS_00001)
      {
        Motor.LeftSpeed = 0;
        Motor.RightSpeed = 0;
        Motor.Action = rotationInPlace_Right;
      } else if (LastLine == LS_10000)
      {
        Motor.LeftSpeed = 0;
        Motor.RightSpeed = 0;
        Motor.Action = rotationInPlace_Left;
      } else
      {
        Motor.LeftSpeed = -3;
        Motor.RightSpeed = -3;
        Motor.Action = move_Back;
      }
      break;
    case LS_all:
      break;
    case LS_00001:
      Motor.LeftSpeed = -2;
      Motor.RightSpeed = -2;
      Motor.Action = rotationInPlace_Right;
      break;
    case LS_00010:
      Motor.LeftSpeed = 3;
      Motor.RightSpeed = -3;
      Motor.Action = move_Forward;

      break;
    case LS_00011:
      Motor.LeftSpeed = 2;
      Motor.RightSpeed = -5;
      Motor.Action = move_Forward;
      break;
    case LS_00100:
      LedStript->MissLineError = 0;
      Motor.LeftSpeed = 0;
      Motor.RightSpeed = 0;
      Motor.Action = move_Forward;
      break;
    case LS_00110:
      Motor.LeftSpeed = 0;
      Motor.RightSpeed = -3;
      Motor.Action = move_Forward;
      break;
    case LS_01000:
      Motor.LeftSpeed = -3;
      Motor.RightSpeed = 3;
      Motor.Action = move_Forward;
      break;
    case LS_01100:
      Motor.LeftSpeed = -3;
      Motor.RightSpeed = 0;
      Motor.Action = move_Forward;
      break;
    case LS_10000:
      Motor.LeftSpeed = -2;
      Motor.RightSpeed = -2;
      Motor.Action = rotationInPlace_Left;
      break;
    case LS_11000:
      Motor.LeftSpeed = -5;
      Motor.RightSpeed = 2;
      Motor.Action = move_Forward;
      break;
    default:
//      if (LedStript->Led_StatusPin == 0x10 || LedStript->Led_StatusPin == 0x18)
//      {
////        Semaphor_CloseRFID = 1 ;
////        for (int var = 0; var < 15; ++var)
////        {
////          HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
////          HAL_Delay(5);
////        }
////        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
//      }
      break;
    }
  } else ///Obs³uga jazdy do ty³u
  {
    switch (LedStript->Led_StatusPin)
    {
    case LS_miss:
      break;
    case LS_all:
      break;
    case LS_00001:
      Motor.LeftSpeed = -2;
      Motor.RightSpeed = -2;
      Motor.Action = rotationInPlace_Right;
      break;
    case LS_00010:
      Motor.LeftSpeed = 3;
      Motor.RightSpeed = -3;
      Motor.Action = move_Back;

      break;
    case LS_00011:
      Motor.LeftSpeed = 2;
      Motor.RightSpeed = -5;
      Motor.Action = move_Back;
      break;
    case LS_00100:
      LedStript->MissLineError = 0;
      Motor.LeftSpeed = 0;
      Motor.RightSpeed = 0;
      Motor.Action = move_Back;
      break;
    case LS_00110:
      Motor.LeftSpeed = -3;
      Motor.RightSpeed = 0;
      Motor.Action = move_Back;
      break;
    case LS_01000:
      Motor.LeftSpeed = 3;
      Motor.RightSpeed = -3;
      Motor.Action = move_Back;
      break;
    case LS_01100:
      Motor.LeftSpeed = 0;
      Motor.RightSpeed = -3;
      Motor.Action = move_Back;
      break;
    case LS_10000:
      Motor.LeftSpeed = -2;
      Motor.RightSpeed = -2;
      Motor.Action = rotationInPlace_Left;
      break;
    case LS_11000:
      Motor.LeftSpeed = -5;
      Motor.RightSpeed = 2;
      Motor.Action = move_Back;
      break;
    default:
      if (LedStript->Led_StatusPin == 0x10 || LedStript->Led_StatusPin == 0x18)
      {
//        Semaphor_CloseRFID = 0;
//        Flag_Close_RFID = 0 ;
//        Semaphor_NoReadRFID = 0 ;
//        for (int var = 0; var < 15; ++var)
//        {
//          HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
//          HAL_Delay(5);
//        }
//        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
      }
      break;
    }
  }
  return Motor;
}

