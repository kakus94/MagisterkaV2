/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
volatile uint16_t Flag_Close_RFID;
volatile uint8_t Semaphor_CloseRFID;
volatile uint8_t Semaphor_NoReadRFID;
volatile uint8_t Count_NoReadRFID;
volatile uint8_t Rotate90_flag;
volatile uint8_t Scan_falg;
volatile uint16_t CheckStatus_flag;
volatile uint8_t noSendTCP;
volatile uint8_t Start_charging;
volatile uint8_t Charging;
volatile uint8_t HomeCard_enable;

volatile uint8_t Incident_flag;
volatile uint8_t Semafor_BackHome;


volatile uint8_t popStos;
volatile uint8_t popDisplay;

volatile float BatteryVoltage;
volatile float BatteryVoltage_countMeasure;



/* Pomiar ADC i temp procesora */
/* Variable ADC */
uint16_t ADC_tab[2];
volatile uint16_t Display_VT;
uint16_t PomiarADC;
double Temperature;
float Vsense;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RFID1_CS_Pin GPIO_PIN_3
#define RFID1_CS_GPIO_Port GPIOE
#define RFID1_RST_Pin GPIO_PIN_4
#define RFID1_RST_GPIO_Port GPIOE
#define RFID2_CS_Pin GPIO_PIN_6
#define RFID2_CS_GPIO_Port GPIOF
#define RFID2_RST_Pin GPIO_PIN_10
#define RFID2_RST_GPIO_Port GPIOF
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOC
#define WIFI_RST_Pin GPIO_PIN_0
#define WIFI_RST_GPIO_Port GPIOG
#define WIFI_CHPD_Pin GPIO_PIN_1
#define WIFI_CHPD_GPIO_Port GPIOG
#define NRF_CSN_Pin GPIO_PIN_12
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_9
#define NRF_IRQ_GPIO_Port GPIOD
#define EXTI8_CloseCard_Pin GPIO_PIN_8
#define EXTI8_CloseCard_GPIO_Port GPIOG
#define EXTI8_CloseCard_EXTI_IRQn EXTI9_5_IRQn
#define INT4_Pin GPIO_PIN_8
#define INT4_GPIO_Port GPIOC
#define INT3_Pin GPIO_PIN_9
#define INT3_GPIO_Port GPIOC
#define INT2_Pin GPIO_PIN_8
#define INT2_GPIO_Port GPIOA
#define INT1_Pin GPIO_PIN_9
#define INT1_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_10
#define OLED_CS_GPIO_Port GPIOG
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOG
#define OLED_RES_Pin GPIO_PIN_12
#define OLED_RES_GPIO_Port GPIOG
#define LEDS_OUT1_Pin GPIO_PIN_7
#define LEDS_OUT1_GPIO_Port GPIOB
#define LEDS_OUT2_Pin GPIO_PIN_8
#define LEDS_OUT2_GPIO_Port GPIOB
#define LEDS_OUT3_Pin GPIO_PIN_9
#define LEDS_OUT3_GPIO_Port GPIOB
#define LEDS_OUT4_Pin GPIO_PIN_0
#define LEDS_OUT4_GPIO_Port GPIOE
#define LEDS_OUT5_Pin GPIO_PIN_1
#define LEDS_OUT5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define TRUE 1
#define FALSE 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
