/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gfxsimulator.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSD1306.h"
#include "RC522_driver.h"
#include "ESP_8266_driver.h"
#include "FIFO_driver.h"
#include "DEVICE_driver.h"
#include "Motor_Control.h"
#include "LED_Strip.h"
#include "Home_Driver.h"
#include "cardSystem.h"
#include "OLED_GUI.h"
#include <string.h>
#include "nrf24.h"
#include "Robot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4*n)))
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Tablica kard i sektorow */
Stos_typeDef* STOS_CardSector;
Stos_typeDef STOS_Read;

/* Variables ESP */
uint8_t Received[100];
bool timeouted;

uint8_t NumberOfTags = 6;
uint8_t TempSectorData[4] = { 0xAA, 0xAA, 0xAA, 0xAA };
uint8_t TempData[32];
uint8_t TempTagsData[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		16, 17, 18, 19, 20, 21, 22, 23, 24 };
PROTOCOL_FrameTypeDef p_frame;

/**
 * FIFO buffers
 */
volatile uint8_t Fifo_Tx[FIFO_BUF_SIZE];
volatile uint8_t Fifo_Rx[FIFO_BUF_SIZE];
FIFO_ApiTypeDef FIFO_TX = { Fifo_Tx, 0, 0, 0, FIFO_BUF_SIZE };
FIFO_ApiTypeDef FIFO_RX = { Fifo_Rx, 0, 0, 0, FIFO_BUF_SIZE };

/**
 * Linear Buffers
 */
uint8_t LinBuff[FIFO_BUF_SIZE];
uint8_t TagData[FIFO_BUF_SIZE];
PROTOCOL_LinearBuffer_ApiTypeDef LinearBuffer = { LinBuff, 0,
FIFO_BUF_SIZE };
PROTOCOL_LinearBuffer_ApiTypeDef TagDataStuct = { TagData, 0,
FIFO_BUF_SIZE };

/* Private variable to RFID */
unsigned char CardID[4];
unsigned char MyID[4] = { 0x6A, 0x08, 0xE7, 0xAB }; //My card on my keys
uint8_t HomeCardID[4] = { 102, 7, 171, 20 };
uint8_t LastCard[4];
uint8_t LastSector[4];
uint8_t data[50];
uint16_t RFID_size = 0;
RFID_type_def rfid1;
RFID_type_def rfid2;
uint8_t rfid_id = 0;
uint8_t rfid_onRead = 0;
uint8_t Home_flag = 0;
uint16_t Motor_Left_impulse = 0;
uint16_t Motor_Right_impulse = 0;

/* Private variable to driver motors */
volatile double error1;
volatile double error2;
volatile int16_t speed = 5;
Motor_InitTypeDef MotorRight;
Motor_InitTypeDef MotorLeft;
MotorPID_InitTypeDef MotorPID_Left;
MotorPID_InitTypeDef MotorPID_Right;
LedStrip_InitTypeDef LedStrip;
LedStrip_Speed_InitTypeDef LedStrip_Speed;

/* Variable flag to clock system */
volatile uint8_t FlagPID;
volatile uint16_t Flag_read_card;
volatile uint8_t FlagRead_LedStrip;
volatile uint16_t FlagRX_th;
volatile uint8_t ADC_flag;
volatile uint16_t Backwards_timerStop;

/* Variable debugging */
volatile uint8_t watek1, watek2, watek3;

/* variable OLDE */
uint16_t OLED_refreshFlag;
uint8_t OLED_refreshOn;
structdef_OLED_GUI OLED_data;

/* NRF variable */
nRF24_RXResult pipe;
uint8_t nRF24_payloadRx[32];
uint8_t nRF24_payloadTx[32];
uint8_t payload_length;
nRF24_TXResult tx_res;

/* Robot */
Robot_Data RobotData;
Config RobotConfig;
Status RobotStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (UART7 == huart->Instance) {
		ITM_SendChar(Received[0]);
		FIFO_PutByte(&FIFO_RX, *Received);
		if (FIFO_RX.length > 16) {
		}
		HAL_UART_Receive_IT(&huart7, Received, 1); // Ponowne w³¹czenie nas³uchiwania
	}
}

void PrepareFrame(uint8_t* data, uint8_t cmd) {
	p_frame.header.dst_address = BROADCAST_ADDRESS;
	p_frame.header.src_address = DEVICE_ADDRESS;
//  PROTOCOL_FrameTypeDef* temp = (PROTOCOL_FrameTypeDef*)data;
	p_frame.header.length = sizeof(PROTOCOL_HeaderTypeDef) + (strlen(data));
	p_frame.header.command = cmd;
	p_frame.header.response = true;
	uint8_t size = strlen(data);
	memcpy(p_frame.resp_payload.get_data_payload.tags_data, data, size);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	RobotData.config = & RobotConfig;
	RobotData.status = & RobotStatus;
	RobotData.stos = &STOS_CardSector;
	RobotData.ActionsPerformed=1;
	/* Variable used to convert internal temperature */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	Scan_falg = 0;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	MX_SPI6_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_UART7_Init();
	MX_GFXSIMULATOR_Init();
	MX_ADC1_Init();
	MX_TIM12_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	/* Set cs and reset in high state */
	HAL_GPIO_WritePin(RFID1_CS_GPIO_Port, RFID1_CS_Pin, SET);
	HAL_GPIO_WritePin(RFID2_CS_GPIO_Port, RFID2_CS_Pin, SET);
	HAL_GPIO_WritePin(RFID1_RST_GPIO_Port, RFID1_RST_Pin, SET);
	HAL_GPIO_WritePin(RFID2_RST_GPIO_Port, RFID2_RST_Pin, SET);

	/* display initialization */
	ssd1306_init();
	ssd1306_clear_screen(0x00);
	ssd1306_hello_word();

	/* MFRC522 initialization  */
	HAL_Delay(300);
	MFRC552_preinit(&rfid1, &rfid2);
	SPI_use_instance = &rfid2;  //	assigning an rfid instance
	MFRC522_Init();
	SPI_use_instance = &rfid1;	//	assigning an rfid instance
	MFRC522_Init();

	/* end of initialization, sound signal */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);

	/* NRF24 initialization */
	nRF24_CE_H();
	HAL_Delay(500);
	nRF24_CE_L();
	nRF24_Check();
	nRF24_Init();
	HAL_Delay(200);
	nRF24_setConfiguration();
	HAL_Delay(200);
	nRF24_PrintSetting();

	/* initialization of encoders and timer */
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim12);				 // start system timer
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); 	 // start of timer interrupts

	/* initialization led stript */
	vLedStrip_Init(&LedStrip);

	/* Initialization motors and regulator PID */
	vMotor_init(&MotorLeft, &MotorRight);
	vMotorPID_init(&MotorPID_Left, &MotorPID_Right);

	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	//TODO: delete this and implement application value task
	MotorPID_Left.ValueTask = 4;
	MotorPID_Right.ValueTask = 4;

	/* Inicialization ESP */
//	HAL_NVIC_EnableIRQ(UART7_IRQn);
//	HAL_UART_Receive_IT(&huart7, Received, 1);
//	myESP_8266_InitClient(1, "ESP8266_EMPE", "1QWERTY7", SERVER_PORT);
//
//	HAL_StatusTypeDef status = HAL_ERROR;
//	HAL_UART_Receive_IT(&huart7, Received, 1);
//	FIFO_Clear(&FIFO_RX);
//	PROTOCOL_LinBuffClr(&LinearBuffer);
//
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_tab, 3);
	printf("sytem init\n\r");
	ssd1306_clear_screen(0x00);
//	noSendTCP = 1;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (RobotNrfTimer > 500 ) {
			RobotNrfTimer = 0;
			RobotData.FirstCall = Robot_CheckBufforNrf();
			if(RobotData.FirstCall)
			{
				RobotData.ActionsPerformed = 0;
				Robot_PopBuffer(&RobotData);
				Robot_PerformAction(&RobotData);
				//Robot_ECHO(&RobotData);
			}
			if(!RobotData.ActionsPerformed)
			{
				Robot_ChangeTX();
				if(Robot_SendData(&RobotData) == nRF24_TX_SUCCESS) RobotData.ActionsPerformed = 1;
				Robot_ChangeRX();
			}


			switch (RobotData.config->state) {
			case eRobotStop:
				vMotor_Control(&MotorLeft, BreakeSoft);
				vMotor_Control(&MotorRight, BreakeSoft);
				vMotorPID_clear(&MotorPID_Left, &MotorPID_Right);
				Scan_falg = 0;
				break;
			case eRobotMove:
				Scan_falg = 1;
				vMotorPID_clear(&MotorPID_Left, &MotorPID_Right);
				break;
			default:
				break;
			}

		}

		if (Start_charging) {
			Home_flag = 0;
			vMotor_Control(&MotorLeft, BreakeSoft);
			vMotor_Control(&MotorRight, BreakeSoft);
			Charging = TRUE;
			Start_charging = FALSE;
		}
		if (Charging) {
			if (BatteryVoltage > 12.4) {
				Charging = FALSE;
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
				Charging = FALSE;
				vMotorPID_clear(&MotorPID_Left, &MotorPID_Right);
				MotorPID_Left.ValueTask = 3;
				MotorPID_Right.ValueTask = 3;
				MotorPID_Left.e_sum = 0;
				MotorPID_Right.e_sum = 0;
				vMotor_Control(&MotorLeft, Back);
				vMotor_Control(&MotorRight, Back);
				Semafor_BackHome = TRUE;
			}
		}

		if (Backwards_timerStop > 3000) {
			Backwards_timerStop = 0;
			Semafor_BackHome = FALSE;
			vMotor_Control(&MotorLeft, BreakeSoft);
			vMotor_Control(&MotorRight, BreakeSoft);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
		}

		if (Display_VT > 1000) {
			Display_VT = 0;
			OLED_data.ADC_valueTemp = ADC_tab[1];
			OLED_data.ADC_valueVoltage = ADC_tab[0];
			OLED_GIU(&OLED_data, eDisplayVoltageAndTemp);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* re-reading card reading support  */
		if (Flag_Close_RFID > 3000) {
			//TODO poprawic powielone zmienen + nie dzialajace przerwanie ponowienia
			if (Count_NoReadRFID > 2) {
				rfid_onRead = 0;
				Semaphor_CloseRFID = 0;
				Count_NoReadRFID = 0;
				Semaphor_NoReadRFID = 0;
				Flag_Close_RFID = 0;
			}
			Flag_Close_RFID = 0;
			Semaphor_CloseRFID = 0;
			Semaphor_NoReadRFID = 1;
			Count_NoReadRFID++;
		}

		if (Semaphor_CloseRFID && Flag_read_card > 250) { //TODO: wprowadzic enuma !!!, stworzyc funkcje switcha spi
			Flag_read_card = 0;
			if (rfid_id) {
				SPI_use_instance = &rfid1;
				rfid_id = RFID1;
			} else {
				SPI_use_instance = &rfid2;
				rfid_id = RFID2;
			}
			if (MFRC522_Check(CardID) == MI_OK) {
				OLED_refreshOn = 1;
				rfid_onRead = 0;
				Semaphor_CloseRFID = 0;
				Count_NoReadRFID = 0;
				Semaphor_NoReadRFID = 0;
				Flag_Close_RFID = 0;

				if (rfid_id) {
					for (int q = 0; q <= 3; q++)
						OLED_data.LastCard[q] = CardID[q];
					pushItem(&STOS_CardSector, &OLED_data.LastCard, &OLED_data.LastSector);
				} else {
					for (int q = 0; q <= 3; q++)
						OLED_data.LastSector[q] = CardID[q];
					if (Home_checkCard((uint8_t*) HomeCardID,
							(uint8_t*) CardID)) {
						NVIC_DisableIRQ(EXTI9_5_IRQn);
						Scan_falg = FALSE;
						HomeCard_enable = TRUE;
						Rotate90_flag = TRUE;
						Motor_Left_impulse = 0;
						Motor_Right_impulse = 0;
						MotorPID_Left.e_sum = 0;
						MotorPID_Right.e_sum = 0;
					}
				}
				OLED_GIU(&OLED_data, eDisplayCard);
			} else {
				printf("Nie wykryto karty \r\n");
			}

		}

		/* thread of the robot's movement rotate90 */
		if (Rotate90_flag && FlagRead_LedStrip > 10) {
			FlagRead_LedStrip = 0;
			Maneuver_StructInit maneuver;
			maneuver.finishImpulse = 900;
			maneuver.leftFinsh = Motor_Left_impulse;
			maneuver.rightFinsh = Motor_Right_impulse;
			MotorPID_Left.ValueTask = 3;
			MotorPID_Right.ValueTask = 3;
//			vMotorAction_LedStrip(&MotorLeft, &MotorRight,
//								rotationInPlace_Left);
			if (Home_ManeuverRotate(&MotorLeft, &MotorRight, rotateLeft,
					&maneuver)) {
				Rotate90_flag = 0;
				Home_flag = 1;
//				NVIC_EnableIRQ(EXTI9_5_IRQn);
			}
		}

		/* thread of the robot's movement home support */
		if (Home_flag && FlagRead_LedStrip >= 10) {
			FlagRead_LedStrip = 0;
			vLedStrip_ReadStatus(&LedStrip);
			LedStrip_Speed = Home_motorControl(&LedStrip);
			if ((speed + LedStrip_Speed.LeftSpeed) >= 0) {
				MotorPID_Left.ValueTask = speed + LedStrip_Speed.LeftSpeed;
			} else {
				MotorPID_Left.ValueTask = 0;
			}
			if ((speed + LedStrip_Speed.RightSpeed) >= 0) {
				MotorPID_Right.ValueTask = speed + LedStrip_Speed.RightSpeed;
			} else {
				MotorPID_Right.ValueTask = 0;
			}
			vMotorAction_LedStrip(&MotorLeft, &MotorRight,
					LedStrip_Speed.Action);
		}

		/* thread of the robot's movement support */
		if (Scan_falg && FlagRead_LedStrip >= 10) {
			FlagRead_LedStrip = 0;
			vLedStrip_ReadStatus(&LedStrip);
			LedStrip_Speed = vLed_control(&LedStrip, Semaphor_NoReadRFID);
			if ((speed + LedStrip_Speed.LeftSpeed) >= 0) {
				MotorPID_Left.ValueTask = speed + LedStrip_Speed.LeftSpeed;
			} else {
				MotorPID_Left.ValueTask = 0;
			}
			if ((speed + LedStrip_Speed.RightSpeed) >= 0) {
				MotorPID_Right.ValueTask = speed + LedStrip_Speed.RightSpeed;
			} else {
				MotorPID_Right.ValueTask = 0;
			}
			vMotorAction_LedStrip(&MotorLeft, &MotorRight,
					LedStrip_Speed.Action);
		}

		/* the thread of the PID Controller */
		if (FlagPID >= 10) {
			FlagPID = 0;
			if (Rotate90_flag) {
				uint16_t encoderCounterL = uGetCounterTim(
						MotorLeft.Tim_Encoder);
				uint16_t encoderCounterR = uGetCounterTim(
						MotorRight.Tim_Encoder);
				if (encoderCounterL < 500) {
					Motor_Left_impulse += encoderCounterL;
				} else {
					Motor_Left_impulse += (1000 - encoderCounterL);
				}
				if (encoderCounterR < 500) {
					Motor_Right_impulse += encoderCounterR;
				} else {
					Motor_Right_impulse += (1000 - encoderCounterR);
				}
			}
			vMotorPID_Control(&MotorPID_Left, &MotorLeft);
			vMotorPID_Control(&MotorPID_Right, &MotorRight);

			watek1 = TIM2->CNT;
			watek2 = TIM3->CNT;
			error1 = MotorPID_Left.ExecutionValue;
			error2 = MotorPID_Right.ExecutionValue;

			vMotor_SetPWM(&MotorLeft, error1);
			vMotor_SetPWM(&MotorRight, error2);

			vClearCounter(MotorLeft.Tim_Encoder);
			vClearCounter(MotorRight.Tim_Encoder);
		}
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//###############
//##############
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM12)
		Display_VT++;
	Flag_read_card++;
	FlagPID++;
	FlagRead_LedStrip++;
	ADC_flag++;
	CheckStatus_flag++;
	RobotNrfTimer++;
	if (Semaphor_CloseRFID) {
		Flag_Close_RFID++;
	}
	if (Semafor_BackHome) {
		Backwards_timerStop++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EXTI8_CloseCard_Pin) {
		OLED_refreshOn = 1;
		Semaphor_NoReadRFID = 0;  //
		Semaphor_CloseRFID = 1;
		rfid_onRead = 1;
	}
	if (GPIO_Pin == GPIO_PIN_3) {
		if (HomeCard_enable)
			Start_charging = TRUE;
		else
			Incident_flag = TRUE;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
