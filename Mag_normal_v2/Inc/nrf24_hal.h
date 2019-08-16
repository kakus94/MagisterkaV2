#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
//#include  "stm32f1xx_hal.h"
//#include  "stm32f1xx_hal_gpio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#include <main.h>


// SPI port peripheral
#define nRF24_SPI_PORT             SPI2

// nRF24 GPIO peripherals
//#define nRF24_GPIO_PERIPHERALS     (RCC_APB2ENR_IOPBEN)

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              NRF_CE_GPIO_Port
#define nRF24_CE_PIN               NRF_CE_Pin
#define nRF24_CE_L()               HAL_GPIO_WritePin(nRF24_CE_PORT,nRF24_CE_PIN,RESET)//GPIO_ResetBits(nRF24_CE_PORT, nRF24_CE_PIN)
#define nRF24_CE_H()               HAL_GPIO_WritePin(nRF24_CE_PORT,nRF24_CE_PIN,SET)//GPIO_SetBits(nRF24_CE_PORT, nRF24_CE_PIN)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             NRF_CSN_GPIO_Port
#define nRF24_CSN_PIN              NRF_CSN_Pin
#define nRF24_CSN_L()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN,RESET)
#define nRF24_CSN_H()              HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN,SET)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             NRF_IRQ_GPIO_Port
#define nRF24_IRQ_PIN              NRF_IRQ_Pin]


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
