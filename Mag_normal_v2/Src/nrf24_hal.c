#include "nrf24_hal.h"

// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
//void nRF24_GPIO_Init(void) {
//    GPIO_InitTypeDef PORT;
//
//    // Enable the nRF24L01 GPIO peripherals
//	RCC->APB2ENR |= nRF24_GPIO_PERIPHERALS;
//
//    // Configure CSN pin
//	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
//	PORT.GPIO_Speed = GPIO_Speed_2MHz;
//	PORT.GPIO_Pin = nRF24_CSN_PIN;
//	GPIO_Init(nRF24_CSN_PORT, &PORT);
//	nRF24_CSN_H();
//
//	// Configure CE pin
//	PORT.GPIO_Pin = nRF24_CE_PIN;
//	GPIO_Init(nRF24_CE_PORT, &PORT);
//	nRF24_CE_L();
//}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
extern SPI_HandleTypeDef hspi2;

uint8_t nRF24_LL_RW(uint8_t data) {
	uint8_t dataRX ;
	 // Wait until TX buffer is empty
//	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_TXE) == RESET);
	 while (RESET == ((READ_REG(hspi2.Instance->SR) & SPI_FLAG_TXE)));
	 HAL_SPI_TransmitReceive(&hspi2,&data,&dataRX,1,100);
	// Send byte to SPI (TXE cleared)
	//SPI_I2S_SendData(nRF24_SPI_PORT, data);
//	HAL_SPI_Transmit(&hspi1,&data,1,100);
	// Wait while receive buffer is empty
//	 while (RESET == ((READ_REG(hspi1.Instance->SR) & SPI_FLAG_RXNE)));
	// Return received byte
//	HAL_SPI_Receive(&hspi1,(uint8_t* )dataReciver,1,100);
	//return (uint8_t)SPI_I2S_ReceiveData(nRF24_SPI_PORT);
	return (uint8_t)dataRX;
}
