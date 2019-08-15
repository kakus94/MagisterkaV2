/*
 * OLED_GUI.h
 *
 *  Created on: 08.08.2019
 *      Author: kkarp
 */

#ifndef OLED_GUI_H_
#define OLED_GUI_H_

#include "SSD1306.h"
#include "spi.h"
#include "main.h"

//const float V25 = 0.76; // [Volts]
//const float Avg_slope = 0.0025; //[Volts/degree]
//const float SupplyVoltage = 3.3; // [Volts]
//const float ADCResolution = 4095.0;
//char clearData[120] = {' '};


typedef struct{
	uint16_t ADC_valueVoltage;
	uint16_t ADC_valueTemp;
	uint8_t LastCard[4];
	uint8_t LastSector[4];
}structdef_OLED_GUI;

typedef enum{
	eDisplayCard,
	eDisplayVoltageAndTemp,
	eDisplayGUIChargeing,
	eDisplayStatus
}StatusMachine;


void OLED_GUI_Hello();
void OLED_GIU(structdef_OLED_GUI* OLEDdata,StatusMachine SM);

#endif /* OLED_GUI_H_ */
