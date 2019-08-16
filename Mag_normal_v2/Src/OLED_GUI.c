/*
 * OLED_GUI.c
 *
 *  Created on: 08.08.2019
 *      Author: kkarp
 */

#include "OLED_GUI.h"

void OLED_GUI_Hello()
{
	ssd1306_hello_word();
}

void OLED_GIU(structdef_OLED_GUI* OLEDdata,StatusMachine SM)
{
	switch (SM) {
		case eDisplayCard:
			/*box cardID*/
			ssd1306_display_num(0, 45, OLEDdata->LastCard[0], 3, 14);
			ssd1306_display_char(21, 45, '-', 14, 1);
			ssd1306_display_num(28, 45, OLEDdata->LastCard[1], 3, 14);
			ssd1306_display_char(49, 45, '-', 14, 1);
			ssd1306_display_num(56, 45, OLEDdata->LastCard[2], 3, 14);
			ssd1306_display_char(77, 45, '-', 14, 1);
			ssd1306_display_num(84, 45, OLEDdata->LastCard[3], 3, 14);
			/*sector cardID*/
			ssd1306_display_num(0, 15, OLEDdata->LastSector[0], 3, 14);
			ssd1306_display_char(21, 15, '-', 14, 1);
			ssd1306_display_num(28, 15, OLEDdata->LastSector[1], 3, 14);
			ssd1306_display_char(49, 15, '-', 14, 1);
			ssd1306_display_num(56, 15, OLEDdata->LastSector[2], 3, 14);
			ssd1306_display_char(77, 15, '-', 14, 1);
			ssd1306_display_num(84, 15, OLEDdata->LastSector[3], 3, 14);
			/*Refresh OLED screen */
			ssd1306_refresh_gram();
			break;
		case eDisplayGUIChargeing:

					break;
		case eDisplayStatus:

					break;
		case eDisplayVoltageAndTemp:
			asm("nop");
			float V25 = 0.76; // [Volts]
			float Avg_slope = 0.0025; //[Volts/degree]
			float SupplyVoltage = 3.3; // [Volts]
			float ADCResolution = 4095.0;
			char clearData[120] = {' '};
			char data[20] = { 0 };

			Vsense = (SupplyVoltage * OLEDdata->ADC_valueTemp) / ADCResolution; // Przeliczenie wartosci zmierzonej na napiecie
			Temperature = ((Vsense - V25) / Avg_slope) + 25; // Obliczenie temperatury

			float tmpVal = OLEDdata->ADC_valueVoltage * (SupplyVoltage/ADCResolution) * 4.27;
			int tmpInt1 = BatteryVoltage = tmpVal;
			float tmpFrac = tmpVal - tmpInt1;
			int tmpInt2 = trunc(tmpFrac * 10);

//			ssd1306_clear_screen(0x00);
			sprintf((char*) data, "%d.%dV", tmpInt1, tmpInt2);
			ssd1306_display_string(0, 0, (uint8_t*) (char*) clearData, 14, 1);
			ssd1306_refresh_gram();
			ssd1306_display_string(0, 0, (uint8_t*) (char*) data, 14, 1);

			 tmpVal = Temperature;
			 tmpInt1 = tmpVal;
			 tmpFrac = tmpVal - tmpInt1;
			 tmpInt2 = trunc(tmpFrac * 10);
			sprintf((char*) data, "%d.%dC", tmpInt1, tmpInt2);
			ssd1306_display_string(50, 0, (uint8_t*) (char*) data, 14, 1);
			ssd1306_refresh_gram();
					break;
		default:
			break;
	}
}
