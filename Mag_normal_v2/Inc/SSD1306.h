/**
  ******************************************************************************
  * @file    SSD1306.h
  * @author  Waveshare Team
  * @version 
  * @date    13-October-2014
  * @brief   This file contains all the functions prototypes for the SSD1306 OLED firmware driver.
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
#ifndef _SSD1306_H_
#define _SSD1306_H_

/* Includes ------------------------------------------------------------------*/
#include "MacroAndConst.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

 void ssd1306_clear_screen(uint8_t chFill);
 void ssd1306_refresh_gram(void);
 void ssd1306_draw_point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint);
 void ssd1306_fill_screen(uint8_t chXpos1, uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot);
 void ssd1306_display_char(uint8_t chXpos, uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode);
 void ssd1306_display_num(uint8_t chXpos, uint8_t chYpos, uint32_t chNum, uint8_t chLen,uint8_t chSize);
 void ssd1306_display_string(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchString, uint8_t chSize, uint8_t chMode);
 void ssd1306_draw_1616char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
 void ssd1306_draw_3216char(uint8_t chXpos, uint8_t chYpos, uint8_t chChar);
 void ssd1306_draw_bitmap(uint8_t chXpos, uint8_t chYpos, const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight);
 extern void ssd1306_init(void);

 //END library ssd1306

 //User interface
 void ssd1306_hello_word();


 //ENE user interface

#endif

/*-------------------------------END OF FILE-------------------------------*/


