#ifndef __UI_H
#define __UI_H
#include "stm32g0xx_hal.h"

#define MAX_PWR	750

typedef struct
{
	uint8_t temp;
	uint16_t speed;
	uint16_t power;
	uint16_t angle;
	uint16_t current;
}ui_data_t;

void lcd_bar(uint8_t x,uint8_t y,uint8_t per);
void lcd_info(void);
void lcd_desk(ui_data_t * ui_data);
#endif

