#include "ss_modbus.h"
#include "stdbool.h"
#include "ui.h"

extern ui_data_t ui_data;

uint8_t reg_data = 0;


/* setting multi register
adress 2000	power 2 byte
adress 2001 current 2 byte 
adress 2002 speed 2 byte 
*/


void SetMultipleRegister(uint16_t startAddress,uint16_t quantity,uint16_t *registerValue)
{
	if(startAddress == 2000)
	{
		ui_data.power = registerValue[0]; 
		ui_data.current = registerValue[1];
		ui_data.speed = registerValue[2];
	}
}
