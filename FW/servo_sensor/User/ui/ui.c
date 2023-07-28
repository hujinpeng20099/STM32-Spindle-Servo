#include "ui.h"
#include "lcd.h"

extern const unsigned char  pic[][32];


ui_data_t ui_data;

void lcd_bar(uint8_t x,uint8_t y,uint8_t per)
{
	if(per>=100)per = 100;
	
	per = per/10;
	
	if(per<=5)
	{
		for(uint8_t i=0;i<per;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',GREEN,BLACK,16,0);
		}
		for(uint8_t i=per;i<10;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',LGRAY,BLACK,16,0);
		}
	}
	else if(per>5&&per<=8)
	{
		for(uint8_t i=0;i<5;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',GREEN,BLACK,16,0);
		}		
		for(uint8_t i=5;i<per;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',YELLOW,BLACK,16,0);
		}	
		for(uint8_t i=per;i<10;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',LGRAY,BLACK,16,0);
		}		
	}
	else if(per>8&&per<=10)
	{
		for(uint8_t i=0;i<5;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',GREEN,BLACK,16,0);
		}		
		for(uint8_t i=5;i<8;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',YELLOW,BLACK,16,0);
		}	
		for(uint8_t i=8;i<per;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',RED,BLACK,16,0);
		}
		for(uint8_t i=per;i<10;i++)
		{
			LCD_ShowChar(x+8*i,y,'|',LGRAY,BLACK,16,0);
		}		
	}
}

void lcd_info(void)
{
	/*show power*/
	LCD_ShowChar(68,5,'w',WHITE,BLACK,24,0);
	
	/*show load percent*/
	LCD_ShowChar(0,36,'0',WHITE,BLACK,16,0);
	LCD_ShowChar(72,36,'%',WHITE,BLACK,16,0);
	
	/*show speed*/
	LCD_ShowIcon(0,82,(u8 *)pic[1],YELLOW,BLACK);
	LCD_ShowString(56,82,"rpm",WHITE,BLACK,16,0);
	
	/*show current*/
	LCD_ShowIcon(0,102,(u8 *)pic[2],YELLOW,BLACK);
	LCD_ShowString(56,102,"mA",WHITE,BLACK,16,0);
	
	/*show angle*/
	LCD_ShowIcon(0,122,(u8 *)pic[3],YELLOW,BLACK);
	LCD_ShowIcon(56,122,(u8 *)pic[0],WHITE,BLACK);
	/*show temp*/
	LCD_ShowIcon(0,142,(u8 *)pic[4],YELLOW,BLACK);
	LCD_ShowIcon(56,142,(u8 *)pic[0],WHITE,BLACK);
}

void lcd_desk(ui_data_t * ui_data)
{
	/*show power*/
	LCD_ShowIntNum(10,0,ui_data->power,3,GREEN,BLACK,32);
	
	/*show load percent*/
	LCD_ShowIntNum(48,36,(ui_data->power)*100/MAX_PWR,3,GREEN,BLACK,16);
	lcd_bar(0,54,(ui_data->power)*100/MAX_PWR);
	
	/*show speed*/
	LCD_ShowIntNum(18,82,ui_data->speed,4,GREEN,BLACK,16);
	
	/*show current*/
	LCD_ShowIntNum(18,102,ui_data->current,4,GREEN,BLACK,16);
	
	/*show angle*/
	LCD_ShowIntNum(26,122,ui_data->angle,3,GREEN,BLACK,16);
	
	/*show temp*/
	LCD_ShowIntNum(26,142,ui_data->temp,3,GREEN,BLACK,16);
}

