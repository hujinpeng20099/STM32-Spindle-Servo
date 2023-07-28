#include "as5047.h"
#include "lcd.h"
#include "math.h"
#include "stdlib.h"


SPI_HandleTypeDef as_spi;
TIM_HandleTypeDef htim1;

static uint16_t parity(uint16_t x)
{
	uint16_t parity = 0;

	while(x != 0)
	{
		parity ^= x;
		x >>= 1;
	}
	return (parity & 0x1);
}

static void AS5047_Write(uint16_t addr,uint16_t data)
{
	if (parity(addr & 0x3FFF) == 1) addr = addr | 0x8000; // set parity bit
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&as_spi, (uint8_t*) &addr, 1, 100) != HAL_OK)
	{
//		Error_Handler();
	}
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_SET);
	
	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit
	
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&as_spi, (uint8_t*) &data, 1, 100) != HAL_OK)
	{
//		Error_Handler();
	}
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_SET);
}

uint16_t AS5047_Read(uint16_t addr)
{
	uint16_t data;
	if (parity(addr | AS4047_RD) == 1) addr = addr | 0x8000; // set parity bit
	addr = addr | AS4047_RD; // it's a read command
	
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(&as_spi, (uint8_t*) &addr,(uint8_t*)&data,1, 100) != HAL_OK)
	{
//		Error_Handler();
	}
	HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_SET);
	data = data & 0x3FFF;  // filter bits outside data, strip bit 14..15
	return data;	
}

void AS5047_Init(void)
{
	/*init spi for com with as5047*/
  as_spi.Instance = SPI1;
  as_spi.Init.Mode = SPI_MODE_MASTER;
  as_spi.Init.Direction = SPI_DIRECTION_2LINES;
  as_spi.Init.DataSize = SPI_DATASIZE_16BIT;
  as_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  as_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  as_spi.Init.NSS = SPI_NSS_SOFT;
  as_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  as_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  as_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  as_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  as_spi.Init.CRCPolynomial = 7;
  as_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  as_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&as_spi) != HAL_OK)
  {
//    Error_Handler();
  }	
	//config AB output 400 pulse
	AS5047_Write(AS4047_SETTINGS2,0x53);
}

float AS5047_Get_Angle(void)
{
	float angle = 0;
	angle = AS5047_Read(AS4047_ANGLECOM);
	angle = (float)((angle* 360.0f) / 16383.0f);
	return angle;
}




