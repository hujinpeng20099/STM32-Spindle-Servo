#include "modbus_com.h"
#include "mbrtuslave.h"
#include "mc_config.h"

#define M_ADDRESS							05

extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern UART_HandleTypeDef huart3;

uint8_t rs485_rx_byte;
uint16_t mcudRxLength=0;
uint8_t mcudRxBuffer[MCUDRECEIVELENGTH];

/* setting one register
adress 2000	sp_ctrl 2 byte
adress 2001 speed_set 2 byte 
*/

void SetSingleRegister(uint16_t registerAddress,uint16_t registerValue)
{
	if(registerAddress == 2000)
	{
		if(registerValue == 0x01)//start motor
		{
			MCI_StartMotor(pMCI[0]);		
		}
		else if(registerValue == 0x10)//fault ack
		{
			MCI_IsCommandAcknowledged(pMCI[0]);
		}
		else //shutdown motor
		{
			MCI_StopMotor(pMCI[0]);
		}
	}
	if(registerAddress == 2001)
	{
		if(registerValue<=10000)
		{
			MCI_ExecSpeedRamp(pMCI[0],(int16_t)((registerValue * SPEED_UNIT) / U_RPM),2000);
		}	
	}
}

/* uart send data */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(mcudRxLength>=MCUDRECEIVELENGTH)
	{
			mcudRxLength=0;
	}
	if(huart->Instance == USART3)
	{
		/*recive data error*/
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE)!=RESET)
		{
				HAL_UART_Receive_IT(&huart3,&rs485_rx_byte,1);
		}			
		if((0!=mcudRxLength)||(M_ADDRESS==rs485_rx_byte))
		{
				mcudRxBuffer[mcudRxLength++] = rs485_rx_byte;
		}
		HAL_UART_Receive_IT(&huart3,&rs485_rx_byte,1);
	}
}


static void McudSendData(uint8_t *sData,uint16_t sSize)
{
	mcudRxLength=0;
	HAL_GPIO_WritePin(RS_DE_GPIO_Port, RS_DE_Pin, GPIO_PIN_RESET);	
	HAL_UART_Transmit(&huart3,sData,sSize,1000);
	HAL_GPIO_WritePin(RS_DE_GPIO_Port, RS_DE_Pin, GPIO_PIN_SET);
}


void LocalSlaveProcess(void)
{
	uint8_t respondBytes[MCUDRECEIVELENGTH];
	uint16_t respondLength=0;    
	if(mcudRxLength>=8)
	{
			respondLength = ParsingMasterAccessCommand(mcudRxBuffer,respondBytes,mcudRxLength,M_ADDRESS);
			if(respondLength!=65535)
			{
					/*send resp data */
					McudSendData(respondBytes,respondLength);
			}
	}
	if(pMCI[0]->State == RUN)
	{
		HAL_GPIO_WritePin(LED_S_GPIO_Port,LED_S_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_S_GPIO_Port,LED_S_Pin,GPIO_PIN_RESET);
	}
}

/* reading multi register
adress 2002	sp_status 2 byte
adress 2003 sp_power 2 byte 
adress 2004 sp_current 2 byte 
adress 2005 sp_speed 2 byte 
*/
void GetHoldingRegister(uint16_t startAddress,uint16_t quantity,uint16_t *registerValue)
{
	if(startAddress == 2002)
	{
		registerValue[0] = pMCI[0]->State;//motor status
		registerValue[1] = PQD_GetAvrgElMotorPowerW(pMPM[M1]);//power w
		if((uint16_t)MCI_GetIqdref(pMCI[0]).q < 60000)
		{
			registerValue[2] = (uint16_t)MCI_GetIqdref(pMCI[0]).q;//d current
		}
		registerValue[3] = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCI[0]) * U_RPM) / SPEED_UNIT);//speed
	}
}
