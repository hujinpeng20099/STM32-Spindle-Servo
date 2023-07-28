/**
  ******************************************************************************
  * @file    g4xx_bemf_ADC_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Sensorless Bemf acquisition with ADC component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup SpeednPosFdbk_Bemf
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BEMFADCFDBK_H
#define BEMFADCFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "speed_ctrl.h"
#include "pwm_common_sixstep.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */
  
/** @addtogroup G4XX_Bemf_ADC_SpeednPosFdbk
  * @{
  */
#define BEMF_BUFFER_LENGTH	((uint8_t) 128) /*!< Length of buffer used to store the bemf measurements */
#define SPEED_BUFFER_LENGTH     ((uint8_t) 18) /*!< Length of buffer used to store the timer period measurements */

/**
  * @brief This structure is used to handle the data of an instance of the Bemf component
  *
  */
typedef struct
{
  uint16_t Value[BEMF_BUFFER_LENGTH];   /*!< Array storing last values */
  uint8_t Index;                        /*!< current array index */
  uint16_t LastValues[3];
} Bemf_ADC_values_t;

/**
  * @brief  Bemf_ADC parameters definition
  */
typedef struct
{
  TIM_TypeDef * LfTim;                 /*!< It contains the pointer to the LF timer
                                           used for speed measurement. */										   
  uint32_t LfTimerChannel;             /*!< Channel of the LF timer used for speed measurement */ 
  ADC_TypeDef * pAdc[3];               /*!< Pointer to the ADC */
  uint32_t AdcChannel[3];              /*!< Array of ADC channels used for BEMF sensing */
  bool gpio_divider_available;         /*!< Availability of the GPIO port enabling the bemf resistor divider */
  GPIO_TypeDef * bemf_divider_port;    /*!< GPIO port enabling the bemf resistor divider */
  uint16_t bemf_divider_pin;           /*!< GPIO pin enabling the bemf resistor divider */
} Bemf_ADC_Params_t;

/**
  * @brief This structure is used to handle the thresholds for bemf zero crossing detection
  *
  */
typedef struct
{
  uint16_t AdcThresholdDown;        /*!< BEMF voltage threshold for zero crossing detection when BEMF is decreasing */
  uint16_t AdcThresholdUp;          /*!< BEMF voltage threshold for zero crossing detection when BEMF is increasing */  
  uint16_t SamplingPoint;           /*!< Pulse value of the timer channel used to trig the ADC when sensing occurs during PWM OFF time */
} Bemf_Sensing_Params;

/**
  * @brief This structure is used to handle the demagnetization time before starting bemf acquisition
  *
  */
typedef struct
{
  uint16_t DemagMinimumSpeedUnit;    /*!< Speed threshold for minimum demagnetization time */
  uint16_t RevUpDemagSpeedConv;      /*!< Convertion factor between speed and demagnetization time */
  uint16_t RunDemagSpeedConv;        /*!< Open loop convertion factor between speed and demagnetization time during */
  uint16_t DemagMinimumThreshold;    /*!< Minimum demagnetization time */
} Bemf_Demag_Params;

/**
  * @brief This structure is used to handle the data of an instance of the B-emf Feedback component
  *
  */
typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  Bemf_ADC_values_t BemfValues;	  /**< bemf measurements of phase  */
  uint8_t ZcEvents;
  Bemf_Sensing_Params Pwm_ON;
  Bemf_Sensing_Params Pwm_OFF;
  Bemf_Sensing_Params *pSensing_Params;
  bool IsOnSensingEnabled;     /*!< Value where 0 means BEMF is sensed during PWM OFF time and 1 or greater means BEMF is sensed during PWM ON time */  
  uint16_t OnSensingEnThres;   /*!< Pulse value of HF timer above which the PWM ON sensing is enabled */
  uint16_t OnSensingDisThres;  /*!< Pulse value of HF timer below which the PWM ON sensing is disabled */
  uint16_t Zc2CommDelay;               /*!< Zero Crossing detection to commutation delay in 15/128 degrees */
  Bemf_ADC_Params_t const *pParams_str;
  uint32_t LfTimerClock;
  bool ADCRegularLocked;       /*!< This flag is set when ADC is locked for bemf acquisition */
  bool SpeedErrorFlag;         /*!< This flag is set when an overvoltage occurs.*/
  uint8_t  SpeedBufferSize;    /*!< Size of the buffer used to calculate the average
                                 speed. It must be less than 18.*/
  uint32_t TimerPeriodBuffer[SPEED_BUFFER_LENGTH];/*!< Holding the last
                                         period captures */    
  uint8_t  TimerPeriodBufferIndex;                                   
  uint32_t  ElPeriodSum;       /* Period accumulator used to speed up the average speed computation*/
  uint32_t  ElPeriodBase;      /* Period accumulator used to speed up the average speed computation*/
  uint32_t hAvrTimerPeriod;
  int8_t hForcedDirection;     /*!< Instantaneous direction of rotor between two
                               captures*/
  uint32_t MinPeriod;          /*!< Minimum acceptable period of the timer for speed measurement */
  volatile uint8_t BufferFilled;   /*!< Indicate the number of speed measuremt
                                         present in the buffer from the start.
                                         It will be max SpeedBufferSize and it
                                         is used to validate the start of speed
                                         averaging. If bBufferFilled is below
                                         SpeedBufferSize the instantaneous
                                         measured speed is returned as average
                                         speed.*/
  uint16_t VariancePercentage;        /*!< Parameter expressing the maximum
                                           allowed variance of speed estimation
                                           */
  uint32_t PseudoPeriodConv;          /*!< Convertion factor between speed and timer period */
  int16_t MecSpeedUnit;
  uint8_t ReliabilityCounter; /*!< Counter for reliability check */
  uint8_t Reliability_hysteresys;    /*!< Number of reliability failed
                                           consecutive tests before a speed
                                           check fault is returned to _Super.bSpeedErrorNumber
                                           */
  uint16_t MinStartUpValidSpeed;     /*!< Absolute value of minimum mechanical
                                            speed  required to validate the start-up.
                                            Expressed in the unit defined by #SPEED_UNIT. */
  uint8_t StartUpConsistThreshold;   /*!< Number of consecutive tests on speed
                                           consistency to be passed before
                                           validating the start-up */
  bool IsSpeedReliable;        /*!< Latest private speed reliability information,
                                     updated by SPD_CalcAvrgMecSpeedUnit, it is
                                     true if the speed measurement variance is
                                     lower then threshold corresponding to
                                     hVariancePercentage */
  bool IsAlgorithmConverged;   /*!< Boolean variable containing observer
                                     convergence information */
  bool IsLoopClosed;           /*!< Boolean variable containing speed
                                     loop status*/   
  int16_t MeasuredElAngle;
  bool ZcDetected;             /*!< This flag is set when zero crossing is detected */  
  int16_t hElAngleAccu;        /*!< Electrical angle accumulator */  
  int16_t hElAngleBase;        /*!< Electrical angle accumulator */ 
  uint16_t DemagCounter;
  uint16_t DemagCounterThreshold;
  Bemf_Demag_Params const DemagParams;
  DrivingMode_t DriveMode;
  int16_t hVirtualMecSpeedUnit;
} Bemf_ADC_Handle_t;

/* Exported functions --------------------------------------------------------*/

/* It initializes all the object variables. */
void BADC_Init( Bemf_ADC_Handle_t *pHandle );

/* It resets the ADC status and empties arrays. */
void BADC_Clear( Bemf_ADC_Handle_t *pHandle );

/* It gets ADC value and check for zero crossing detection.*/
bool BADC_IsZcDetected( Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC);

/* It starts speed measurement. */
void BADC_SpeedMeasureOn(Bemf_ADC_Handle_t *pHandle );

/* It stops speed measurement. */
void BADC_SpeedMeasureOff(Bemf_ADC_Handle_t *pHandle );

/* It sets the trigger point of the ADC */
void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC, SpeednTorqCtrl_Handle_t *pHandleSTC );

/* Computes the rotor average mechanical speed in the unit defined by #SPEED_UNIT and returns it in pMecSpeedUnit. */
bool BADC_CalcAvrgMecSpeedUnit( Bemf_ADC_Handle_t * pHandle, int16_t * pMecSpeedUnit );

/* Callback of the speed loop timer. It updates the electrical angle. */
void * BADC_LfTim_UP_IRQHandler( Bemf_ADC_Handle_t *pHandle );

/* @brief  forces the rotation direction. */
void BADC_SetDirection( Bemf_ADC_Handle_t * pHandle, uint8_t direction );

/* It checks whether the state observer algorithm converged.*/
bool BADC_IsObserverConverged( Bemf_ADC_Handle_t * pHandle );

/* It starts the bemf acquisition.*/
void BADC_Start(Bemf_ADC_Handle_t *pHandle, uint8_t step);

/* It stops the bemf acquisition.*/
void BADC_Stop(Bemf_ADC_Handle_t *pHandle);

/* It selects the phase for the bemf acquisition.*/
void BADC_SelectAdcChannel(Bemf_ADC_Handle_t * pHandle, uint8_t Phase);

/* It updates the estimated electrical angle.*/
int16_t BADC_CalcElAngle(Bemf_ADC_Handle_t * pHandle);

/* It computes the estimated average electrical speed ElSpeedDpp expressed in dpp. */
void BADC_CalcAvrgElSpeedDpp( Bemf_ADC_Handle_t * pHandle );

/* It configures the sensorless speed loop timer in open loop operation. */
void BADC_SetLfTimerRevUp(Bemf_ADC_Handle_t * pHandle, int16_t hAvrMecSpeedUnit);

/* It computes the demagnetization time during revup procedure. */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC );

/* It computes the demagnetization time in closed loop operation.*/
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC );

/* It sets the flag when switch over phase ends.*/
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle);

/* It returns last converted Back-emf value.*/
uint16_t BADC_GetLastBemfValue(Bemf_ADC_Handle_t *pHandle, uint8_t phase);

/* It returns the zero crossing detection flag. */
bool BADC_GetBemfZcrFlag(Bemf_ADC_Handle_t *pHandle);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* BEMFADCFDBK_H */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
