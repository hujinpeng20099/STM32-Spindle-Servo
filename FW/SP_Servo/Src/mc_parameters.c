
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
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
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

#include "r3_2_g4xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx - Shared Resources
  *
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{
   .OPAMPx_1 = OPAMP1,
   .OPAMPx_2 = OPAMP2,
   .OPAMPx_3 = OPAMP3,
    // OPAMPMatrix is used to specify if the ADC source comes from internal channel of which OPAMP.

  .OPAMPSelect_1 = { OPAMP2
                   ,OPAMP1
                   ,OPAMP1
                   ,OPAMP1
                   ,OPAMP1
                   ,OPAMP2
                 },
  .OPAMPSelect_2 = { OPAMP3
                   ,OPAMP3
                   ,OPAMP3
                   ,OPAMP2
                   ,OPAMP2
                   ,OPAMP3
                  },

  .OPAMPConfig1 = { OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                 },
  .OPAMPConfig2 = { PIN_CONNECT
                   ,DIRECT_CONNECT
                   ,DIRECT_CONNECT
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,PIN_CONNECT
                  },
};
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ADC1,
  .ADCx_2 = ADC2,
  /* Motor Control Kit config */
   .ADCConfig1 = { ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },
   .ADCConfig2 = { ( uint32_t )( 12<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( VPOPAMP3<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( VPOPAMP3<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 3<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 12<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },

   .ADCDataReg1 = { ADC2
                 , ADC1
                 , ADC1
                 , ADC1
                 , ADC1
                 , ADC2
                 },
  .ADCDataReg2 =  { ADC1
                 , ADC2
                 , ADC2
                 , ADC2
                 , ADC2
                 , ADC1
                  },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
 .pwm_en_u_port      = MC_NULL,
 .pwm_en_u_pin       = (uint16_t) 0,
 .pwm_en_v_port      = MC_NULL,
 .pwm_en_v_pin       = (uint16_t) 0,
 .pwm_en_w_port      = MC_NULL,
 .pwm_en_w_pin       = (uint16_t) 0,

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = INT_MODE,

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMPParams     = &R3_3_OPAMPParamsM1,
/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = COMP1,
  .CompOCPAInvInput_MODE = INT_MODE,
  .CompOCPBSelection     = COMP2,
  .CompOCPBInvInput_MODE = INT_MODE,
  .CompOCPCSelection     = COMP4,
  .CompOCPCInvInput_MODE = INT_MODE,
  .DAC_OCP_ASelection    = DAC3,
  .DAC_OCP_BSelection    = DAC3,
  .DAC_OCP_CSelection    = DAC3,
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_1,
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_2,
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_2,

  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  9803,
  .DAC_OVP_Threshold =  23830,

};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

