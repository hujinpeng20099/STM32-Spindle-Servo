/**
  ******************************************************************************
  * @file    mc_perf.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Execution time measurement
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

#include "parameters_conversion.h"
#include "mc_perf.h"

void  MC_Perf_Measure_Init (MC_Perf_Handle_t *pHandle)
{
  uint8_t  i;
  Perf_Handle_t  *pHdl;

  /* Set Debug mod for DWT IP Enabling */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  if (DWT->CTRL != 0) {                         // Check if DWT is present
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;      // Enable Cycle Counter
  }

  for (i = 0; i<MC_PERF_NB_TRACES; i++) {
    pHdl = &pHandle->MC_Perf_TraceLog[i];
    pHdl->StartMeasure = 0;
    pHdl->DeltaTimeInCycle = 0;
    pHdl->min = UINT32_MAX;
    pHdl->max = 0;
  }
  pHandle->BG_Task_OnGoing = false;
  pHandle->AccHighFreqTasksCnt = 0;

}

void  MC_Perf_Clear(MC_Perf_Handle_t *pHandle)
{
  uint8_t  i;
  Perf_Handle_t  *pHdl;

  for (i = 0; i<MC_PERF_NB_TRACES; i++) {
    pHdl = &pHandle->MC_Perf_TraceLog[i];
    pHdl->DeltaTimeInCycle = 0;
    pHdl->min = UINT32_MAX;
    pHdl->max = 0;
  }
}

/**
 * @brief  Start the measure of a code section called in foreground.
 * @param  pHandle: handler of the performance measurement component
 * @param  CodeSection: code section to measure
 */
void  MC_Perf_Measure_Start (MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
  uint32_t StartMeasure = DWT->CYCCNT;
  pHandle->MC_Perf_TraceLog[CodeSection].StartMeasure = StartMeasure;
}

/**
 * @brief  Start the measure of a code section called in background
 * @param  pHandle: handler of the performance measurement component
 * @param  CodeSection: code section to measure
 */
void  MC_BG_Perf_Measure_Start (MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
  pHandle->BG_Task_OnGoing = true;
  pHandle->AccHighFreqTasksCnt = 0;
  uint32_t StartMeasure = DWT->CYCCNT;
  pHandle->MC_Perf_TraceLog[CodeSection].StartMeasure = StartMeasure;
}

/**
 * @brief  Stop the measurement of a code section and compute elapse time
 * @param  pHandle: handler of the performance measurement component
 * @param  CodeSection: code section to measure
 */
void  MC_Perf_Measure_Stop (MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
  uint32_t StopMeasure;
  Perf_Handle_t *pHdl;

  StopMeasure = DWT->CYCCNT;
  pHdl = &pHandle->MC_Perf_TraceLog[CodeSection];

  /* Check Overflow cases */
  if (StopMeasure < pHdl->StartMeasure)
  {
    pHdl->DeltaTimeInCycle = (UINT32_MAX - pHdl->StartMeasure) + StopMeasure;
  }
  else
  {
    pHdl->DeltaTimeInCycle = StopMeasure - pHdl->StartMeasure;
  }

  if( pHandle->BG_Task_OnGoing )
  {
    pHandle->AccHighFreqTasksCnt += pHdl->DeltaTimeInCycle;
  }

  if (pHdl->max < pHdl->DeltaTimeInCycle) { pHdl->max = pHdl->DeltaTimeInCycle; }
  if (pHdl->min > pHdl->DeltaTimeInCycle) { pHdl->min = pHdl->DeltaTimeInCycle; }

}

/**
 * @brief  Stop the measurement of a code section in BG and compute elapse time.
 * @param  pHandle: handler of the performance measurement component
 * @param  CodeSection: code section to measure
 */
void  MC_BG_Perf_Measure_Stop (MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
  Perf_Handle_t *pHdl;

  uint32_t StopMeasure = DWT->CYCCNT;
  pHandle->BG_Task_OnGoing = false;

  pHdl  = &pHandle->MC_Perf_TraceLog[CodeSection];

  /* Check Overflow cases */
  if (StopMeasure < pHdl->StartMeasure)
  {
    pHdl->DeltaTimeInCycle = (UINT32_MAX - pHdl->StartMeasure) + StopMeasure;
  }
  else
  {
    pHdl->DeltaTimeInCycle = StopMeasure - pHdl->StartMeasure;
  }

  if (pHdl->DeltaTimeInCycle > pHandle->AccHighFreqTasksCnt)
  {
    pHdl->DeltaTimeInCycle -= pHandle->AccHighFreqTasksCnt;
  }
  if (pHdl->max < pHdl->DeltaTimeInCycle) { pHdl->max = pHdl->DeltaTimeInCycle; }
  if (pHdl->min > pHdl->DeltaTimeInCycle) { pHdl->min = pHdl->DeltaTimeInCycle; }

}

/**
 * @brief  It returns the current CPU load of both High and Medium frequency tasks
 * @param  pHandle: handler of the performance measurement component
 * @retval CPU load
 */
float MC_Perf_GetCPU_Load( MC_Perf_Handle_t * pHandle )
{
  float MFT_cpu_load;
  float HFT_cpu_load;

  MFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].DeltaTimeInCycle / SYSCLK_FREQ ) * MEDIUM_FREQUENCY_TASK_RATE) * 100;
  HFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTask].DeltaTimeInCycle / SYSCLK_FREQ ) * (PWM_FREQUENCY/REGULATION_EXECUTION_RATE)) * 100;

  return ( (float) (MFT_cpu_load + HFT_cpu_load) );
}

/**
 * @brief  It returns the maximum CPU load of both High and Medium frequency tasks
 * @param  pHandle: handler of the performance measurement component
 * @retval Max CPU load measured
 */
float MC_Perf_GetMaxCPU_Load( MC_Perf_Handle_t * pHandle )
{
  float MFT_cpu_load;
  float HFT_cpu_load;

  MFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].max / SYSCLK_FREQ ) * MEDIUM_FREQUENCY_TASK_RATE) * 100;
  HFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTask].max / SYSCLK_FREQ ) * (PWM_FREQUENCY/REGULATION_EXECUTION_RATE)) * 100;

  return ( (float) (MFT_cpu_load + HFT_cpu_load) );
}

/**
 * @brief  It returns the minimum CPU load of both High and Medium frequency tasks
 * @param  pHandle: handler of the performance measurement component
 * @retval Min CPU load measured
 */
float MC_Perf_GetMinCPU_Load( MC_Perf_Handle_t * pHandle )
{
  float MFT_cpu_load;
  float HFT_cpu_load;

  MFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].min / SYSCLK_FREQ ) * MEDIUM_FREQUENCY_TASK_RATE) * 100;
  HFT_cpu_load = (((float)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTask].min / SYSCLK_FREQ ) * (PWM_FREQUENCY/REGULATION_EXECUTION_RATE)) * 100;

  if (pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTask].min == UINT32_MAX)
  {
    HFT_cpu_load = 0;
  }
  return ( (float) (MFT_cpu_load + HFT_cpu_load) );
}

/************************ (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
