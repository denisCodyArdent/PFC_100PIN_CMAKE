/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim.c
  * @brief   This file provides code for the configuration
  *          of the HRTIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "hrtim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

HRTIM_HandleTypeDef hhrtim1;
DMA_HandleTypeDef hdma_hrtim1_a;
DMA_HandleTypeDef hdma_hrtim1_b;
DMA_HandleTypeDef hdma_hrtim1_e;
DMA_HandleTypeDef hdma_hrtim1_d;
DMA_HandleTypeDef hdma_hrtim1_c;
DMA_HandleTypeDef hdma_hrtim1_f;

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_EventCfgTypeDef pEventCfg = {0};
  HRTIM_FaultCfgTypeDef pFaultCfg = {0};
  HRTIM_FaultBlankingCfgTypeDef pFaultBlkCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_CaptureCfgTypeDef pCaptureCfg = {0};
  HRTIM_TimerEventFilteringCfgTypeDef pTimerEventFilteringCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_EventPrescalerConfig(&hhrtim1, HRTIM_EVENTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV1SRC_COMP2_OUT;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_HIGH;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_LEVEL;
  pEventCfg.FastMode = HRTIM_EVENTFASTMODE_DISABLE;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_1, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV2SRC_COMP4_OUT;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_2, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV3SRC_COMP6_OUT;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_3, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV4SRC_COMP1_OUT;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_LOW;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_4, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV5SRC_COMP7_OUT;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_HIGH;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_RISINGEDGE;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_5, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV6SRC_GPIO;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_LEVEL;
  pEventCfg.Filter = HRTIM_EVENTFILTER_NONE;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_6, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV7SRC_GPIO;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_7, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV8SRC_COMP3_OUT;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_LOW;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_8, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV9SRC_COMP5_OUT;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_9, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV10SRC_GPIO;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_HIGH;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_10, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_INTERNAL;
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_HIGH;
  pFaultCfg.Filter = HRTIM_FAULTFILTER_NONE;
  pFaultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultBlkCfg.Threshold = 0;
  pFaultBlkCfg.ResetMode = HRTIM_FAULTCOUNTERRST_UNCONDITIONAL;
  pFaultBlkCfg.BlankingSource = HRTIM_FAULTBLANKINGMODE_RSTALIGNED;
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1, HRTIM_FAULTMODECTL_ENABLED);
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_2, HRTIM_FAULTMODECTL_ENABLED);
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_ENABLED);
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_4, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_LOW;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_4, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_4, HRTIM_FAULTMODECTL_ENABLED);
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_5, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_EEVINPUT;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_5, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_5, HRTIM_FAULTMODECTL_ENABLED);
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_6, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_INTERNAL;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_6, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_6, HRTIM_FAULTMODECTL_ENABLED);
  pTimeBaseCfg.Period = 2307;
  pTimeBaseCfg.RepetitionCounter = 0;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED;
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT1|HRTIM_TIMFAULTENABLE_FAULT4
                              |HRTIM_TIMFAULTENABLE_FAULT2|HRTIM_TIMFAULTENABLE_FAULT3
                              |HRTIM_TIMFAULTENABLE_FAULT5|HRTIM_TIMFAULTENABLE_FAULT6;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READONLY;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT2|HRTIM_TIMFAULTENABLE_FAULT5
                              |HRTIM_TIMFAULTENABLE_FAULT1|HRTIM_TIMFAULTENABLE_FAULT3
                              |HRTIM_TIMFAULTENABLE_FAULT4|HRTIM_TIMFAULTENABLE_FAULT6;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER1_CMP1;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_CPT1;
  pTimerCfg.DMASrcAddress = (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CPT1xR;;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_EEV_6;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMASrcAddress = (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CPT1xR;;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT3|HRTIM_TIMFAULTENABLE_FAULT6
                              |HRTIM_TIMFAULTENABLE_FAULT1|HRTIM_TIMFAULTENABLE_FAULT2
                              |HRTIM_TIMFAULTENABLE_FAULT4|HRTIM_TIMFAULTENABLE_FAULT5;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READONLY;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER1_CMP4;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_CPT1;
  pTimerCfg.DMASrcAddress = (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CPT1xR;;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER4_CMP1;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0xFFFD;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 100;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 4000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 150;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 65503;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCaptureCfg.Trigger = HRTIM_CAPTURETRIGGER_EEV_10;
  if (HAL_HRTIM_WaveformCaptureConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_CAPTUREUNIT_1, &pCaptureCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCaptureCfg.Trigger = HRTIM_CAPTURETRIGGER_EEV_7;
  if (HAL_HRTIM_WaveformCaptureConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_CAPTUREUNIT_1, &pCaptureCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCaptureCfg.Trigger = HRTIM_CAPTURETRIGGER_EEV_6;
  if (HAL_HRTIM_WaveformCaptureConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_CAPTUREUNIT_1, &pCaptureCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP1;
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_EVENT_1, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP3;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_2, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_ENABLED;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_4, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP1;
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_EVENT_4, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP3;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_5, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_7, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_8, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
  pDeadTimeCfg.RisingValue = 30;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 30;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP3;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMBEV1_TIMACMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMEEV1_TIMACMP4;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.RepetitionCounter = 0x00;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0xFFFD;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 100;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 10;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 150;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.RepetitionCounter = 0;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 10;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 100;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xFFFD;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspInit 0 */

  /* USER CODE END HRTIM1_MspInit 0 */
    /* HRTIM1 clock enable */
    __HAL_RCC_HRTIM1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PC6     ------> HRTIM1_EEV10
    PB4     ------> HRTIM1_EEV7
    PB5     ------> HRTIM1_EEV6
    */
    GPIO_InitStruct.Pin = SYNC_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_HRTIM1;
    HAL_GPIO_Init(SYNC_3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SYNC_2_Pin|SYNC_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* HRTIM1 DMA Init */
    /* HRTIM1_A Init */
    hdma_hrtim1_a.Instance = DMA1_Channel1;
    hdma_hrtim1_a.Init.Request = DMA_REQUEST_HRTIM1_A;
    hdma_hrtim1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hrtim1_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_a.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_a.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_a.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_a) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerA,hdma_hrtim1_a);

    /* HRTIM1_B Init */
    hdma_hrtim1_b.Instance = DMA1_Channel2;
    hdma_hrtim1_b.Init.Request = DMA_REQUEST_HRTIM1_B;
    hdma_hrtim1_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hrtim1_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_b.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_b.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_b.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_b.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_b) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerB,hdma_hrtim1_b);

    /* HRTIM1_E Init */
    hdma_hrtim1_e.Instance = DMA1_Channel3;
    hdma_hrtim1_e.Init.Request = DMA_REQUEST_HRTIM1_E;
    hdma_hrtim1_e.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_hrtim1_e.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_e.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_e.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_e.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_e.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_e.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_e) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerE,hdma_hrtim1_e);

    /* HRTIM1_D Init */
    hdma_hrtim1_d.Instance = DMA1_Channel6;
    hdma_hrtim1_d.Init.Request = DMA_REQUEST_HRTIM1_D;
    hdma_hrtim1_d.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_hrtim1_d.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_d.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_d.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_d.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_d.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_d.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_d) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerD,hdma_hrtim1_d);

    /* HRTIM1_C Init */
    hdma_hrtim1_c.Instance = DMA1_Channel7;
    hdma_hrtim1_c.Init.Request = DMA_REQUEST_HRTIM1_C;
    hdma_hrtim1_c.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_hrtim1_c.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_c.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_c.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_c.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_c.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_c.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_c) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerC,hdma_hrtim1_c);

    /* HRTIM1_F Init */
    hdma_hrtim1_f.Instance = DMA1_Channel8;
    hdma_hrtim1_f.Init.Request = DMA_REQUEST_HRTIM1_F;
    hdma_hrtim1_f.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hrtim1_f.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hrtim1_f.Init.MemInc = DMA_MINC_DISABLE;
    hdma_hrtim1_f.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hrtim1_f.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hrtim1_f.Init.Mode = DMA_CIRCULAR;
    hdma_hrtim1_f.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_hrtim1_f) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hrtimHandle,hdmaTimerF,hdma_hrtim1_f);

  /* USER CODE BEGIN HRTIM1_MspInit 1 */

  /* USER CODE END HRTIM1_MspInit 1 */
  }
}

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspPostInit 0 */

  /* USER CODE END HRTIM1_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**HRTIM1 GPIO Configuration
    PC8     ------> HRTIM1_CHE1
    PC9     ------> HRTIM1_CHE2
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    PA10     ------> HRTIM1_CHB1
    PA11     ------> HRTIM1_CHB2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_HRTIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN HRTIM1_MspPostInit 1 */

  /* USER CODE END HRTIM1_MspPostInit 1 */
  }

}

void HAL_HRTIM_MspDeInit(HRTIM_HandleTypeDef* hrtimHandle)
{

  if(hrtimHandle->Instance==HRTIM1)
  {
  /* USER CODE BEGIN HRTIM1_MspDeInit 0 */

  /* USER CODE END HRTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HRTIM1_CLK_DISABLE();

    /**HRTIM1 GPIO Configuration
    PC6     ------> HRTIM1_EEV10
    PC8     ------> HRTIM1_CHE1
    PC9     ------> HRTIM1_CHE2
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    PA10     ------> HRTIM1_CHB1
    PA11     ------> HRTIM1_CHB2
    PB4     ------> HRTIM1_EEV7
    PB5     ------> HRTIM1_EEV6
    */
    HAL_GPIO_DeInit(GPIOC, SYNC_3_Pin|GPIO_PIN_8|GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11);

    HAL_GPIO_DeInit(GPIOB, SYNC_2_Pin|SYNC_1_Pin);

    /* HRTIM1 DMA DeInit */
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerA);
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerB);
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerE);
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerD);
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerC);
    HAL_DMA_DeInit(hrtimHandle->hdmaTimerF);
  /* USER CODE BEGIN HRTIM1_MspDeInit 1 */

  /* USER CODE END HRTIM1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
