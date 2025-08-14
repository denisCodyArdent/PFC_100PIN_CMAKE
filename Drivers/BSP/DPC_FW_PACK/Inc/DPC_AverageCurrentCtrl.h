/**
  ******************************************************************************
  * @file    DPC_AvgeresisCurrentCtrl.h
  * @brief   This file contains the headers of the Avgeresis Current Ctrl Module.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AVERAGECURRENTCTRL_H
#define __AVERAGECURRENTCTRL_H


/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "DPC_CommonData.h"


/* Exported types ------------------------------------------------------------*/


//*** AVERAGE CURRENT CTRL of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ------------- ***/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
ErrorStatus DCP_AVGCC_FeedForwardInit(DPC_AVGCC_FeedForward_t* pFF,
                                    uint8_t Shift, uint8_t Tripping, 
                                    int32_t StepUpParam, int32_t StepDownParam,
                                    int32_t UpperThresholdParam, int32_t LowerThresholdParam);

int32_t DCP_AVGCC_FeedForwardControl(DPC_AVGCC_FeedForward_t* pFF, int32_t AvgValue, int32_t MovAvgValue,
                                   DPC_PID_PIs32_t* pPI, DPC_CMNDAT_PFC_ControlData_t* pCtrlData);

void DCP_AVGCC_FeedForwardUpdate(DPC_AVGCC_FeedForward_t* pFF, DPC_AVGCC_Avg3Channels_t* pAvg);

void DPC_AVGCC_AvgMulControlLutSel(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LUT_TTPPFC_t* pLUT,
                                 DPC_LPCNTRL_ConverterControl_t* pConvCtrl,
                                 DPC_CMNDAT_PFC_ControlData_t* pCtrlData,
                                 int32_t PeakCurrentRef);

void DPC_AVGCC_AvgMulControlRunTime(DPC_AVGCC_Avg3Channels_t* pAvg);

void DPC_AVGCC_AvgControlInit(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LPCNTRL_VoltageControl_t* pVoltCtrl);

void DPC_AVGCC_AvgChannelsControl(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LUT_TTPPFC_t* pLUT, DPC_CMNDAT_PFC_RawData_t* pData, DPC_CMNDAT_DataSet1_t* pDataSet1, DPC_CMNDAT_DataSet2_t* pDataSet2, DPC_CMNDAT_PFC_ControlData_t* pCtrlData);
void DPC_AVGCC_AvgChannelsActuation(DPC_AVGCC_Avg3Channels_t* pAvg);
void DPC_AVGCC_AvgDrivingActuation(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LPCNTRL_PhaseShedding_t* pPhSh, DPC_CMNDAT_PFC_ControlData_t* pCtrlData, DPC_LUT_TTPPFC_t* pLUT);
void DPC_AVGCC_AvgDrivingActuationInit(DPC_AVGCC_Avg3Channels_t* pAvg, FlagStatus SingleCycleTest);

void DPC_AVGCC_MulIndexControl(DPC_AVGCC_Avg3Channels_t* pAvg);
void DPC_AVGCC_ZvdEnableControl(DPC_AVGCC_Avg3Channels_t* pAvg);
void DPC_AVGCC_ZvdProtectionControl(DPC_AVGCC_Avg3Channels_t* pAvg, FlagStatus ZvdControlMode);
void DPC_AVGCC_ZvdFollowerControl(DPC_AVGCC_Avg3Channels_t* pAvg, FlagStatus ZvdFollowerMode);

int32_t DPC_AVGCC_InputVoltageFeedForward(DPC_AVGCC_InputFeedForward_t* pInputFF, uint16_t InputVoltage, int32_t PeakCurrentRef);
void DPC_AVGCC_InputVoltageFeedForwardInit(DPC_AVGCC_InputFeedForward_t* pInputFF,
                                            uint16_t ActualInputVoltage,
                                            uint16_t NominalInputVoltage,
                                            uint16_t MinInputVoltage,
                                            uint16_t MaxInputVoltage);

void DPC_AVGCC_CurrentSenseCalibration(DPC_AVGCC_Avg3Channels_t* pAvg,
                                     uint16_t InductorAvgCurrentCh1,
                                     uint16_t InductorAvgCurrentCh2,
                                     uint16_t InductorAvgCurrentCh3,
                                     uint16_t InductorAvgCurrentTotal,
                                     uint16_t OutputCurrentDC);

void DPC_AVGCC_AvgCurrentControlActivation(DPC_AVGCC_Avg3Channels_t* pAvg,
                                         DPC_LUT_TTPPFC_t* pLUT,
                                         DPC_LPCNTRL_PhaseShedding_t* pPhSh,
                                         DPC_CMNDAT_PFC_RawData_t* pData,
                                         DPC_CMNDAT_DataSet1_t* pDataSet1,
                                         DPC_CMNDAT_DataSet2_t* pDataSet2,
                                         DPC_CMNDAT_PFC_ControlData_t* pCtrlData,
                                         DPC_LPCNTRL_ConverterControl_t* pConvCtrl,
                                         int32_t PeakCurrentRef);
void DPC_AVGCC_AvgCurrentControlSynch(DPC_AVGCC_Avg3Channels_t* pAvg);

void DPC_AVGCC_SoftDutyInit(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LUT_SoftDuty_t* pDuty);
uint32_t DPC_AVGCC_SoftDutySel(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LUT_SoftDuty_t* pDuty, FlagStatus DutyPolarityType);
void DPC_AVGCC_SoftDutyIndexReset(DPC_LUT_SoftDuty_t* pDuty);
uint32_t DPC_AVGCC_DutyCalcForBoostCCM(DPC_CMNDAT_PFC_ControlData_t* pCtrlData, DPC_AVGCC_Avg3Channels_t* pAvg, DPC_LUT_TTPPFC_t* pLUT);

//*** HYSTERESIS CURRENT CTRL of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/

#endif