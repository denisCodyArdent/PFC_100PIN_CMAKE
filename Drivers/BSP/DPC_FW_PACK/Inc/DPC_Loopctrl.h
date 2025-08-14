/**
  ******************************************************************************
  * @file    Loopctrl.h
  * @brief   This file contains the headers of the Loop_Ctrl Module.
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
#ifndef __DPC_LOOPCTRL_H
#define __DPC_LOOPCTRL_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

#include "DPC_CommonData.h"

/* Exported types ------------------------------------------------------------*/

void DPC_LPCNTRL_HardwareFaultEnable(void);
void DPC_LPCNTRL_DrivingOutputStop(void);
void DPC_LPCNTRL_MulControlTimingMeasurementSet(void);
void DPC_LPCNTRL_MulControlTimingDefaultSet(void);
int16_t DPC_LPCNTRL_TemperatureGet(uint16_t SensorOutputValue);
void DPC_LPCNTRL_PhaseShedding(DPC_LPCNTRL_PhaseShedding_t* pPhSh, DPC_CMNDAT_PFC_ControlData_t* pCtrlData, DPC_LPCNTRL_VoltageControl_t* pVoltCtrl, DPC_LPCNTRL_ConverterControl_t* pConvCtrl);
void DPC_LPCNTRL_PhaseSheddingUpdate(DPC_LPCNTRL_PhaseShedding_t* pPhSh, uint16_t InputVoltage);
void DPC_LPCNTRL_PhaseSheddingControlInit(DPC_LPCNTRL_PhaseShedding_t* pPhSh);
void DPC_LPCNTRL_RelayControlInit(DPC_LPCNTRL_Inrush_t* pRelay);
void DPC_LPCNTRL_ProtectionControlInit(DPC_LPCNTRL_Protection_t* pProt);
void DPC_LPCNTRL_ProtectionControlCalibration(DPC_LPCNTRL_Protection_t* pProt, DPC_CMNDAT_PFC_ControlData_t* pCtrlData);
void DPC_LPCNTRL_ConverterControlInit(DPC_LPCNTRL_ConverterControl_t* pConvCtrl); 
void DPC_LPCNTRL_FanControlInit(DPC_LPCNTRL_Fan_t* pFan);
void DPC_LPCNTRL_VoltageControlInit(DPC_LPCNTRL_VoltageControl_t* pVoltCtrl);
void DPC_LPCNTRL_ConverterStatusUpdate(DPC_LPCNTRL_Led_t* pLed, DPC_LPCNTRL_Inrush_t* pRelay, DPC_LPCNTRL_Fan_t* pFan); 
void DPC_LPCNTRL_BusVoltageUpdate(DPC_CMNDAT_PFC_ControlData_t* pCtrlData,  DPC_MTH_RampGenerator_t* pRamp, DPC_LPCNTRL_VoltageControl_t* pVoltCtrl, DPC_LPCNTRL_ConverterControl_t* pConvCtrl);
void DPC_LPCNTRL_ACpeakCurrentUpdate(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_CMNDAT_PFC_ControlData_t* pCtrlData, DPC_LPCNTRL_ConverterControl_t* pConvCtrl);
void DPC_LPCNTRL_ACphaseCurrentUpdate(DPC_CMNDAT_PFC_ControlData_t* pCtrlData);

#endif /*__DPC_LOOPCTRL.H */