/**
  ******************************************************************************
  * @file    Pid.h
  * @brief   This file contains the headers of the Pid Module.
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
#ifndef __PID_H
#define __PID_H


/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "DPC_CommonData.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/
void DPC_PIs32Init(DPC_PID_PIs32_t *pPI, uint32_t Init_Val_Kp, uint32_t Init_Val_Ki,
                   int32_t Init_PIsat_up, int32_t Init_PIsat_down,
                   int32_t Init_Integral_sat_up, int32_t Init_Integral_sat_down,
                   FlagStatus satPI_toggle_local, FlagStatus antiwindPI_toggle_local,
                   int32_t Antiwindup_Gain_local, int32_t resetValue);

int32_t DPC_PIs32Type0(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI);
int32_t DPC_PIs32Type1(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI);
int32_t DPC_PIs32Type2(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI);

void DPC_PIs32Reset(DPC_PID_PIs32_t *pPI);
void DPC_PIs32_LimitsUpdate(DPC_PID_PIs32_t *pPI, DPC_LPCNTRL_VoltageControl_t *pVctrl);
void DPC_PIs32_ParametersUpdate(DPC_PID_PIs32_t *pPI, DPC_LPCNTRL_VoltageControl_t *pVctrl);
//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/

 
#endif