/**
  ******************************************************************************
  * @file    DPC_Math.h
  * @brief   This file contains the headers of the transform module.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#ifndef __FASTMATH_H
#define __FASTMATH_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "DPC_CommonData.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */   
#endif


//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/

uint16_t DCP_MTH_Average(uint16_t InputData, DPC_MTH_Average_t* pAvg);
uint16_t DCP_MTH_Average2Stages(uint16_t InputData, DPC_MTH_Average2Stages_t* pAvg);
uint16_t DCP_MTH_Average3Stages(uint16_t InputData, DPC_MTH_Average3Stages_t* pAvg);
uint16_t DCP_MTH_Average4Stages(uint16_t InputData, DPC_MTH_Average4Stages_t* pAvg);

uint16_t DCP_MTH_MovingAverage(uint16_t InputData, DPC_MTH_MovingAverage_t* pMovAvg);

ErrorStatus DCP_MTH_RampInit(int32_t Init_Output, int32_t Final_Output, uint32_t Ramp_Duration, DPC_MTH_RampGenerator_t* pRamp);
int32_t DCP_MTH_RampGeneration(DPC_MTH_RampGenerator_t* pRamp);

ErrorStatus DCP_MTH_ExpFuncInit(int32_t Init_Output, int32_t Final_Output, uint32_t Exp_Duration, int32_t Exp_Of_Exp, DPC_MTH_ExpGenerator_t* pExp);
int32_t DCP_MTH_ExpFuncGeneration(DPC_MTH_ExpGenerator_t* pExp);
int32_t DCP_MTH_SlopeLimiter(int32_t Input, DCP_MTH_SlopeLimiter_t* pSlopeLim);
ErrorStatus DCP_MTH_SlopeLimiterInit(DCP_MTH_SlopeLimiter_t* pSlopeLim);
void DCP_MTH_SlopeLimiterReset(DCP_MTH_SlopeLimiter_t* pSlopeLim);


//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/

