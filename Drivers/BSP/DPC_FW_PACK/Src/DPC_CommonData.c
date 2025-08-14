/**
  ******************************************************************************
  * @file           : Data.c
  * @brief          : Data Collection and access interfaces
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
/* Includes ------------------------------------------------------------------*/
#include "DPC_CommonData.h"
    
/* Private variables ---------------------------------------------------------*/
 
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


//*** COMMON DATA of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/

void DPC_CMNDAT_GetDataAllSet(uint16_t* pDataSet1, uint16_t* pDataSet2,uint16_t* pDataSet3, DPC_CMNDAT_PFC_RawData_t* pDataAllSet, FunctionalState GetDataMode)                            
{
  if (!GetDataMode){
   //*** ADC mode BEGIN***//
//   pDataAllSet->uhIL1   = (uint16_t) pDataSet1[IL1_RANK_ID]; //average Inductor current of channel 1
//   pDataAllSet->uhIL2   = (uint16_t) pDataSet1[IL2_RANK_ID]; //average Inductor current of channel 2
//   pDataAllSet->uhIL3   = (uint16_t) pDataSet1[IL3_RANK_ID]; //average Inductor current of channel 3
   pDataAllSet->uhVout  = (uint16_t) pDataSet1[VOUT_RANK_ID]; //PFC output voltage
//   pDataAllSet->uhIout  = (uint16_t) pDataSet1[IOUT_RANK_ID]; //PFC output current
   pDataAllSet->uhTemp  = (uint16_t) pDataSet1[TEMP_RANK_ID]; //PFC ambient temperature
   pDataAllSet->uhVinL1 = (uint16_t) pDataSet1[VIN_L1_RANK_ID]; //PFC Input voltage L1 (line)
   pDataAllSet->uhVinL2 = (uint16_t) pDataSet1[VIN_L2_RANK_ID]; //PFC Input voltage L2 (neutral)
   pDataAllSet->uhVinPreSwitch = (uint16_t) pDataSet3[0]; //PFC Input voltage pre-switching

//   pDataAllSet->uhIin    = (uint16_t) pDataSet2[I_IN_RANK_ID]; //PFC total inductor current (sum of all channels)
//   pDataAllSet->uhIinFlt = (uint16_t) pDataSet2[I_IN_FLT_RANK_ID]; //PFC total average inductor current (sum of all channels)
   //*** ADC mode END***//
  }
  else{ 
   //*** Development mode BEGIN***//
   pDataAllSet->uhIL1   = pDataAllSet->uhIL1Debug; //Development Value of average Inductor current of channel 1
   pDataAllSet->uhIL2   = pDataAllSet->uhIL2Debug; //Development Value of average Inductor current of channel 2
   pDataAllSet->uhIL3   = pDataAllSet->uhIL3Debug; //Development Value of average Inductor current of channel 3
   pDataAllSet->uhVout  = pDataAllSet->uhVoutDebug; //Development Value of PFC output voltage
   pDataAllSet->uhIout  = pDataAllSet->uhIoutDebug; //Development Value of PFC output current
   pDataAllSet->uhTemp  = pDataAllSet->uhTempDebug; //Development Value of PFC ambient temperature
   pDataAllSet->uhVinL1 = pDataAllSet->uhVinL1Debug; //Development Value of PFC Input voltage L1 (line)
   pDataAllSet->uhVinL2 = pDataAllSet->uhVinL2Debug; //Development Value of PFC Input voltage L2 (neutral)
   pDataAllSet->uhVinL2 = pDataAllSet->uhVinL2Debug;
   pDataAllSet->uhVinPreSwitch   = pDataAllSet->uhVinPreSwitchDebug; //Development Value of PFC Input voltage pre-switching
   pDataAllSet->uhIinFlt = pDataAllSet->uhIinFltDebug; //Development Value of PFC total average inductor current (sum of all channels)   
   //*** Development mode END***//
  }
     
}





//*** COMMON DATA of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/


