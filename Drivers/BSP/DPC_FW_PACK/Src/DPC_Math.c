/**
  ******************************************************************************
  * @file           : Transform.c
  * @brief          : Data Collectiona and access interfaces
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
#include "DPC_Math.h"

#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

/* Private define ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/

/**
* @brief  Average
* @param  InputData, Input Value
* @param  pAvg, pointer to moving average struct
*
* @retval Average output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
uint16_t DCP_MTH_Average(uint16_t InputData, DPC_MTH_Average_t* pAvg)
{
   
   pAvg->uwAccVal = pAvg->uwAccVal - pAvg->uhAvgVal;
   pAvg->uwAccVal = pAvg->uwAccVal + InputData;
   pAvg->uhAvgVal = pAvg->uwAccVal/pAvg->uhWeight;
   
   return pAvg->uhAvgVal;
    
}


/**
* @brief  Average - 2 stages (cascaded)
* @param  InputData, Input Value
* @param  pAvg, pointer to moving average struct - 2 stages (cascaded)
*
* @retval Average output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
uint16_t DCP_MTH_Average2Stages(uint16_t InputData, DPC_MTH_Average2Stages_t* pAvg)
{
   //stage 1
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal - pAvg->stage_1.uhAvgVal;
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal + InputData;
   pAvg->stage_1.uhAvgVal = pAvg->stage_1.uwAccVal/pAvg->stage_1.uhWeight;
   //stage 2
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal - pAvg->stage_2.uhAvgVal;
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal + pAvg->stage_1.uhAvgVal;
   pAvg->stage_2.uhAvgVal = pAvg->stage_2.uwAccVal/pAvg->stage_2.uhWeight;

   return pAvg->stage_2.uhAvgVal;   
}

/**
* @brief  Average - 3 stages (cascaded)
* @param  InputData, Input Value
* @param  pAvg, pointer to moving average struct - 3 stages (cascaded)
*
* @retval Average output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
uint16_t DCP_MTH_Average3Stages(uint16_t InputData, DPC_MTH_Average3Stages_t* pAvg)
{
   //stage 1
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal - pAvg->stage_1.uhAvgVal;
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal + InputData;
   pAvg->stage_1.uhAvgVal = pAvg->stage_1.uwAccVal/pAvg->stage_1.uhWeight;
   //stage 2
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal - pAvg->stage_2.uhAvgVal;
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal + pAvg->stage_1.uhAvgVal;
   pAvg->stage_2.uhAvgVal = pAvg->stage_2.uwAccVal/pAvg->stage_2.uhWeight;
   //stage 3
   pAvg->stage_3.uwAccVal = pAvg->stage_3.uwAccVal - pAvg->stage_3.uhAvgVal;
   pAvg->stage_3.uwAccVal = pAvg->stage_3.uwAccVal + pAvg->stage_2.uhAvgVal;
   pAvg->stage_3.uhAvgVal = pAvg->stage_3.uwAccVal/pAvg->stage_3.uhWeight;
    
   return pAvg->stage_3.uhAvgVal;   
}


/**
* @brief  Average - 4 stages (cascaded)
* @param  InputData, Input Value
* @param  pAvg, pointer to moving average struct - 4 stages (cascaded)
*
* @retval Average output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
uint16_t DCP_MTH_Average4Stages(uint16_t InputData, DPC_MTH_Average4Stages_t* pAvg)
{
   //stage 1
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal - pAvg->stage_1.uhAvgVal;
   pAvg->stage_1.uwAccVal = pAvg->stage_1.uwAccVal + InputData;
   pAvg->stage_1.uhAvgVal = pAvg->stage_1.uwAccVal/pAvg->stage_1.uhWeight;
   //stage 2
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal - pAvg->stage_2.uhAvgVal;
   pAvg->stage_2.uwAccVal = pAvg->stage_2.uwAccVal + pAvg->stage_1.uhAvgVal;
   pAvg->stage_2.uhAvgVal = pAvg->stage_2.uwAccVal/pAvg->stage_2.uhWeight;
   //stage 3
   pAvg->stage_3.uwAccVal = pAvg->stage_3.uwAccVal - pAvg->stage_3.uhAvgVal;
   pAvg->stage_3.uwAccVal = pAvg->stage_3.uwAccVal + pAvg->stage_2.uhAvgVal;
   pAvg->stage_3.uhAvgVal = pAvg->stage_3.uwAccVal/pAvg->stage_3.uhWeight;
   //stage 4
   pAvg->stage_4.uwAccVal = pAvg->stage_4.uwAccVal - pAvg->stage_4.uhAvgVal;
   pAvg->stage_4.uwAccVal = pAvg->stage_4.uwAccVal + pAvg->stage_3.uhAvgVal;
   pAvg->stage_4.uhAvgVal = pAvg->stage_4.uwAccVal/pAvg->stage_4.uhWeight;
   
   return pAvg->stage_4.uhAvgVal;   
}



/**
* @brief  Moving Average
* @param  InputData, Input Value
* @param  pMovAvg, pointer to moving average struct
*
* @retval Average output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
uint16_t DCP_MTH_MovingAverage(uint16_t InputData, DPC_MTH_MovingAverage_t* pMovAvg)
{
  static uint8_t i=0; 

    pMovAvg->uwSumVal = (pMovAvg->uwSumVal - pMovAvg->uhaAvg[i]) + InputData;
    
    pMovAvg->uhMovAvgVal = pMovAvg->uwSumVal / pMovAvg->uhNumSamples;
    
    pMovAvg->uhaAvg[i]  = InputData;
    
    i++;
    if(i == pMovAvg->uhNumSamples)
    {
      i = 0;
    }

    return  pMovAvg->uhMovAvgVal;

}


/**
* @brief  Ramp Initialization function
* @param  Init_Output, Initial Ramp output value
* @param  Final_Output, Final Ramp output value
* @param  Ramp_Duration, Duration of the Ramp expressed in number of
*         execution cycles.
*         The duration in seconds can be calculated by multipling 
*         the Ramp_Duration by the duration [s] of one execution cycle. 
* @param  pRamp, pointer to Ramp struct
*
* @retval ErrorRetVal, ErrorStatus.
*
* @note Function valid for STM32G4xx microconroller family  
*/
ErrorStatus DCP_MTH_RampInit(int32_t Init_Output, int32_t Final_Output, uint32_t Ramp_Duration, DPC_MTH_RampGenerator_t* pRamp)
{
  ErrorStatus ErrorRetVal = ERROR;
  
  pRamp->wOutput = Init_Output;
  pRamp->wInitVal = Init_Output;
  pRamp->wFinalVal = Final_Output;
  pRamp->uwDuration = Ramp_Duration;
  pRamp->ubRampShiftExp = 16;
  pRamp->wAccumulator = pRamp->wInitVal << pRamp->ubRampShiftExp;
  pRamp->EndOfRamp = RESET;  
  pRamp->wStep = ((pRamp->wFinalVal - pRamp->wInitVal) << pRamp->ubRampShiftExp) / (int32_t)pRamp->uwDuration;
  
  // Ramp Initialization error check BEGIN //
  if (pRamp->wStep ==! 0){
  ErrorRetVal = SUCCESS;
  }
  // Ramp Initialization error check END //
  
  return ErrorRetVal;
  
}


/**
* @brief  Ramp Generation function
* @param  None
* @param  pRamp, pointer to Ramp struct
*
* @retval Ramp output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
int32_t DCP_MTH_RampGeneration(DPC_MTH_RampGenerator_t* pRamp)
{
  pRamp->wAccumulator = pRamp->wAccumulator + pRamp->wStep;
  pRamp->wOutput = pRamp->wAccumulator >> pRamp->ubRampShiftExp;
  
  if (((pRamp->wStep > 0) && (pRamp->wOutput > pRamp->wFinalVal)) || 
      ((pRamp->wStep < 0) && (pRamp->wOutput < pRamp->wFinalVal))){
  pRamp->wOutput = pRamp->wFinalVal;
  pRamp->EndOfRamp = SET;  
  }
  
  return pRamp->wOutput;
}


/**
* @brief  Exponentiation Initialization function
* @param  Init_Output, Initial Exp output value
* @param  Final_Output, Final Exp output value
* @param  Exp_Duration, Duration of the Exp expressed in number of
*         execution cycles.
*         The duration in seconds can be calculated by multipling 
*         the Exp_Duration by the duration [s] of one execution cycle. 
* @param  pExp, pointer to Exp struct
*
* @retval ErrorRetVal, ErrorStatus.
*
* @note Function valid for STM32G4xx microconroller family  
*/
ErrorStatus DCP_MTH_ExpFuncInit(int32_t Init_Output, int32_t Final_Output, uint32_t Exp_Duration, int32_t Exp_Of_Exp, DPC_MTH_ExpGenerator_t* pExp)
{
  ErrorStatus ErrorRetVal = ERROR;
  
  pExp->wOutput = Init_Output;
  pExp->wInitVal = Init_Output;
  pExp->wFinalVal = Final_Output;
  pExp->uwDuration = Exp_Duration;
  
  pExp->uwStepIndex = 0;
  pExp->Exponent = (double)pExp->uwStepIndex/(double)pExp->uwDuration;
  pExp->ExponentOfExponent = Exp_Of_Exp;
  
  if (pExp->wFinalVal > pExp->wInitVal){
    pExp->Base = pExp->wFinalVal;
    pExp->wOutput = (int32_t)pow((double)pExp->Base, pow((double)pExp->Exponent,(double)pExp->ExponentOfExponent)) + pExp->wInitVal - 1; //Init Out for exp=0
  }
  else if (pExp->wFinalVal < pExp->wInitVal){
    pExp->Base = pExp->wInitVal;
    pExp->wOutput = pExp->wInitVal - (int32_t)(pow((double)pExp->Base, pow((double)pExp->Exponent,(double)pExp->ExponentOfExponent))) + pExp->wFinalVal; //Init Out for exp=0
  }
  else {
    pExp->Base = pExp->wInitVal;
    pExp->wOutput = pExp->wInitVal;
  }
  
//  pExp->ubExpShiftExp = 16;
//  pExp->wAccumulator = pExp->wInitVal << pExp->ubExpShiftExp;
  pExp->EndOfExp = RESET;  
//  pExp->wStep = ((pExp->wFinalVal - pExp->wInitVal) << pExp->ubExpShiftExp) / (int32_t)pExp->uwDuration;
  
//  // Exp Initialization error check BEGIN //
//  if (pExp->wStep ==! 0){
  ErrorRetVal = SUCCESS;
//  }
//  // Exp Initialization error check END //
  
  return ErrorRetVal;
  
}


/**
* @brief  Exponentiation Generation function
* @param  None
* @param  pExp, pointer to Exp struct
*
* @retval Exp output value.
*
* @note Function valid for STM32G4xx microconroller family  
*/
int32_t DCP_MTH_ExpFuncGeneration(DPC_MTH_ExpGenerator_t* pExp)
{
//  pExp->wAccumulator = pExp->wAccumulator + pExp->wStep;
  
  
   if (pExp->wFinalVal > pExp->wInitVal){
    pExp->Base = pExp->wFinalVal;
    pExp->wOutput = (int32_t)pow((double)pExp->Base, pow((double)pExp->Exponent,(double)pExp->ExponentOfExponent)) + pExp->wInitVal - 1;
    if(pExp->wOutput > pExp->wFinalVal) pExp->wOutput = pExp->wFinalVal;
  }
  else if (pExp->wFinalVal < pExp->wInitVal){
    pExp->Base = pExp->wInitVal;
    pExp->wOutput = pExp->wInitVal - (int32_t)(pow((double)pExp->Base, pow((double)pExp->Exponent,(double)pExp->ExponentOfExponent))) + pExp->wFinalVal;
    if(pExp->wOutput < pExp->wFinalVal) pExp->wOutput = pExp->wFinalVal;
  }
  else {
    pExp->Base = pExp->wInitVal;
    pExp->wOutput = pExp->wInitVal;
  }
   
  if (pExp->uwStepIndex <= pExp->uwDuration){
  pExp->uwStepIndex++;
  pExp->Exponent = (double)pExp->uwStepIndex/(double)pExp->uwDuration;
  }
  else{
  
//  if (((pExp->wStep > 0) && (pExp->wOutput > pExp->wFinalVal)) || 
//      ((pExp->wStep < 0) && (pExp->wOutput < pExp->wFinalVal))){
//  pExp->wOutput = pExp->wFinalVal;
  pExp->EndOfExp = SET;  
//  }
  }
  return pExp->wOutput;
}



ErrorStatus DCP_MTH_SlopeLimiterInit(DCP_MTH_SlopeLimiter_t* pSlopeLim){
  
  ErrorStatus ErrorRetVal = ERROR;
  
  pSlopeLim->wPreviousValue = 0;
  pSlopeLim->wOutputValue = 0;
  pSlopeLim->wSlopeMax = (int32_t)(roundf((float)DPC_SLOPE_MAX));
  pSlopeLim->wSlopeMin = -(int32_t)(roundf((float)DPC_SLOPE_MAX));
  
  if((pSlopeLim->wSlopeMax != 0) && (pSlopeLim->wSlopeMin != 0)) ErrorRetVal = SUCCESS;
  
  return ErrorRetVal;

}

int32_t DCP_MTH_SlopeLimiter(int32_t Input, DCP_MTH_SlopeLimiter_t* pSlopeLim){
 
  pSlopeLim->wOutputValue = Input;
  
  if((Input - pSlopeLim->wPreviousValue) > pSlopeLim->wSlopeMax){
//    TEST_1_TOGGLE;
    pSlopeLim->wOutputValue = (pSlopeLim->wPreviousValue + pSlopeLim->wSlopeMax);
//    TEST_1_OFF;
//    pSlopeLim->wPreviousValue = Input;
  }
  else if((Input - pSlopeLim->wPreviousValue) < pSlopeLim->wSlopeMin){
//    TEST_2_TOGGLE;
    pSlopeLim->wOutputValue = (pSlopeLim->wPreviousValue + pSlopeLim->wSlopeMin);
//    TEST_1_OFF;
//    pSlopeLim->wPreviousValue = Input;
  } 
  
  
//  pSlopeLim->wPreviousValue = Input;
  pSlopeLim->wPreviousValue = pSlopeLim->wOutputValue;
  
  return pSlopeLim->wOutputValue;
  
}


void DCP_MTH_SlopeLimiterReset(DCP_MTH_SlopeLimiter_t* pSlopeLim){
  
  pSlopeLim->wPreviousValue = 0;

}








//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/

