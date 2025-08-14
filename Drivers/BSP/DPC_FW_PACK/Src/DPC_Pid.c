/**
******************************************************************************
* @file           : PID.c
* @brief          : PI Module
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
#include "DPC_Pid.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/

/**
* @brief  DPC_PI_s32_Init: Init PI Data Struct.
* @param  pPI: pointer to a PI_STRUCT_t  that contains
*         the configuration information and data for the specified PI. 
*
* @param  Init_Val_Kp: Init Kp data
* @param  Init_Val_Ki: Init Ki data
* @param  Init_PIsat_up: Init PI saturation Up data
* @param  Init_PIsat_down: Init PI saturation Down data
* @param  Init_Integral_sat_up: Init PI saturation Down data
* @param  Init_Integral_sat_down: Init PI saturation Down data
* @param  satPI_toggle_local: Init PI saturation Down data
* @param  antiwindPI_toggle_local: Init PI saturation Down data
* @param  Antiwindup_Gain_local: Init PI saturation Down data
* @param  resetValue: Init PI saturation Down data
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/  
void DPC_PIs32Init(DPC_PID_PIs32_t *pPI, uint32_t Init_Val_Kp, uint32_t Init_Val_Ki,
                   int32_t Init_PIsat_up, int32_t Init_PIsat_down,
                   int32_t Init_Integral_sat_up, int32_t Init_Integral_sat_down,
                   FlagStatus satPI_toggle_local,FlagStatus antiwindPI_toggle_local,
                   int32_t Antiwindup_Gain_local, int32_t resetValue)
{
  pPI->wIntegral = 0;
  pPI->wOut = 0;
  pPI->wError = 0;
  pPI->wIntegralOut = 0;
  pPI->resetPI = RESET;
  pPI->uwK0 = Init_Val_Kp; //K0=Kp
  pPI->uwK1 = Init_Val_Ki; //*Init_Val_Ts; //K1=Ki*Ts
  pPI->satPI_toggle = satPI_toggle_local;
  pPI->antiwindPI_toggle = antiwindPI_toggle_local;
  pPI->wAntiwindupGain = Antiwindup_Gain_local;
  pPI->wResetValue = resetValue;
  pPI->ubPI_Q_FxP = DPC_PI_EXP; //exponential 2^(ubPI_Q_FxP) PI parameters multiplier factor
                    //to allows integer calculation without losing resolution.
  pPI->wOutUpperSat=(int32_t)(Init_PIsat_up << pPI->ubPI_Q_FxP);
  pPI->wOutLowerSat=(int32_t)(Init_PIsat_down << pPI->ubPI_Q_FxP);
  pPI->wIntegralUpperSat=(int32_t)(Init_Integral_sat_up << pPI->ubPI_Q_FxP);
  pPI->wIntegralLowerSat=(int32_t)(Init_Integral_sat_down << pPI->ubPI_Q_FxP);  
  
}


/**
* @brief  DPC_PIs32Type0: PI function without anti-windup
* @param  Ref_s32: reference value
* @param  Feed_s32: feedback value
* @param  pPI: pointer to a PI_s32_t  that contains
*         the configuration information and data for the specified PI. 
*
* @retval wOutSat: Return output data of PI regulator
*
* @note Function valid for STM32G4xx microconroller family  
*/
int32_t DPC_PIs32Type0(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI)
{
  pPI->wRef=Ref_s32;
  pPI->wFeed=Feed_s32;
  
  pPI->wError=Ref_s32-Feed_s32;
  
  if(pPI->resetPI==SET)
  {
    pPI->wIntegral=pPI->wResetValue;
  }
  else
  {
    pPI->wIntegral=pPI->wIntegral + (pPI->uwK1*pPI->wError);
  }
  pPI->wIntegralOut=pPI->wIntegral;
  pPI->wOut = (pPI->uwK0*pPI->wError) + pPI->wIntegralOut;  
  
  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->wOut>pPI->wOutUpperSat)
    {
      pPI->wOutSat=pPI->wOutUpperSat;
    }
    else if(    pPI->wOut<pPI->wOutLowerSat)
    {
      pPI->wOutSat=pPI->wOutLowerSat;
    }
    else {
      pPI->wOutSat=pPI->wOut;
    }   
  }
  else {
    pPI->wOutSat=pPI->wOut;
  }
  //End Check Saturation
  
  pPI->wOutSat=pPI->wOutSat >> pPI->ubPI_Q_FxP; //scaling PI output 
  
  return pPI->wOutSat;  
}



/**
* @brief  DPC_PIs32Type1: PI function with anti-windup (clamping method)
* @param  Ref_s32: reference value
* @param  Feed_s32: feedback value
* @param  pPI: pointer to a DPC_PID_PIs32_t that contains
*         the configuration information and data for the specified PI. 
*
* @retval wOutSat: Return output data of PI regulator
*
* @note Function valid for STM32G4xx microconroller family  
*/
int32_t DPC_PIs32Type1(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI)
{
  pPI->wRef=Ref_s32;
  pPI->wFeed=Feed_s32;
  
  pPI->wError=Ref_s32-Feed_s32;
  
  if(pPI->resetPI==SET)
  {
    pPI->wIntegral=pPI->wResetValue;
  }
  else
  {
    pPI->wIntegralOut = pPI->wIntegral + (pPI->uwK1*pPI->wError);
  }
    //Start Check Antiwindup
  if (pPI->antiwindPI_toggle == SET){
    //Saturation
   if (pPI->wIntegralOut > pPI->wIntegralUpperSat) {     
      pPI->wIntegralOut = pPI->wIntegralUpperSat;
   } 
   else {
      if (pPI->wIntegralOut < pPI->wIntegralLowerSat) {
         pPI->wIntegralOut = pPI->wIntegralLowerSat;
      } else {
         pPI->wIntegral = pPI->wIntegralOut;
      }
   }
  }
  else {
    pPI->wIntegral = pPI->wIntegralOut;
  }
  //End Check Antiwindup 
  
  pPI->wOut=(pPI->uwK0*pPI->wError) + pPI->wIntegralOut;  
  
  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(pPI->wOut > pPI->wOutUpperSat)
    {
      pPI->wOutSat = pPI->wOutUpperSat;
    }
    else if(pPI->wOut < pPI->wOutLowerSat)
    {
      pPI->wOutSat = pPI->wOutLowerSat;
    }
    else {
      pPI->wOutSat = pPI->wOut;
    }   
  }
  else {
    pPI->wOutSat = pPI->wOut;  
  }
  //End Check Saturation
  
  pPI->wOutSat = pPI->wOutSat >> pPI->ubPI_Q_FxP; //scaling PI output 
  
  return pPI->wOutSat;  
}


/**
* @brief  DPC_PIs32Type2: PI function with anti-windup (back-calculation method)
* @param  Ref_s32: reference value
* @param  Feed_s32: feedback value
* @param  pPI: pointer to a DPC_PID_PIs32_t that contains
*         the configuration information and data for the specified PI. 
*
* @retval wOutSat: Return output data of PI regulator
*
* @note Function valid for STM32G4xx microconroller family  
*/
int32_t DPC_PIs32Type2(int32_t Ref_s32, int32_t Feed_s32 , DPC_PID_PIs32_t *pPI)
{
  pPI->wRef=Ref_s32;
  pPI->wFeed=Feed_s32;
  
  pPI->wError=Ref_s32-Feed_s32;
  
  if(pPI->resetPI==SET)
  {
    pPI->wIntegral=pPI->wResetValue;
  }
  else
  {
    pPI->wIntegral=pPI->wIntegral+(pPI->uwK1*pPI->wError)+pPI->wAntiwindupTerm;
  }
  pPI->wIntegralOut=pPI->wIntegral;
  pPI->wOut=(pPI->uwK0*pPI->wError)+pPI->wIntegralOut;  
  
  //Start Check Saturation
  if (pPI->satPI_toggle==SET){
    //Saturation
    if(    pPI->wOut>pPI->wOutUpperSat)
    {
      pPI->wOutSat=pPI->wOutUpperSat;
    }
    else if(    pPI->wOut < pPI->wOutLowerSat)
    {
      pPI->wOutSat=pPI->wOutLowerSat;
    }
    else {
      pPI->wOutSat=pPI->wOut;
    }
    
    //Start Check Antiwindup
    if (pPI->antiwindPI_toggle==SET){
      //Saturation
      pPI->wAntiwindupTerm=(pPI->wOutSat-pPI->wOut)*pPI->wAntiwindupGain;
    }
    else {
      pPI->wAntiwindupTerm=0;
    }
    //End Check Antiwindup    
  }
  else {
    pPI->wOutSat=pPI->wOut;  
    pPI->wAntiwindupTerm=0;
  }
  //End Check Saturation
  
  pPI->wOutSat=pPI->wOutSat>>pPI->ubPI_Q_FxP; //scaling PI output
  
  return pPI->wOutSat;  
}


/**
* @brief  DPC_PIs32Reset: Reset PIs32 value
* @param  pPI: pointer to a DPC_PID_PIs32_t that contains
*         the configuration information and data for the specified PI. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_PIs32Reset(DPC_PID_PIs32_t *pPI)
{
  pPI->wIntegral = pPI->wResetValue;
}


/**
* @brief  DPC_PIs32_LimitsUpdate: Update PIs32 limits value
* @param  pPI: pointer to a DPC_PID_PIs32_t that contains
*         the configuration information and data for the specified PI. 
* @param  pVctrl: pointer to a DPC_LPCNTRL_VoltageControl_t that contains
*         the configuration information and data for the specified Voltage Control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/ 
void DPC_PIs32_LimitsUpdate(DPC_PID_PIs32_t *pPI, DPC_LPCNTRL_VoltageControl_t *pVctrl)
{
   pPI->wIntegralUpperSat =  pVctrl->wIpkMax << pPI->ubPI_Q_FxP;
   pPI->wIntegralLowerSat = -(pVctrl->wIpkMax) << pPI->ubPI_Q_FxP;
   pPI->wOutUpperSat = pVctrl->wIpkMax << pPI->ubPI_Q_FxP;
   pPI->wOutLowerSat = -(pVctrl->wIpkMax) << pPI->ubPI_Q_FxP;
}






//*** MATH of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/