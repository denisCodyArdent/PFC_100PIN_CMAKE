/**
  ******************************************************************************
  * @file           : DPC_Loopctrl.c
  * @brief          : Loop Control  Module
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
#ifdef STM32G474xx
  #include "stm32g4xx_hal.h"
#endif

#include "hrtim.h"
#include "tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "DPC_Loopctrl.h"
#include "dac.h"
#include "DPC_Math.h"

/* Private variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  DPC_LPCNTRL_DrivingOutputStop: Stop the output driving signals. 
*
* @param  none 
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_DrivingOutputStop(void){
  
HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
LF_LS_OFF;
LF_HS_OFF;

}

/**
* @brief  DPC_LPCNTRL_HardwareFaultEnable: Enable the HRTIM hardware fault 
*
* @param  none 
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_HardwareFaultEnable(void){
  
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1 + HRTIM_FAULT_2 + HRTIM_FAULT_3 + HRTIM_FAULT_4 + HRTIM_FAULT_5 + HRTIM_FAULT_6, HRTIM_FAULTMODECTL_ENABLED);
 
}


/**
* @brief  DPC_LPCNTRL_MulControlTimingMeasurementSet: Set the MUL timer for the line frequency measurement
*
* @param  none 
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_MulControlTimingMeasurementSet(void){
        HAL_TIM_Base_Stop_IT(&htim3); //MUL timer stop
        LL_TIM_SetAutoReload(TIM3,200); //20us MUL timer autoreload precision (1=100ns) for freq. meas.
        HAL_TIM_Base_Start_IT(&htim3); //MUL timer start
}


/**
* @brief  DPC_LPCNTRL_MulControlTimingMeasurementSet: Set the MUL timer for the default operation
*
* @param  none 
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_MulControlTimingDefaultSet(void){
        HAL_TIM_Base_Stop_IT(&htim3); //MUL timer stop
        MX_TIM3_Init();               
        LL_TIM_SetAutoReload(TIM3,(uint16_t)((DPC_LUT_PERIOD_TIM_CNT * 200) / DPC_LUT_PERIOD_POINTS)); //MUL timer autoreload set according to freq. meas.
        HAL_TIM_Base_Start_IT(&htim3); //MUL timer start
}




/**
* @brief  DPC_LPCNTRL_TemperatureGet: gets the temperature for the STLM20 sensor.
* 
*         TEMPERATURE SENSOR CHARACTERISTIC (STLM20)
*         Second order parabolic eq V0= (�3.88 *10-6 T^2) + (�1.15 *10�2 *T) + 1.8639
*         solving for T: T= -1481.96 + sqrt(2.1962*10^6 + (1.8639-V0)/3.88*10^-6)
*         Table 2. First order equations (approx.) optimized for different temperature ranges
*           Temperature range           Linear equation VO =            Maximum deviation of linear equation from parabolic equation (�C)
*           T min (�C) Tmax (�C) 
*            �55        130             �11.79 mV/�C * T + 1.8528 V             �1.41 
*            �40        110             �11.77 mV/�C * T + 1.8577 V             �0.93 
*            �30        100             �11.77 mV/�C * T + 1.8605 V             �0.70 
*            �40         85             �11.67 mV/�C * T + 1.8583 V             �0.65 
*            �10         65             �11.71 mV/�C * T + 1.8641 V             �0.23 
*             35         45             �11.81 mV/�C * T + 1.8701 V             �0.004 
*             20         30             �11.69 mV/�C * T + 1.8663 V             �0.004*         
* @param  SensorOutputValue: ADC value coming from sensor output. 
* 
* @retval Temperature in �C
*
* @note Function valid for STM32G4xx microconroller family  
*/
int16_t DPC_LPCNTRL_TemperatureGet(uint16_t SensorOutputValue){

  int16_t Temperature;

  Temperature = (int16_t)(((((int32_t)SensorOutputValue*330000)/4095) - 186630) / -1169);
  
  return Temperature;
}




/**
* @brief  DPC_LPCNTRL_PhaseShedding: Phase shedding function for 3ch interleaved PFC.
*         It performs the phase shedding function with a sequential activation of the channels (eg. 1ch -> 2ch -> 3ch or 3ch -> 2ch -> 1ch). 
*
* @param  pPhSh: pointer to a phase shedding structure that contains
*         the configuration information and data for the specified control. 
* 
* @param  pCtrlData: pointer to the value to be compared with the threshods for
*         activation/disactivation of the channels. 
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_PhaseShedding(DPC_LPCNTRL_PhaseShedding_t* pPhSh,
                               DPC_CMNDAT_PFC_ControlData_t* pCtrlData,
                               DPC_LPCNTRL_VoltageControl_t* pVoltCtrl,
                               DPC_LPCNTRL_ConverterControl_t* pConvCtrl){
                                 
if((pConvCtrl->ConversionMode == DPC_PFC_MODE)||(pConvCtrl->ConversionMode == DPC_NONE_MODE)){
          if (pPhSh->ubNumOfActiveChannels == 3){
              if (pCtrlData->uwIload < (uint32_t)pPhSh->uhCurrentTh3to2_dc){ 
              pPhSh->ubNumOfActiveChannels = 2;
             }
            }  
          if (pPhSh->ubNumOfActiveChannels == 2){
              if (pCtrlData->uwIload < (uint32_t)pPhSh->uhCurrentTh2to1_dc){
              pPhSh->ubNumOfActiveChannels = 1;
             }
             
              if (pCtrlData->uwIload > (uint32_t)pPhSh->uhCurrentTh2to3_dc){
              pPhSh->ubNumOfActiveChannels = 3;
             }         
            }
          if (pPhSh->ubNumOfActiveChannels <= 1){
              if (pCtrlData->uwIload > (uint32_t)pPhSh->uhCurrentTh1to2_dc){
              pPhSh->ubNumOfActiveChannels = 2;
             }
            } 
   }//if((pConvCtrl->ConversionMode == DPC_PFC_MODE)||(pConvCtrl->ConversionMode == DPC_NONE_MODE))

else if(pConvCtrl->ConversionMode == DPC_INVERTER_MODE){
  
            if (pPhSh->ubNumOfActiveChannels == 3){
              if (pVoltCtrl->wIpk < (int32_t)pPhSh->uhCurrentTh3to2_ac){ 
              pPhSh->ubNumOfActiveChannels = 2;
             }
            }  
          if (pPhSh->ubNumOfActiveChannels == 2){
              if (pVoltCtrl->wIpk < (int32_t)pPhSh->uhCurrentTh2to1_ac){
              pPhSh->ubNumOfActiveChannels = 1;
             }
             
              if (pVoltCtrl->wIpk > (int32_t)pPhSh->uhCurrentTh2to3_ac){
              pPhSh->ubNumOfActiveChannels = 3;
             }         
            }
          if (pPhSh->ubNumOfActiveChannels <= 1){
              if (pVoltCtrl->wIpk > (int32_t)pPhSh->uhCurrentTh1to2_ac){
              pPhSh->ubNumOfActiveChannels = 2;
             }
            } 
  
   }//else if(pConvCtrl->ConversionMode == DPC_INVERTER_MODE)
 
}

/**
* @brief  DPC_LPCNTRL_PhaseSheddingUpdate: Phase shedding update function for 3ch interleaved PFC.
*         It performs the phase shedding parameters/thresholds update basing on the actual Input Voltage. 
*
* @param  pPhSh: pointer to a phase shedding structure that contains
*         the configuration information and data for the specified control. 
* 
* @param  InputVoltage: actual Input Voltage [expressed in Vrms]
* 
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_PhaseSheddingUpdate(DPC_LPCNTRL_PhaseShedding_t* pPhSh, uint16_t InputVoltage){
  
  //dc side
  pPhSh->uhCurrentTh1to2_dc = (pPhSh->uhCurrentTh1to2_dc_Init * InputVoltage) / DPC_VIN_NOM;	
  pPhSh->uhCurrentTh2to1_dc = (pPhSh->uhCurrentTh2to1_dc_Init * InputVoltage) / DPC_VIN_NOM;	
  pPhSh->uhCurrentTh2to3_dc = (pPhSh->uhCurrentTh2to3_dc_Init * InputVoltage) / DPC_VIN_NOM;	
  pPhSh->uhCurrentTh3to2_dc = (pPhSh->uhCurrentTh3to2_dc_Init * InputVoltage) / DPC_VIN_NOM;

}

/**
* @brief  DPC_LPCNTRL_PhaseSheddingControlInit: Phase shedding init function for 3ch interleaved PFC.
*         It performs the phase shedding parameters/thresholds init. 
*
* @param  pPhSh: pointer to a phase shedding structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_PhaseSheddingControlInit(DPC_LPCNTRL_PhaseShedding_t* pPhSh){
   
   pPhSh->ubNumOfActiveChannels = DPC_NCH;
   //dc side current thresholds init
   pPhSh->uhCurrentTh1to2_dc_Init = DPC_IDC_1TO2;	
   pPhSh->uhCurrentTh2to1_dc_Init = DPC_IDC_2TO1;	
   pPhSh->uhCurrentTh2to3_dc_Init = DPC_IDC_2TO3;	
   pPhSh->uhCurrentTh3to2_dc_Init = DPC_IDC_3TO2;   
   pPhSh->uhCurrentTh1to2_dc = pPhSh->uhCurrentTh1to2_dc_Init;	
   pPhSh->uhCurrentTh2to1_dc = pPhSh->uhCurrentTh2to1_dc_Init;	
   pPhSh->uhCurrentTh2to3_dc = pPhSh->uhCurrentTh2to3_dc_Init;	
   pPhSh->uhCurrentTh3to2_dc = pPhSh->uhCurrentTh3to2_dc_Init; 
   //ac side current thresholds init
   pPhSh->uhCurrentTh1to2_ac_Init = DPC_IAC_PK_1TO2;	
   pPhSh->uhCurrentTh2to1_ac_Init = DPC_IAC_PK_2TO1;	
   pPhSh->uhCurrentTh2to3_ac_Init = DPC_IAC_PK_2TO3;	
   pPhSh->uhCurrentTh3to2_ac_Init = DPC_IAC_PK_3TO2;

   pPhSh->uhCurrentTh1to2_ac = pPhSh->uhCurrentTh1to2_ac_Init;	
   pPhSh->uhCurrentTh2to1_ac = pPhSh->uhCurrentTh2to1_ac_Init;	
   pPhSh->uhCurrentTh2to3_ac = pPhSh->uhCurrentTh2to3_ac_Init;	
   pPhSh->uhCurrentTh3to2_ac = pPhSh->uhCurrentTh3to2_ac_Init; 
}

/**
* @brief  DPC_LPCNTRL_RelayControlInit: Relay control init function.
*         It performs the relay control init. 
*
* @param  pRelay: pointer to a relay control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_RelayControlInit(DPC_LPCNTRL_Inrush_t* pRelay){
   
   pRelay->RelayControl = DISABLE;
   pRelay->RelayInrushState = IDLE;

}
void DPC_LPCNTRL_MainsSwControlInit(DPC_LPCNTRL_Rly_t* pRelay){
  pRelay->RelayControl = DISABLE;
  pRelay->PastConversionMode = DPC_INVERTER_MODE;
  pRelay->PresentConversionMode = DPC_PFC_MODE;
  }
/**
* @brief  DPC_LPCNTRL_ProtectionControlInit: Protections control init function.
*         It performs the protections control init. 
*
* @param  pProt: pointer to a protection control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_ProtectionControlInit(DPC_LPCNTRL_Protection_t* pProt){
  
   if (DPC_OTP_EN == true){ pProt->OverTemperatureProtection = ENABLE; } else { pProt->OverTemperatureProtection = DISABLE;} // OTP enable control 
   if (DPC_OCP_EN == true){ pProt->OverCurrentProtection = ENABLE; } else { pProt->OverCurrentProtection = DISABLE;} // OCP enable control   
   pProt->OverCurrentTrigger = RESET;
   pProt->OCPEventCh1High = RESET;
   pProt->OCPEventCh1Low = RESET;
   pProt->OCPEventCh2High = RESET;
   pProt->OCPEventCh2Low = RESET;
   pProt->OCPEventCh3High = RESET;
   pProt->OCPEventCh3Low = RESET;  

   pProt->uwOCPHighLevelCh1 = (uint32_t)roundf((float)((DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f); // OCP threshold calculation   
   pProt->uwOCPLowLevelCh1 = (uint32_t)roundf((float)(-(DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f); // OCP threshold calculation   

   pProt->uwOCPHighLevelCh2 = (uint32_t)roundf((float)((DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f);//pProt->uwOCPHighLevelCh1;
   pProt->uwOCPLowLevelCh2 = (uint32_t)roundf((float)(-(DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f);//pProt->uwOCPLowLevelCh1;
   
   pProt->uwOCPHighLevelCh3 = (uint32_t)roundf((float)((DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f);//pProt->uwOCPHighLevelCh1;
   pProt->uwOCPLowLevelCh3 = (uint32_t)roundf((float)(-(DPC_IL_GAIN * DPC_OCP_VALUE) + DPC_IZERO_OFFSET_IL) * (float)1.0f);//pProt->uwOCPLowLevelCh1;
   
   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pProt->uwOCPHighLevelCh1); // OCP Ch1 high threshold update
   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pProt->uwOCPLowLevelCh1); // OCP Ch1 low threshold update
   
   HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pProt->uwOCPHighLevelCh2); // OCP Ch2 high threshold update 
   HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pProt->uwOCPLowLevelCh2); // OCP Ch2 low threshold update
   
   HAL_DAC_SetValue(&hdac4, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pProt->uwOCPHighLevelCh3); // OCP Ch3 high threshold update
   HAL_DAC_SetValue(&hdac4, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pProt->uwOCPLowLevelCh3); // OCP Ch3 low threshold update 
//   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4000); // OCP threshold update 
      
//   pProt->uwIthOCP = (uint32_t)((float)(DPC_OCP_SENSE_GAIN * DPC_OCP_VALUE - DPC_OCP_DIODE_VF) * ((powf(2.0,(float)DPC_ADC_DAC_NBIT)- 1.0f)/(float)DPC_VDD_MCU)); // OCP threshold calculation   
//   HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pProt->uwIthOCP); // OCP threshold update 
   
   
   pProt->ubErrorCode = 0; // Error code indicator

}

/**
* @brief  DPC_LPCNTRL_ProtectionControlCalibration: Protections control calibration function.
*         It performs the protections control init. 
*
* @param  pProt: pointer to a protection control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_ProtectionControlCalibration(DPC_LPCNTRL_Protection_t* pProt, DPC_CMNDAT_PFC_ControlData_t* pCtrlData){
  
//   pProt->uwIthOCP = (uint32_t)((uint32_t)pProt->uwIthOCP + (uint32_t)pCtrlData->uhAvgIL1 - (uint32_t)((float)DPC_IZERO_OFFSET * (powf(2,(float)DPC_ADC_DAC_NBIT)/(float)DPC_VDD_MCU)));
//   pProt->uwIthOCP = (pProt->uwIthOCP + 10);// + 2038 - 2048;   
//   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pProt->uwIthOCP); // OCP threshold update 

//   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4000); // OCP threshold update 
      
//   pProt->uwIthOCP = (uint32_t)((float)(DPC_OCP_SENSE_GAIN * DPC_OCP_VALUE - DPC_OCP_DIODE_VF) * ((powf(2.0,(float)DPC_ADC_DAC_NBIT)- 1.0f)/(float)DPC_VDD_MCU)); // OCP threshold calculation   
//   HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pProt->uwIthOCP); // OCP threshold update 
   


}



/**
* @brief  DPC_LPCNTRL_ConverterControlInit: Converter control init function.
*         It performs the converter control init. 
*
* @param  pConvCtrl: pointer to a converter control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_ConverterControlInit(DPC_LPCNTRL_ConverterControl_t* pConvCtrl){
  // this is forced at the moment. 
   
   pConvCtrl->ConversionMode = DPC_PFC_MODE; // check for conversion mode ac-dc/dc-ac
   
   if (pConvCtrl->ConversionMode == DPC_INVERTER_MODE) LED_BLUE_ON;
   pConvCtrl->Flag = SET;
   pConvCtrl->ubS = CONVERTER_INRUSH;
   pConvCtrl->ubRunState = 0;
   pConvCtrl->ConversionStart = DPC_MANUAL_START;
   pConvCtrl->VoutNewSetpoint_Volt = (float)DPC_VBUS_REF_DEFAULT;
   pConvCtrl->IacRefInverterNewValue_Ampere = (float)DPC_IPK_REF_INVERTER_DEFAULT;
   

}

/**
* @brief  DPC_LPCNTRL_FanControlInit: Fan control init function.
*         It performs the fan control init. 
*
* @param  pFan: pointer to a fan control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_FanControlInit(DPC_LPCNTRL_Fan_t* pFan){
   
   pFan->FanEnable = DPC_FAN_ENABLE;

}

/**
* @brief  DPC_LPCNTRL_VoltageControlInit: Voltage control init function.
*         It performs the converter voltage control init. 
*
* @param  pVoltCtrl: pointer to a voltage control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_VoltageControlInit(DPC_LPCNTRL_VoltageControl_t* pVoltCtrl){
   
   pVoltCtrl->VoltageControl = DISABLE;
   pVoltCtrl->BurstRising = IDLE;
   pVoltCtrl->SoftStartup = IDLE;
   pVoltCtrl->BusVoltageUpdate = IDLE;
   pVoltCtrl->uhIpkIoutFF = 0;
//   pVoltCtrl->uwIpkMax = (uint32_t)DPC_IPK_MAX;
   
   pVoltCtrl->wIpkMax = (int32_t)roundf((float)DPC_IPK_REF_MAX * (float)DPC_IL_TOT_GAIN * (float)1.0f);
   pVoltCtrl->uwIpkMin = (uint32_t)roundf((float)DPC_IPK_REF_MIN * (float)DPC_IL_TOT_GAIN * (float)1.0f);
   pVoltCtrl->uwIpkBurst = (uint32_t)roundf((float)DPC_IPK_REF_BURST * (float)DPC_IL_TOT_GAIN * (float)1.0f);   
   pVoltCtrl->LoadFFEnable = DPC_LOAD_FF_EN;
   pVoltCtrl->VinFFEnable = DPC_VINP_FF_EN;
  
}

/**
* @brief  DPC_LPCNTRL_ConverterStatusUpdate: Converter control status update function.
*         It performs the converter control status update. 
*
* @param  pLed: pointer to a led indicator control structure that contains
*         the configuration information and data for the specified control. 
*
* @param  pRelay: pointer to a relay control structure that contains
*         the configuration information and data for the specified control. 
*
* @param  pFan: pointer to a fan control structure that contains
*         the configuration information and data for the specified control. 
*
* @retval None
*
* @note Function valid for STM32G4xx microconroller family  
*/
void DPC_LPCNTRL_ConverterStatusUpdate(DPC_LPCNTRL_Led_t* pLed, DPC_LPCNTRL_Inrush_t* pRelay,DPC_LPCNTRL_Rly_t* pMainsSw, DPC_LPCNTRL_Fan_t* pFan){
   
   pLed->GreenLed = LED_GREEN_STATE;
   pLed->RedLed = LED_RED_STATE;
   pLed->YellowLed = LED_YELLOW_STATE;
   pLed->BlueLed = LED_BLUE_STATE;
   pFan->FanState1 = FAN_STATE;
   pRelay->Relay = RELAY_STATE;
   pMainsSw->Relay =MAINS_SW_STATE ;


}



void DPC_LPCNTRL_BusVoltageUpdate(DPC_CMNDAT_PFC_ControlData_t* pCtrlData,  DPC_MTH_RampGenerator_t* pRamp, DPC_LPCNTRL_VoltageControl_t* pVoltCtrl, DPC_LPCNTRL_ConverterControl_t* pConvCtrl){
   
    //New Vbus(dc) Volt value saturation
    if((float)pConvCtrl->VoutNewSetpoint_Volt > (float)DPC_VBUS_REF_MAX) pConvCtrl->VoutNewSetpoint_Volt = (float)DPC_VBUS_REF_MAX;
    if((float)pConvCtrl->VoutNewSetpoint_Volt < (float)DPC_VBUS_REF_MIN) pConvCtrl->VoutNewSetpoint_Volt = (float)DPC_VBUS_REF_MIN;
    //New Vbus(dc) Volt/DIG conversion
    pCtrlData->uhVoutNewSetpoint = (uint16_t)roundf((float)pConvCtrl->VoutNewSetpoint_Volt * (float)pCtrlData->VdcGain);
    
    if ( ((pCtrlData->uhVoutRef > pCtrlData->uhVoutNewSetpoint) || (pCtrlData->uhVoutRef < pCtrlData->uhVoutNewSetpoint)) && (pVoltCtrl->BusVoltageUpdate == IDLE)){
      
      if(DCP_MTH_RampInit((int32_t)pCtrlData->uhVoutRef, (int32_t)pCtrlData->uhVoutNewSetpoint, DPC_VBUS_UPDATE_DURATION, pRamp)){     
    //        eb1= 30; //error managment TBD
      } 
      pVoltCtrl->BusVoltageUpdate = RUNNING;
    }
    
    if (pVoltCtrl->BusVoltageUpdate == RUNNING){
          if ((pCtrlData->uhVoutRef < pCtrlData->uhVoutNewSetpoint) || (pCtrlData->uhVoutRef > pCtrlData->uhVoutNewSetpoint)) { // Vout increment 
            pCtrlData->uhVoutRef = (uint16_t)DCP_MTH_RampGeneration(pRamp);
            pCtrlData->uhVoutBurstMax = (uint16_t)(roundf((float)pCtrlData->uhVoutRef * (float)DPC_VBURST_UPDATE_COEFF_MAX));
            pCtrlData->uhVoutBurstMin = (uint16_t)(roundf((float)pCtrlData->uhVoutRef * (float)DPC_VBURST_UPDATE_COEFF_MIN));          
          }
          else {
            pCtrlData->uhVoutRef = pCtrlData->uhVoutNewSetpoint;
            pVoltCtrl->BusVoltageUpdate = COMPLETE;
            pVoltCtrl->BusVoltageUpdate = IDLE;
            }
    }  
  
}




void DPC_LPCNTRL_ACpeakCurrentUpdate(DPC_AVGCC_Avg3Channels_t* pAvg, DPC_CMNDAT_PFC_ControlData_t* pCtrlData, DPC_LPCNTRL_ConverterControl_t* pConvCtrl){
  
    //New Iac Ampere value saturation
    if((float)pConvCtrl->IacRefInverterNewValue_Ampere > (float)DPC_IPK_REF_INVERTER_MAX) pConvCtrl->IacRefInverterNewValue_Ampere = (float)DPC_IPK_REF_INVERTER_MAX;
    if((float)pConvCtrl->IacRefInverterNewValue_Ampere < (float)DPC_IPK_REF_INVERTER_MIN) pConvCtrl->IacRefInverterNewValue_Ampere = (float)DPC_IPK_REF_INVERTER_MIN;
    //New Iac A/DIG conversion
    pCtrlData->uhIacRefInverterNewValue = (uint16_t)roundf((float)pConvCtrl->IacRefInverterNewValue_Ampere * (float)pCtrlData->ILtotGain);
    
      if ( ((pCtrlData->uhIacRefInverter > pCtrlData->uhIacRefInverterNewValue) || (pCtrlData->uhIacRefInverter < pCtrlData->uhIacRefInverterNewValue)) && (pAvg->IacPkUpdate == IDLE)){
      
      if(DCP_MTH_RampInit((int32_t)pCtrlData->uhIacRefInverter, (int32_t)pCtrlData->uhIacRefInverterNewValue, DPC_IAC_SOFTSTARTUP_UPDATE_DURATION, &pAvg->Iac_Ramp)){     
  //        eb1= 30; //error managment TBD
      } 
      pAvg->IacPkUpdate = RUNNING;
    }
  
    if (pAvg->IacPkUpdate == RUNNING){
      
      if ((pCtrlData->uhIacRefInverter < pCtrlData->uhIacRefInverterNewValue) || (pCtrlData->uhIacRefInverter > pCtrlData->uhIacRefInverterNewValue)) {
        pCtrlData->uhIacRefInverter = (uint16_t)DCP_MTH_RampGeneration(&pAvg->Iac_Ramp); 
      }
      else {
        pCtrlData->uhIacRefInverter = pCtrlData->uhIacRefInverterNewValue;
        pAvg->IacPkUpdate = COMPLETE;
        pAvg->IacPkUpdate = IDLE;
        }
    
    } 


}

void DPC_LPCNTRL_ACphaseCurrentUpdate(DPC_CMNDAT_PFC_ControlData_t* pCtrlData){
    
   //New Iac phase value saturation
    if((float)pCtrlData->PhaseAngleAdjDegrees_NewValue > (float)DPC_AC_PHASE_ANGLE_ADJ_MAX) pCtrlData->PhaseAngleAdjDegrees_NewValue = (float)DPC_AC_PHASE_ANGLE_ADJ_MAX;
    if((float)pCtrlData->PhaseAngleAdjDegrees_NewValue < (float)DPC_AC_PHASE_ANGLE_ADJ_MIN) pCtrlData->PhaseAngleAdjDegrees_NewValue = (float)DPC_AC_PHASE_ANGLE_ADJ_MIN;
    
    //New Iac phase/cnt conversion
    if(pCtrlData->PhaseAngleAdjDegrees_NewValue >= 0.0f){ //phase lagging
    pCtrlData->uhPhaseAngleAdjIndex_NewValue = (uint16_t)roundf(((float)pCtrlData->PhaseAngleAdjDegrees_NewValue * (float)DPC_LUT_PERIOD_POINTS) / (float)DPC_AC_PHASE_ANGLE_PERIOD);
    }
    else{ //phase leading
    pCtrlData->uhPhaseAngleAdjIndex_NewValue = (uint16_t)roundf((float)DPC_LUT_PERIOD_POINTS + (((float)pCtrlData->PhaseAngleAdjDegrees_NewValue * (float)DPC_LUT_PERIOD_POINTS) / (float)DPC_AC_PHASE_ANGLE_PERIOD));
    }
    
    
    if ((pCtrlData->uhPhaseAngleAdjIndex > pCtrlData->uhPhaseAngleAdjIndex_NewValue) || (pCtrlData->uhPhaseAngleAdjIndex < pCtrlData->uhPhaseAngleAdjIndex_NewValue)){
      pCtrlData->uhPhaseAngleAdjIndex = pCtrlData->uhPhaseAngleAdjIndex_NewValue;
      pCtrlData->PhaseAngleAdjDegrees = pCtrlData->PhaseAngleAdjDegrees_NewValue;
      
    }
      
}




