/**
******************************************************************************
* @file           : DPC_STEVAL-BIDIRCB.c
* @brief          : Application program body
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
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


#include "DPC_Application.h"

/* PACK CODE BEGIN Includes */

#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//Include HAL driver
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_hal_gpio.h"
#include <math.h>

#include "DPC_LUT.h"
#include "DPC_Math.h"
#include "DPC_Pid.h"
#include "DPC_FSM.h"
#include "DPC_AverageCurrentCtrl.h"
#include "DPC_Loopctrl.h"
#include "DPC_Faulterror.h"
#include "DPC_Timeout.h"


/* PACK CODE END Includes */


//*** LOCAL FUNCTIONS PROTOTYPES BEGIN ***//
DPC_FAULTERROR_LIST_TypeDef DPC_ProtectionDetect(void);
DPC_LPCNTRL_ConversionMode_t DPC_ModeCheck(void);
//*** LOCAL FUNCTIONS PROTOTYPES END***//

//*** STRUCT DEFINITION BEGIN ***//
DPC_CMNDAT_DataSet1_t Data_Set1; 
DPC_CMNDAT_DataSet2_t Data_Set2; 
DPC_CMNDAT_DataSet3_t Data_Set3;// need to add pre switch voltages so we can determine when to switch back
DPC_CMNDAT_PFC_RawData_t Data_adc; //***raw variable creation from data struct in DPC_CommonData.h
DPC_CMNDAT_PFC_ControlData_t Control_Data; //***control variable creation from data struct in DPC_CommonData.h
DPC_MTH_Average_t Data_Avg_IL1_avg_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average_t Data_Avg_IL2_avg_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average_t Data_Avg_IL3_avg_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average_t Data_Avg_Temp_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average3Stages_t Data_Avg_Vout_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average3Stages_t Data_Avg_Vin_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_Average4Stages_t Data_Avg_Iout_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_MovingAverage_t Data_MovAvg_Iout_PFC; //***variable creation from data struct in DPC_Math.h
DPC_MTH_RampGenerator_t Vout_PFC_Ramp;
DPC_PID_PIs32_t Vout_PI_t; //***variable creation from data struct in DPC_Pid.h
DPC_AVGCC_FeedForward_t Load_FF;
DPC_AVGCC_InputFeedForward_t Input_FF;
DPC_LUT_TTPPFC_t LUT_Tables;
DPC_LUT_ERROR_CHECK_t LUT_Error;
DPC_AVGCC_Avg3Channels_t Current_Control;
DPC_LPCNTRL_VoltageControl_t PFC_VoltageControl;
DPC_LPCNTRL_PhaseShedding_t PFC_PhaseShedding;
DPC_LPCNTRL_Inrush_t PFC_Relay;
DPC_LPCNTRL_Rly_t Mains_SW_Relay;
DPC_LPCNTRL_Led_t PFC_Led;
DPC_LPCNTRL_Fan_t PFC_Fan;
DPC_LPCNTRL_Protection_t PFC_Protection;
DPC_LPCNTRL_Feature_t PFC_Feature;
DPC_LPCNTRL_ConverterControl_t PFC_Control;

uint8_t rx_buff[3];
uint16_t Vbus_ref_rx=0;


//*** STRUCT DEFINITION END ***//


/* PACK CODE BEGIN PV */
/*!FSM*/

/**
* @}
*/
void DPC_APPLICATION_Init(void)
{
  /* PACK CODE BEGIN 2 */
  

   /*** struct init BEGIN ***/

   //*** Data process init ***//
   //IL1
   Data_Avg_IL1_avg_PFC.uhWeight = DPC_IL1_WEIGHT_1;
   //IL2
   Data_Avg_IL2_avg_PFC.uhWeight = DPC_IL2_WEIGHT_1;
   //IL3
   Data_Avg_IL3_avg_PFC.uhWeight = DPC_IL3_WEIGHT_1;
   //Vout  
   Data_Avg_Vout_PFC.stage_1.uhWeight = DPC_VOUT_WEIGHT_1;
   Data_Avg_Vout_PFC.stage_2.uhWeight = DPC_VOUT_WEIGHT_2;
   Data_Avg_Vout_PFC.stage_3.uhWeight = DPC_VOUT_WEIGHT_3;
   //Iout
   Data_Avg_Iout_PFC.stage_1.uhWeight = DPC_IOUT_WEIGHT_1;
   Data_Avg_Iout_PFC.stage_2.uhWeight = DPC_IOUT_WEIGHT_2;
   Data_Avg_Iout_PFC.stage_3.uhWeight = DPC_IOUT_WEIGHT_3;
   Data_Avg_Iout_PFC.stage_4.uhWeight = DPC_IOUT_WEIGHT_4;
   Data_MovAvg_Iout_PFC.uhNumSamples = DPC_AVG_SAMPLE;
   //Ambient Temperature
   Data_Avg_Temp_PFC.uhWeight = DPC_TEMP_WEIGHT_1;
   //Vin
   Data_Avg_Vin_PFC.stage_1.uhWeight = DPC_VIN_WEIGHT_1;
   Data_Avg_Vin_PFC.stage_2.uhWeight = DPC_VIN_WEIGHT_2;
   Data_Avg_Vin_PFC.stage_3.uhWeight = DPC_VIN_WEIGHT_3; 

   
   //*** Relay control init ***//
   DPC_LPCNTRL_RelayControlInit(&PFC_Relay);
   DPC_LPCNTRL_MainsSwControlInit(&Mains_SW_Relay);

   //*** Voltage control init ***//
   DPC_LPCNTRL_VoltageControlInit(&PFC_VoltageControl);
   
   //*** Phase shedding control init ***// 
   DPC_LPCNTRL_PhaseSheddingControlInit(&PFC_PhaseShedding);

   //*** Protection control init ***//
   DPC_LPCNTRL_ProtectionControlInit(&PFC_Protection);
  
   //*** PFC control init ***//
   DPC_LPCNTRL_ConverterControlInit(&PFC_Control);   
      
   //*** Control data init ***//
   Control_Data.ILtotGain = (float)((float)DPC_IL_TOT_GAIN * (float)1.0f);
   Control_Data.ILGain = (float)((float)DPC_IL_GAIN * (float)1.0f);
   Control_Data.IdcGain = (float)((float)DPC_IDC_GAIN * (float)1.0f);
   Control_Data.VacGain = (float)((float)DPC_VAC_GAIN * (float)1.0f);
   Control_Data.VdcGain = (float)((float)DPC_VBUS_GAIN * (float)1.0f);
   Control_Data.uhVinRmsMin = (DPC_VIN_MIN * DPC_VIN_MIN_TOLERANCE) / DPC_VIN_TOLERANCE_DIV;
   Control_Data.uhVinRmsMax = (DPC_VIN_MAX * DPC_VIN_MAX_TOLERANCE) / DPC_VIN_TOLERANCE_DIV;
   Control_Data.uhVinRmsFactor = (uint16_t)roundf((float)448.0 * (float)2.60360551431601 / (float)Control_Data.VacGain);
   Control_Data.uhVinRmsDer = (DPC_VIN_MIN_DER * DPC_VIN_MIN_DER_TOLERANCE) / DPC_VIN_TOLERANCE_DIV;
   Control_Data.uhVoutRef = (uint16_t)roundf((float)DPC_VBUS_REF_DEFAULT * (float)Control_Data.VdcGain);
   Control_Data.uhVoutBurstMax = (uint16_t)(roundf((float)Control_Data.uhVoutRef * (float)DPC_VBURST_UPDATE_COEFF_MAX));
   Control_Data.uhVoutBurstMin = (uint16_t)(roundf((float)Control_Data.uhVoutRef * (float)DPC_VBURST_UPDATE_COEFF_MIN));
   Control_Data.uhVoutOverVoltageValue = (uint16_t)roundf((float)DPC_VBUS_OVP * (float)Control_Data.VdcGain);
   Control_Data.uhVbusUnderVoltageValuePFC = (uint16_t)roundf((float)DPC_VBUS_UVP_PFC * (float)Control_Data.VdcGain);
   Control_Data.uhVbusUnderVoltageValueInverter = (uint16_t)roundf((float)DPC_VBUS_UVP_INVERTER * (float)Control_Data.VdcGain);
   Control_Data.uwVoutSetPoint = 0;
   Control_Data.uhVoutNewSetpoint = Control_Data.uhVoutRef;
   Control_Data.uhVinDropoutThresholdDetect = (uint16_t)roundf((float)DPC_VIN_DROPOUT_DETECT * (float)Control_Data.VacGain); // Input voltage threshold for Drop-Out detect
   Control_Data.uhVinDropoutThresholdEnd = (Control_Data.uhVinDropoutThresholdDetect * DPC_VIN_DROPOUT_END_FACTOR)/DPC_VIN_DROPOUT_DIV; // Input voltage threshold for Drop-Out end
   Control_Data.uhIacRefInverter = (uint16_t)roundf((float)DPC_IPK_REF_INVERTER_DEFAULT * (float)Control_Data.ILtotGain);
   Control_Data.uhIacRefInverterNewValue = Control_Data.uhIacRefInverter;
   Control_Data.PhaseAngleAdjDegrees = (float)DPC_AC_PHASE_ANGLE_ADJ;
   Control_Data.PhaseAngleAdjDegrees_NewValue = Control_Data.PhaseAngleAdjDegrees;
   Control_Data.uhPhaseAngleAdjIndex = (uint16_t)roundf(((float)Control_Data.PhaseAngleAdjDegrees * (float)DPC_LUT_PERIOD_POINTS) / (float)DPC_AC_PHASE_ANGLE_PERIOD);
   Control_Data.uhPhaseAngleAdjIndex_NewValue = Control_Data.uhPhaseAngleAdjIndex;
   Control_Data.uhIdcLoadConnected = (uint16_t)roundf((float)DPC_POUT / (float)DPC_VBUS_REF_DEFAULT * (float)DPC_IDC_LOAD_CONNECTED_PERC/100.0f * (float)Control_Data.IdcGain);
   Control_Data.uhIdcLoadDisconnected = (uint16_t)roundf((float)Control_Data.uhIdcLoadConnected * 0.7f);
   Control_Data.uhIdcLoadStartupMaxValue = (uint16_t)roundf((float)DPC_POUT / (float)DPC_VBUS_REF_DEFAULT * (float)DPC_IDC_MAX_LOAD_STARTUP_PERC/100.0f * (float)Control_Data.IdcGain);
   
   //*** Fan control init ***//  
   DPC_LPCNTRL_FanControlInit(&PFC_Fan);
   PFC_Fan.uhIoutFan = (uint16_t)roundf((float)DPC_POUT / (float)DPC_VBUS_REF_DEFAULT * (float)DPC_ILOAD_FAN_PERC/100.0f * (float)Control_Data.IdcGain);
   //*** Voltage PI regulator init ***//
   DPC_PIs32Init(&Vout_PI_t, 
                 (uint32_t)DPC_KP_VDC_MICRO, (uint32_t)DPC_KI_VDC_MICRO,
                 (int32_t)PFC_VoltageControl.wIpkMax, (int32_t)-PFC_VoltageControl.wIpkMax,
                 (int32_t)PFC_VoltageControl.wIpkMax, (int32_t)-PFC_VoltageControl.wIpkMax,
                 DPC_VOLTAGE_PI_SAT_ENABLE, DPC_VOLTAGE_PI_ANTIWINDUP_ENABLE,
                 (int32_t) DPC_VOLTAGE_PI_ANTIWINDUP_GAIN, (int32_t) DPC_VOLTAGE_PI_RESET_VALUE);
   
   
   
   //*** LUT init ***//
   DPC_LUT_Init(&LUT_Tables, &LUT_Error);
   Current_Control.uwIzeroOffsetIL = (uint32_t)roundf((float)DPC_IZERO_OFFSET_IL * (float)1.0f);
   Current_Control.uwIzeroOffsetIdc = (uint32_t)roundf((float)DPC_IZERO_OFFSET_IDC * (float)1.0f);   
   Current_Control.uwRisingDeadTimeMax = DPC_RISING_DEAD_TIME_MAX;
   Current_Control.uwFallingDeadTimeMax = DPC_FALLING_DEAD_TIME_MAX;  
   Current_Control.uwRisingDeadTimeMin = DPC_RISING_DEAD_TIME_MIN;
   Current_Control.uwFallingDeadTimeMin = DPC_FALLING_DEAD_TIME_MIN; 
   Current_Control.uwTswPeriod = (uint32_t)roundf((1.0f/(float)DPC_SWITCHING_FREQUENCY) * (float)DPC_HRTIM_FREQ);
   Current_Control.uwPhaseShiftCh2Mode2 = Current_Control.uwTswPeriod * (uint32_t)DPC_PHASESHIFT_CH2_2 / (uint32_t)DPC_PHASE_PERIOD;
   Current_Control.uwPhaseShiftCh2Mode3 = Current_Control.uwTswPeriod * (uint32_t)DPC_PHASESHIFT_CH2_3 / (uint32_t)DPC_PHASE_PERIOD;
   Current_Control.uwPhaseShiftCh3 = Current_Control.uwTswPeriod * (uint32_t)DPC_PHASESHIFT_CH3 / (uint32_t)DPC_PHASE_PERIOD;
   Current_Control.uwTonMax = (uint32_t)((float)Current_Control.uwTswPeriod * (float)DPC_DUTY_MAX);
   Current_Control.uwTonMin = (uint32_t)((float)Current_Control.uwTswPeriod * (float)DPC_DUTY_MIN);
   
     //*** Total Current PI init ***//
   DPC_PIs32Init(&Current_Control.Iac_PI, 
                 (uint32_t)DPC_KP_IAC_MICRO, (uint32_t)DPC_KI_IAC_MICRO,
                 (int32_t)Current_Control.uwTonMax, (int32_t)Current_Control.uwTonMin,
                 (int32_t)Current_Control.uwTonMax, (int32_t)Current_Control.uwTonMin,
                 DPC_CURRENT_PI_SAT_ENABLE, DPC_CURRENT_PI_ANTIWINDUP_ENABLE,
                 (int32_t) DPC_CURRENT_PI_ANTIWINDUP_GAIN, (int32_t) DPC_CURRENT_PI_RESET_VALUE);
   //*** Average current control init ***//
   DPC_AVGCC_AvgControlInit(&Current_Control, &PFC_VoltageControl);
   
   //*** Average current control driving init ***//
   DPC_AVGCC_AvgDrivingActuationInit(&Current_Control, DPC_NORMAL_CYCLE_DRIVING);
  
   /*** struct init END ***/
   
   //*** BOOT0 SECURE LOCK AVOID UNDESIRED MCU STARTUP BEHAVIOR *****************//   
FLASH_OBProgramInitTypeDef OptionsBytesStruct;
OptionsBytesStruct.OptionType=OPTIONBYTE_BOOT_LOCK;
HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

if (OptionsBytesStruct.BootEntryPoint!=OB_BOOT_LOCK_ENABLE){
HAL_FLASH_Unlock();
HAL_FLASH_OB_Unlock();
OptionsBytesStruct.BootEntryPoint=OB_BOOT_LOCK_ENABLE;
  if (HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK){
    Error_Handler();
  }
HAL_FLASH_OB_Lock();
HAL_FLASH_OB_Launch();
HAL_FLASH_Lock();
}
//*** BOOT0 SECURE LOCK AVOID UNDESIRED MCU STARTUP BEHAVIOR *****************// 
   
   
   /*** Peripheral start/config BEGIN ***/
   //*** DMA NVIC disable***//
   HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
   HAL_NVIC_DisableIRQ(DMA1_Channel8_IRQn);
   
   //*** DACs start***//
   HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
   HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);   
   HAL_DAC_Start(&hdac2,DAC_CHANNEL_1);
   HAL_DAC_Start(&hdac3,DAC_CHANNEL_1);
   HAL_DAC_Start(&hdac3,DAC_CHANNEL_2); 
   HAL_DAC_Start(&hdac4,DAC_CHANNEL_1);
   HAL_DAC_Start(&hdac4,DAC_CHANNEL_2);

   //*** Comparators start***//
   HAL_COMP_Start(&hcomp1);
   HAL_COMP_Start(&hcomp2);
   HAL_COMP_Lock(&hcomp2); 
   HAL_COMP_Start(&hcomp3);
   HAL_COMP_Start(&hcomp4);
   HAL_COMP_Start(&hcomp5);
   HAL_COMP_Start(&hcomp6);
   HAL_COMP_Start(&hcomp7);
   HAL_COMP_Lock(&hcomp7); //lock OCP configuration in read-only to avoid any register alteration
   HAL_OPAMP_Start(&hopamp2);

   //*** TIM start***//  
   HAL_TIM_Base_Start_IT(&htim2);
   HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1 , HAL_MASTER_TIMER_ID + HAL_SLAVE1_TIMER_ID + HAL_SLAVE2_TIMER_ID);// + HRTIM_TIMERID_TIMER_D + HRTIM_TIMERID_TIMER_E + HRTIM_TIMERID_TIMER_F);   
    
   //*** ADCs calibration***//
   HAL_ADCEx_Calibration_Start(&ADC_SET1_ID, ADC_SINGLE_ENDED);
//   HAL_ADCEx_Calibration_Start(&ADC_SET2_ID, ADC_SINGLE_ENDED);
   
   //*** ADCs start with DMA***//
   HAL_ADC_Start_DMA(&ADC_SET1_ID,(uint32_t*)&Data_Set1,ADC_SET1_LENGTH);

   //TODO need to set up ACD_SET3
   /*** Peripheral start/config END ***/
   
//   HAL_UART_Receive_IT(&huart4,rx_buff,3);
  
  /* PACK CODE END 2 */
  
}

/*
* LM background task
*/
void DPC_APPLICATION_Process(void)
{
  
  
/* PACK CODE BEGIN 3 */


   
// MAIN STATE MACHINE BEGIN_____________________________________________________
   DPC_FSM_Application(); //0.9us with High-speed OPT
// MAIN STATE MACHINE END_______________________________________________________      
      

      
  //* PACK CODE END 3 */

}

/**
  * @brief  Executes converter's state machine WAIT STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_WAIT_Func(void)
{
bool RetVal = true;

  if (PFC_Control.Flag){
    PFC_Control.Flag = RESET;
    LED_YELLOW_ON;
    PFC_Relay.RelayInrushState = INIT;
    DPC_TO_Set(DPC_TO_INRUSH, DPC_TO_INRUSH_TICK);
  }
    
  Control_Data.uhVinPkVolt = ((uint32_t)Control_Data.uhVinRmsVolt * DPC_VIN_PK_FACTOR) / DPC_VIN_GAIN_DIV;
//  Control_Data.uhVinPkToVout = ((uint32_t)Control_Data.uhVinPkVolt * DPC_AVOUT) / DPC_VIN_GAIN_DIV;
//  Control_Data.uhDeltaVinPkVoutRelayToVout = ((uint32_t)DPC_DELTA_VINPK_VOUT_RELAY * DPC_AVOUT) / DPC_VIN_GAIN_DIV;
  Control_Data.uhVinPkToVout = (uint16_t)((float)Control_Data.uhVinPkVolt * (float)Control_Data.VdcGain);
  Control_Data.uhDeltaVinPkVoutRelayToVout = (uint16_t)((float)DPC_DELTA_VINPK_VOUT_RELAY * (float)Control_Data.VdcGain);
  
  if(Control_Data.uhVinPkToVout >= Control_Data.uhDeltaVinPkVoutRelayToVout){
    Control_Data.uhVoutMinRelay = Control_Data.uhVinPkToVout - Control_Data.uhDeltaVinPkVoutRelayToVout;
  }
  else{
    Control_Data.uhVoutMinRelay = Control_Data.uhVinPkToVout;
  }
                

  if ((DPC_TO_Check(DPC_TO_INRUSH) == TO_OUT_TOOK) && (PFC_Relay.RelayInrushState == INIT) && (Control_Data.uhAvgVout >= Control_Data.uhVoutMinRelay )) {
    PFC_Relay.RelayInrushState = RUNNING;
   
  }
  
  if ((DPC_TO_Check(DPC_TO_INRUSH) == TO_OUT_TOOK) && (PFC_Relay.RelayInrushState == COMPLETE)) {
    RELAY_ON; //Relay turn-on command
    PFC_Relay.RelayInrushState = IDLE;
  }
  
            
  if (PFC_Relay.Relay == GPIO_PIN_SET){
    if (PFC_Protection.OverCurrentProtection == ENABLE) {
      DPC_LPCNTRL_HardwareFaultEnable();
    }
    PFC_Control.Flag = SET;
    PFC_Control.ubS = FREQUENCY_DETECT;
    DPC_FSM_State_Set(DPC_FSM_IDLE);
  }

 return RetVal;
}

/**
  * @brief  Executes converter's state machine IDLE STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_IDLE_Func(void)
{
bool RetVal = true;


  if (PFC_Control.Flag){
     // frequency measurement system reset
    HAL_TIM_Base_Stop_IT(&htim3); // MULTIPLIER INTERRUPT DISABLE;
    Current_Control.ZvdFilter = DISABLE;
    Current_Control.ZvdFrequencyFollower = DISABLE;
    MX_TIM3_Init();
    Current_Control.uhRunIndex = 0;
    Current_Control.uhSyncIndex = 0;
    HAL_NVIC_EnableIRQ(ZVD_VACsign_F_EXTI_IRQn); // ZVD interrupt enable rising edge
    Current_Control.FrequencyCheck = IDLE;
    PFC_Control.Flag = RESET;
  }
  
  //*** MAINS VOLTAGE/FREQUENCY CHECK BEGIN ***//
  if (Current_Control.FrequencyCheck == IDLE) {
    Current_Control.FrequencyCheck = START;
  }
        if (Current_Control.FrequencyCheck == COMPLETE) {
            Current_Control.FrequencyCheck = IDLE;
            
          if ((Current_Control.ubLineFrequencyMeasure >= DPC_LINE_FREQ_MIN) && (Current_Control.ubLineFrequencyMeasure <= DPC_LINE_FREQ_MAX)){
            Current_Control.uhZvdTimerCounterMin = (Current_Control.uhZvdTimerCounterDefault * DPC_LINE_FREQ_NOM) / (DPC_LINE_FREQ_MAX + 2); //66; 
            Current_Control.uhZvdTimerCounterMed = (Current_Control.uhZvdTimerCounterDefault * DPC_LINE_FREQ_NOM) / Current_Control.ubLineFrequencyMeasure;
            Current_Control.uhZvdTimerCounterMax = (Current_Control.uhZvdTimerCounterDefault * DPC_LINE_FREQ_NOM)/ (DPC_LINE_FREQ_MIN - 2); //44;  
            LL_TIM_SetAutoReload(TIM3,Current_Control.uhZvdTimerCounterMed);
            Current_Control.uhZvdTimerCounter = Current_Control.uhZvdTimerCounterMed; 
            Current_Control.ZvdFrequencyFollower = ENABLE;
            Current_Control.ubLineFrequencyValue = Current_Control.ubLineFrequencyMeasure;
            }
    
 
    //*** INPUT VOLTAGE CHECK BEGIN ***// 
    if ((Control_Data.uhVinRmsVolt >= Control_Data.uhVinRmsMin) && (Control_Data.uhVinRmsVolt <= Control_Data.uhVinRmsMax)){
      DPC_AVGCC_InputVoltageFeedForwardInit(&Input_FF, Control_Data.uhVinRmsVolt, DPC_VIN_NOM, DPC_VIN_MIN, DPC_VIN_MAX);
      PFC_VoltageControl.uhVinKff = Input_FF.uhKff;           
      if(DCP_AVGCC_FeedForwardInit(&Load_FF, (uint8_t) DPC_FF_SHIFT, (uint8_t) DPC_FF_TRIPPING,
                                 (int32_t) DPC_FF_UP, (int32_t) DPC_FF_DOWN,
                                 (int32_t) DPC_FF_UPPER_THRESHOLD, (int32_t) DPC_FF_LOWER_THRESHOLD)){
        //error managment   
      }
    }
    

    if ((Control_Data.uhVinRmsVolt >= Control_Data.uhVinRmsMin) && (Control_Data.uhVinRmsVolt <= Control_Data.uhVinRmsMax) &&
        (Current_Control.ubLineFrequencyMeasure >= DPC_LINE_FREQ_MIN) && (Current_Control.ubLineFrequencyMeasure <= DPC_LINE_FREQ_MAX) &&
         Control_Data.uhVinRmsVolt!=0){
    PFC_Control.Flag = SET;           
    PFC_Control.ubS = ILOAD_CHECK;
    PFC_Control.ConversionStart = DPC_MANUAL_START;
    DPC_FSM_State_Set(DPC_FSM_INIT); 
    } 
    else {
      PFC_Protection.ubErrorCode = 10;
      PFC_Control.Flag = SET;
      PFC_Control.ubS = FREQUENCY_DETECT;
      Current_Control.FrequencyCheck = IDLE;
    }
  }// if (Current_Control.FrequencyCheck == COMPLETE)
  
  //*** MAINS VOLTAGE/FREQUENCY CHECK BEGIN ***//
  
return RetVal;
}


/**
  * @brief  Executes converter's state machine START STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_INIT_Func(void)
{
bool RetVal = true; 


DPC_FAULTERROR_LIST_TypeDef ProtectionDetect_status = NO_FAULT;
  
  ProtectionDetect_status = DPC_ProtectionDetect();
  
  if (ProtectionDetect_status){
     PFC_Control.Flag = SET;
     DPC_FLT_Faulterror_Set(ProtectionDetect_status);
     PFC_Control.ubS = HALT;
     DPC_FSM_State_Set(DPC_FSM_STOP);
     RetVal = false; 
  }
  else{
    
      if (PFC_Control.Flag){
          PFC_Control.Flag = RESET;
   }
    //*** Inductor current sense calibration BEGIN ***//
   if(Current_Control.CurrentSensorCalibrationDone == RESET){
    DPC_AVGCC_CurrentSenseCalibration(&Current_Control,
                                    Control_Data.uhAvgIL1,
                                    Control_Data.uhAvgIL2,
                                    Control_Data.uhAvgIL3,
                                    Data_adc.uhIinFlt,
                                    Data_Set1.uhBufferADC2[IOUT_RANK_ID]);
    
   }
   //*** Inductor current sense calibration END ***//
   
   //*** Protection control calibration BEGIN ***//
   DPC_LPCNTRL_ProtectionControlCalibration(&PFC_Protection, &Control_Data);
   //*** Protection control calibration END ***// 

    if (PFC_Control.ConversionStart){
      if (PFC_Control.ConversionMode == DPC_PFC_MODE){
                        //*** No-Load Start-up
                        if (Data_Avg_Iout_PFC.stage_3.uhAvgVal < Control_Data.uhIdcLoadConnected){
                            Current_Control.ZvdFilter = ENABLE;
                            PFC_Control.Flag = SET;
                            PFC_Control.ubS = BURST_MODE;
                      PFC_Control.ubRunState = BURST_MODE;
                      DPC_FSM_State_Set(DPC_FSM_RUN);
                    }
                        //*** Load Start-up
                    else {
                      if ((Data_Avg_Iout_PFC.stage_3.uhAvgVal >= Control_Data.uhIdcLoadConnected) && (Data_Avg_Iout_PFC.stage_3.uhAvgVal <= Control_Data.uhIdcLoadStartupMaxValue)){
                      Current_Control.ZvdFilter = ENABLE;
                      PFC_Control.Flag = SET;
                      PFC_Control.ubS = SOFT_STARTUP;
                      DPC_FSM_State_Set(DPC_FSM_START);
                      }
                    } //else
           }//if (PFC_Control == DPC_PFC_MODE)
      else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
        Current_Control.ZvdFilter = ENABLE;
        PFC_Control.Flag = SET;
        PFC_Control.ubS = SOFT_STARTUP;
        DPC_FSM_State_Set(DPC_FSM_START);
        }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)
      else if (PFC_Control.ConversionMode == DPC_NONE_MODE){
        Current_Control.ZvdFilter = ENABLE;
        PFC_Control.Flag = SET;
        PFC_Control.ubS = SOFT_STARTUP;
        DPC_FSM_State_Set(DPC_FSM_START);  
        }//else if (PFC_Control.ConversionMode == DPC_NONE_MODE)
    }
 }// else 
 
 
 return RetVal;
}

/**
  * @brief  Executes converter's state machine START STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_START_Func(void)
{
bool RetVal = true; 

DPC_FAULTERROR_LIST_TypeDef ProtectionDetect_status = NO_FAULT;
  DPC_LPCNTRL_ConverterControl_t current_mode;
  
  ProtectionDetect_status = DPC_ProtectionDetect();
  
  
  if (ProtectionDetect_status){
     PFC_Control.Flag = SET;
     DPC_FLT_Faulterror_Set(ProtectionDetect_status);
     PFC_Control.ubS = HALT;
     DPC_FSM_State_Set(DPC_FSM_STOP);
     RetVal = false; 
  } 
  else{ 
        if (PFC_Control.Flag){
          DPC_TO_Set(DPC_TO_PFC_START, DPC_TO_PFC_START_TICK);
          PFC_Control.Flag = RESET;
        }
        if (DPC_TO_Check(DPC_TO_PFC_START) == TO_OUT_TOOK){
          
          if (PFC_Control.ConversionMode == DPC_PFC_MODE){
            PFC_VoltageControl.SoftStartup = START;
            PFC_VoltageControl.VoltageControl = ENABLE;
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_PFC_MODE;
            PFC_Control.ubRunState = RUN_PFC_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }
          else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
            Current_Control.IacPkSoftStart = START; //Iac softstart
//            Current_Control.IacPkSoftStart = IDLE; //Iac step
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_INVERTER_MODE;
            PFC_Control.ubRunState = RUN_INVERTER_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)
          
//          PFC_Control.Flag = SET;
//          PFC_Control.ubS = RUN_PFC_MODE;
//          PFC_Control.ubRunState = RUN_PFC_MODE;
//          DPC_FSM_State_Set(DPC_FSM_RUN);
        }
      }//else
 return RetVal;
}


/**
  * @brief  Executes converter's state machine RUN STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_RUN_Func(void)
{
bool RetVal = true;

DPC_FAULTERROR_LIST_TypeDef ProtectionDetect_status = NO_FAULT;
  
  ProtectionDetect_status = DPC_ProtectionDetect();
  
  if (ProtectionDetect_status){
     DPC_FLT_Faulterror_Set(ProtectionDetect_status);
     PFC_Control.Flag = SET;
     PFC_Control.ubS = HALT;
     DPC_FSM_State_Set(DPC_FSM_STOP);
     RetVal = false; 
  }
  else{ 
    switch (PFC_Control.ubRunState){
      case RUN_PFC_MODE:{
        if (PFC_Control.Flag){
          PFC_VoltageControl.VoltageControl = ENABLE;
          
//          //could be done in the previous state
//          if (PFC_Control.ConversionMode == DPC_PFC_MODE){
//            PFC_VoltageControl.VoltageControl = ENABLE;
//            }
//          else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
//          }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)
//          //could be done in the previous state
          
          
          PFC_Control.Flag = RESET;
        }

////        // DROP-OUTPUT DETECT CONDITION 
////        if (Data_Avg_Vin_PFC.stage_1.uhAvgVal <= Control_Data.uhVinDropoutThresholdDetect){          
////          PFC_Control.Flag = SET;
////          PFC_Control.ubS = DROP_OUT;
////          PFC_Control.ubRunState = DROP_OUT;
////          break;
////        }



        
//           if (PFC_Control.ConversionMode == DPC_PFC_MODE){
                  //BURST DETECTION IN PFC MODE
              if ((Data_Avg_Vout_PFC.stage_1.uhAvgVal > Control_Data.uhVoutBurstMax) || (Data_Avg_Iout_PFC.stage_3.uhAvgVal < Control_Data.uhIdcLoadDisconnected)){
                PFC_Control.Flag = SET;
                PFC_Control.ubS = BURST_MODE;
                PFC_Control.ubRunState = BURST_MODE;
                break;
              }
//             }
//           else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
//                 //BURST DETECTION IN INVERTER MODE
//              if ((Data_Avg_Vout_PFC.stage_1.uhAvgVal > Control_Data.uhVoutBurstMax)){
//                PFC_Control.Flag = SET;
//                PFC_Control.ubS = HALT;
//                DPC_FSM_State_Set(DPC_FSM_STOP);
//                break;
//              }
//             }        
                
        
//        //BURST DETECTION
//        if ((Data_Avg_Vout_PFC.stage_1.uhAvgVal > Control_Data.uhVoutBurstMax) || (Data_Avg_Iout_PFC.stage_3.uhAvgVal < DPC_ILOAD_MAX_CONF-5)){
//          PFC_Control.Flag = SET;
//          PFC_Control.ubS = PFC_BURST;
//          PFC_Control.ubRunState = PFC_BURST;
//          break;
//        }
      
        break;

    }//PFC_MODE_RUN
      case RUN_INVERTER_MODE:{
        if (PFC_Control.Flag){
//          PFC_VoltageControl.VoltageControl = ENABLE;   
          PFC_Control.Flag = RESET;
        }

////        // DROP-OUTPUT DETECT CONDITION 
////        if (Data_Avg_Vin_PFC.stage_1.uhAvgVal <= Control_Data.uhVinDropoutThresholdDetect){          
////          PFC_Control.Flag = SET;
////          PFC_Control.ubS = DROP_OUT;
////          PFC_Control.ubRunState = DROP_OUT;
////          break;
////        }       
        break;

    }//RUN_INVERTER_MODE
      case BURST_MODE:{
        if (PFC_Control.Flag){
          DPC_LPCNTRL_DrivingOutputStop(); 
          Current_Control.BoostSwitchCh1 = RESET;
          Current_Control.BoostSwitchCh2 = RESET;
          Current_Control.BoostSwitchCh3 = RESET; 
          Current_Control.RectSwitchCh1 = RESET;
          Current_Control.RectSwitchCh2 = RESET;
          Current_Control.RectSwitchCh3 = RESET;
          Current_Control.CurrentControl = DISABLE;
          DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
          Current_Control.DutySoftStart = IDLE;
          PFC_VoltageControl.BurstRising = IDLE;
          PFC_VoltageControl.VoltageControl = DISABLE;
          PFC_VoltageControl.wIpkPI = 0;
          PFC_VoltageControl.wIpk = 0; 
          Vout_PI_t.wOut=0;
          Vout_PI_t.wIntegral=0;
          PFC_Control.Flag = RESET;
          break;
        }
 
////        // DROP-OUTPUT DETECT CONDITION 
////        if (Data_Avg_Vin_PFC.stage_1.uhAvgVal <= Control_Data.uhVinDropoutThresholdDetect){ 
////          PFC_Control.Flag = SET;
////          PFC_Control.ubS = DROP_OUT;
////          PFC_Control.ubRunState = DROP_OUT;
////          break;
////        }
        
        // Burst end condition without load
        if ((PFC_VoltageControl.BurstRising == IDLE) && (Data_adc.uhVout < Control_Data.uhVoutBurstMin) && (Data_Avg_Iout_PFC.stage_3.uhAvgVal < Control_Data.uhIdcLoadConnected)){
          PFC_VoltageControl.BurstRising = START;
          break;
        }
        if (PFC_VoltageControl.BurstRising == RUNNING){
          if (Data_Avg_Vout_PFC.stage_1.uhAvgVal >= Control_Data.uhVoutBurstMax){ 
          
            PFC_VoltageControl.BurstRising = COMPLETE;
            PFC_VoltageControl.BurstRising = IDLE;
            DPC_LPCNTRL_DrivingOutputStop();
            Current_Control.BoostSwitchCh1 = RESET;
            Current_Control.BoostSwitchCh2 = RESET;
            Current_Control.BoostSwitchCh3 = RESET; 
            Current_Control.RectSwitchCh1 = RESET;
            Current_Control.RectSwitchCh2 = RESET;
            Current_Control.RectSwitchCh3 = RESET;
            Current_Control.CurrentControl = DISABLE;
            DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
            Current_Control.DutySoftStart = IDLE;
            if (PFC_VoltageControl.PFC_OK == RESET){
              LED_YELLOW_OFF;
              LED_GREEN_ON;
              PFC_VoltageControl.PFC_OK = SET;
            }
          }
        }
        
        // Burst end condition with load
        if ((Data_adc.uhVout < Control_Data.uhVoutBurstMin) && (Data_Avg_Iout_PFC.stage_3.uhAvgVal >= Control_Data.uhIdcLoadConnected)){
          DPC_LPCNTRL_DrivingOutputStop();
          Current_Control.BoostSwitchCh1 = RESET;
          Current_Control.BoostSwitchCh2 = RESET;
          Current_Control.BoostSwitchCh3 = RESET; 
          Current_Control.RectSwitchCh1 = RESET;
          Current_Control.RectSwitchCh2 = RESET;
          Current_Control.RectSwitchCh3 = RESET;
          Current_Control.CurrentControl = DISABLE;
          DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
          Current_Control.DutySoftStart = IDLE;
          PFC_VoltageControl.BurstRising = COMPLETE;        
              PFC_Control.Flag = SET;
              PFC_Control.ubS = RUN_PFC_MODE;
              PFC_Control.ubRunState = RUN_PFC_MODE;
          break;
        }
        break;
    }//PFC_BURST  
      case DROP_OUT:{
        if (PFC_Control.Flag){
          DPC_LPCNTRL_DrivingOutputStop();
          Current_Control.BoostSwitchCh1 = RESET;
          Current_Control.BoostSwitchCh2 = RESET;
          Current_Control.BoostSwitchCh3 = RESET; 
          Current_Control.RectSwitchCh1 = RESET;
          Current_Control.RectSwitchCh2 = RESET;
          Current_Control.RectSwitchCh3 = RESET;
          Current_Control.CurrentControl = DISABLE;
          DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
          Current_Control.DutySoftStart = IDLE;
          LF_LS_OFF;
          LF_HS_OFF;
          PFC_VoltageControl.wIpk = 0;
          DPC_TO_Set(DPC_TO_DROP_OUT, DPC_TO_DROP_OUT_TICK); // we need to drop into inverter mode here
          PFC_Control.Flag = RESET;
          break;
        }
//        if (DPC_TO_Check(DPC_TO_DROP_OUT) == TO_OUT_TOOK){
//          PFC_Protection.ubErrorCode = 11;
//          PFC_Control.Flag = SET;
//          PFC_Control.ubS = HALT;
//          DPC_FSM_State_Set(DPC_FSM_STOP);
//          break;
//        }
        
          if (PFC_Control.ConversionMode == DPC_PFC_MODE){
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_PFC_MODE;
            PFC_Control.ubRunState = RUN_PFC_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }
          else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_INVERTER_MODE;
            PFC_Control.ubRunState = RUN_INVERTER_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)        
        
       
        
        
        // DROP-OUTPUT END CONDITION
        if (Data_Avg_Vin_PFC.stage_1.uhAvgVal > Control_Data.uhVinDropoutThresholdEnd){
          if (PFC_Control.ConversionMode == DPC_PFC_MODE){
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_PFC_MODE;
            PFC_Control.ubRunState = RUN_PFC_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }
          else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
            PFC_Control.Flag = SET;
            PFC_Control.ubS = RUN_INVERTER_MODE;
            PFC_Control.ubRunState = RUN_INVERTER_MODE;
            DPC_FSM_State_Set(DPC_FSM_RUN);
          }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)             
          break;
        }
        break;
      }//DROP_OUT
  } //switch (RUN_STATE)
 }//else OverVoltage prot.
 return RetVal;
}



/**
  * @brief  Executes converter's state machine STOP STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_STOP_Func(void)
{
bool RetVal = true;   

  if (PFC_Control.Flag){
    DPC_LPCNTRL_DrivingOutputStop();
    Current_Control.BoostSwitchCh1 = RESET;
    Current_Control.BoostSwitchCh2 = RESET;
    Current_Control.BoostSwitchCh3 = RESET; 
    Current_Control.RectSwitchCh1 = RESET;
    Current_Control.RectSwitchCh2 = RESET;
    Current_Control.RectSwitchCh3 = RESET;
    Current_Control.CurrentControl = DISABLE;
    DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
    Current_Control.DutySoftStart = IDLE;
    PFC_VoltageControl.VoltageControl = DISABLE;
    PFC_VoltageControl.wIpkPI = 0;
    PFC_VoltageControl.SoftStartup = IDLE;
    PFC_VoltageControl.BurstRising = IDLE;
    Current_Control.FrequencyCheck = IDLE;
    PFC_VoltageControl.uhIpkIoutFF = 0;
    PFC_VoltageControl.wIpk = 0;
    PFC_VoltageControl.PFC_OK = RESET;
    LED_GREEN_OFF;
    LED_YELLOW_ON;      
    PFC_Control.ConversionStart = DISABLE; 
    DPC_TO_Set(DPC_TO_PFC_STOP, DPC_TO_PFC_STOP_TICK);
    PFC_Control.Flag = RESET;
   }
        if (DPC_TO_Check(DPC_TO_PFC_STOP) == TO_OUT_TOOK){
          PFC_Control.Flag = SET;
    PFC_Control.ubS = HALT;
    DPC_FSM_State_Set(DPC_FSM_ERROR);
   }
 return RetVal;
}

/**
  * @brief  Executes converter's state machine ERR/FAUL STate Function
  * @param  None
  * @retval true/false
  */
bool DPC_FSM_ERROR_Func(void)
{
bool RetVal = true; 

  DPC_FAULTERROR_LIST_TypeDef ProtectionDetect_status = NO_FAULT;
  
  ProtectionDetect_status = DPC_FLT_Faulterror_Check();
  
  if ((ProtectionDetect_status == FAULT_OCS) ||
      (ProtectionDetect_status == FAULT_OVL) ||
      (ProtectionDetect_status == FAULT_AOT) ||
      (ProtectionDetect_status == FAULT_ACF_OR) ||
      (ProtectionDetect_status == FAULT_CURR_CALIB)||
      (ProtectionDetect_status == FAULT_NONE_MODE))  { //permanent fault need to look at AOT as a fault
        if (PFC_Control.Flag){          
          DPC_LPCNTRL_DrivingOutputStop();
          Current_Control.BoostSwitchCh1 = RESET;
          Current_Control.BoostSwitchCh2 = RESET;
          Current_Control.BoostSwitchCh3 = RESET; 
          Current_Control.RectSwitchCh1 = RESET;
          Current_Control.RectSwitchCh2 = RESET;
          Current_Control.RectSwitchCh3 = RESET;
          Current_Control.CurrentControl = DISABLE;
          DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
          Current_Control.DutySoftStart = IDLE;
          PFC_VoltageControl.VoltageControl = DISABLE;
          PFC_VoltageControl.wIpkPI = 0;
          PFC_VoltageControl.SoftStartup = IDLE;
          PFC_VoltageControl.BurstRising = IDLE;
          Current_Control.FrequencyCheck = IDLE;
          PFC_VoltageControl.uhIpkIoutFF = 0;
          PFC_VoltageControl.wIpk = 0;
          PFC_VoltageControl.PFC_OK = RESET;
          Current_Control.ZvdFrequencyFollower = DISABLE;
          Current_Control.ZvdFilter = DISABLE;
          HAL_TIM_Base_Stop_IT(&htim3);
          HAL_NVIC_DisableIRQ(ZVD_VACsign_F_EXTI_IRQn);
          __HAL_GPIO_EXTI_CLEAR_FLAG(ZVD_VACsign_F_Pin);
          FAN_OFF;
          LED_GREEN_OFF;
          LED_YELLOW_OFF;
          LED_RED_ON;
          PFC_Control.Flag = RESET;
          PFC_VoltageControl.PFC_OK = RESET; 
    }
        PFC_Control.Flag = SET;
        PFC_Control.ubS = FAULT;
        DPC_FSM_State_Set(DPC_FSM_FAULT);
  }
  else{ //recovery from error
      if  (ProtectionDetect_status==ERROR_AC_PRESWITCH )
          {PFC_Control.ConversionMode = DPC_INVERTER_MODE;
            MAINS_SW_OFF;
          }
    
      if (PFC_Control.Flag){
          DPC_LPCNTRL_DrivingOutputStop();
          Current_Control.BoostSwitchCh1 = RESET;
          Current_Control.BoostSwitchCh2 = RESET;
          Current_Control.BoostSwitchCh3 = RESET; 
          Current_Control.RectSwitchCh1 = RESET;
          Current_Control.RectSwitchCh2 = RESET;
          Current_Control.RectSwitchCh3 = RESET;
          Current_Control.CurrentControl = DISABLE;
          DCP_MTH_SlopeLimiterReset(&Current_Control.IrefLimited);
          Current_Control.DutySoftStart = IDLE;
          PFC_VoltageControl.VoltageControl = DISABLE;
          PFC_VoltageControl.wIpkPI = 0;
          PFC_VoltageControl.SoftStartup = IDLE;
          PFC_VoltageControl.BurstRising = IDLE;
          Current_Control.FrequencyCheck = IDLE;
          PFC_VoltageControl.uhIpkIoutFF = 0;
          PFC_VoltageControl.wIpk = 0;
          PFC_VoltageControl.PFC_OK = RESET;
          LED_GREEN_OFF;
          LED_YELLOW_ON;      
          PFC_Control.ConversionStart = DISABLE;      
          PFC_Control.Flag = RESET;
          PFC_Control.ubRunState = 0;
   }
    //TOODO recovery here
    PFC_Control.Flag = SET;
    PFC_Control.ubS = FREQUENCY_DETECT;
    DPC_FLT_Error_Reset(ERROR_ALL);
    DPC_FSM_State_Set(DPC_FSM_IDLE); //Frequency detect and restart 
  
 }
 
 return RetVal;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  
  
//********** VOLTAGE CONTROL INTERRUPT BEGIN **********//-----------------------
  if (htim->Instance == TIM2){
// TEST_2_ON;
   TimeoutMng();
   
   //*** ADC data acquisition BEGIN***//________________________________________
   DPC_CMNDAT_GetDataAllSet((uint16_t*)&Data_Set1, (uint16_t*)&Data_Set2, (uint16_t*)&Data_Set3, &Data_adc, DPC_ADC_MODE);      
   //*** ADC data acquisition END***//__________________________________________
 //TODO look at the input voltage prior switch
   
  //*** Vinp differential measurement START ***//_______________________________
  Data_adc.uhVin = (uint16_t)fabsf((float)(Data_adc.uhVinL1 - Data_adc.uhVinL2));
  //*** Vinp differential measurement END ***//_________________________________
  
  //*** Iload zero-offset compensation BEGIN ***//______________________________     
//  if (Data_adc.uhIout < DPC_IZERO_ILOAD) {
//    Data_adc.uhIout = 0; //UPDATE: sign change for bidirectionality
//  }
//  else{
//    Data_adc.uhIout = Data_adc.uhIout - DPC_IZERO_ILOAD;
//  }

  
//  if (Data_adc.uhIout < (uint16_t)((int32_t)Current_Control.uwIzeroOffsetIdc + Current_Control.wDeltazeroOffsetIdc)) {
//    Data_adc.uhIout = 0;
//  }
//  else{
   Data_adc.uhIout = (uint16_t)fabsf((float)Data_Set1.uhBufferADC2[IOUT_RANK_ID] - (float)Current_Control.uwIzeroOffsetIdc - (float)Current_Control.wDeltazeroOffsetIdc);
//  }  
  
//  if (Data_Set1.uhBufferADC2[IOUT_RANK_ID] < (uint16_t)((int32_t)Current_Control.uwIzeroOffsetIdc)) {
//    Data_adc.uhIout = 0;
//  }
//  else{
//   Data_adc.uhIout = (uint16_t)((int32_t)Data_Set1.uhBufferADC2[IOUT_RANK_ID] - (int32_t)Current_Control.uwIzeroOffsetIdc);
//  }  
  //*** Iload zero-offset compensation END ***//________________________________   
  
  
  

  //*** Data averaging/filtering BEGIN ***//____________________________________
//  Control_Data.uhAvgIL1 = DCP_MTH_Average(Data_adc.uhIL1, &Data_Avg_IL1_avg_PFC);  
//  Control_Data.uhAvgIL2 = DCP_MTH_Average(Data_adc.uhIL2, &Data_Avg_IL2_avg_PFC);
//  Control_Data.uhAvgIL3 = DCP_MTH_Average(Data_adc.uhIL3, &Data_Avg_IL3_avg_PFC);  
  Control_Data.uhAvgVout = DCP_MTH_Average3Stages(Data_adc.uhVout, &Data_Avg_Vout_PFC);  
  Control_Data.uhAvgIout = DCP_MTH_Average4Stages(Data_adc.uhIout, &Data_Avg_Iout_PFC);  
  Control_Data.uhAvgTemp = DCP_MTH_Average(Data_adc.uhTemp, &Data_Avg_Temp_PFC);  
  Control_Data.uhVinRms = DCP_MTH_Average3Stages(Data_adc.uhVin, &Data_Avg_Vin_PFC);
  Control_Data.uhMovAvgIout = DCP_MTH_MovingAverage(Data_adc.uhIout, &Data_MovAvg_Iout_PFC);  
  //*** Data averaging/filtering END ***//______________________________________      

  
   //*** Vbus (dc) conversion in volt BEGIN ***//____________________________________
//  Control_Data.AvgVoutVolt = (float)(((uint32_t)Control_Data.uhAvgVout * DPC_VOUT_GAIN_DIV) / DPC_AVOUT); //Vbus conversion in volt
  Control_Data.AvgVoutVolt = (float)((float)Control_Data.uhAvgVout / (float)Control_Data.VdcGain); //Vbus conversion in volt  
   //*** Vbus (dc) conversion in volt END ***//______________________________________  
  
  
   //*** Vinp (ac) conversion in volt BEGIN ***//____________________________________
  Control_Data.uhVinRmsVolt = (uint16_t)(((uint32_t)Control_Data.uhVinRms * Control_Data.uhVinRmsFactor)/1024); //vinp conversion in volt
  Control_Data.uhVinPreSwitchRms = (uint16_t)(((uint32_t)Control_Data.uhVinRms * Control_Data.uhVinRmsFactor)/1024); //vinp conversion in volt
   //*** Vinp (ac) conversion in volt END ***//______________________________________


   //*** Vinp low voltage check BEGIN ***//_____________________________________  
  if (Control_Data.uhVinRmsVolt < Control_Data.uhVinRmsMin) {
    if (PFC_Protection.ubInputLowVoltageTimerCount < DPC_VIN_LOW_VOLTAGE_RESET_COUNT) PFC_Protection.ubInputLowVoltageTimerCount++;
  }
  else {
    PFC_Protection.ubInputLowVoltageTimerCount= 0;
  }
   //*** Vinp low voltage check END ***//_______________________________________
   
  
//  *** Phase shedding control BEGIN ***//______________________________________
  DPC_LPCNTRL_PhaseSheddingUpdate(&PFC_PhaseShedding, Control_Data.uhVinRmsVolt);
  DPC_LPCNTRL_PhaseShedding(&PFC_PhaseShedding, &Control_Data, &PFC_VoltageControl, &PFC_Control);
//  *** Phase shedding control END ***//________________________________________
 

//    //*** Power limitation for Vin > Vin_der BEGIN ***//
//    if (Control_Data.uhVinRmsVolt > DPC_VIN_MIN_DER){
////    PFC_VoltageControl.uwIpkMax = (uint32_t)(DPC_IPK_MAX * DPC_VIN_MIN_DER) / Control_Data.uhVinRmsVolt;  //Current limit update for voltage-loop= vin_min*ipk_max/vinp
//    PFC_VoltageControl.uwIpkMax = (uint32_t)(DPC_IPK_MAX * DPC_VIN_MIN_DER) / Control_Data.uhVinRmsVolt;  //Current limit update for voltage-loop= vin_min*ipk_max/vinp
//    } 
//    //*** Power limitation for Vin > Vin_der END ***//

    
    //*** Fan control BEGIN ***//
    if (PFC_Fan.FanEnable){
      //FAN 1 CONTROL
        if ((Control_Data.uhAvgIout >= (uint32_t)PFC_Fan.uhIoutFan) && (PFC_Fan.FanState1 == GPIO_PIN_RESET)){
          FAN_ON;
        }
        else if ((Control_Data.uhAvgIout < (uint32_t)PFC_Fan.uhIoutFan) && (PFC_Fan.FanState1 == GPIO_PIN_SET)){
          FAN_OFF;
        }
    }
    //*** Fan control END ***//  

   
    //*** Temperature conversion in �C BEGIN ***// 
    Control_Data.uhAvgTempDegrees = DPC_LPCNTRL_TemperatureGet(Data_adc.uhTemp); 
    //*** Temperature conversion in �C END ***// 
    
        // =============================================================================
        // Line frequency calculation in Hz
         if (PFC_Control.ubS > FREQUENCY_DETECT){
            Current_Control.ubLineFrequencyPreviousValue = Current_Control.ubLineFrequencyValue;
            Current_Control.ubLineFrequencyValue = ((uint32_t)DPC_LINE_FREQ_NOM * Current_Control.uhZvdTimerCounterDefault)/Current_Control.uhZvdTimerCounter;
        // =============================================================================    
         }
    
    //*** Converter status update BEGIN ***//
    DPC_LPCNTRL_ConverterStatusUpdate(&PFC_Led, &PFC_Relay, &Mains_SW_Relay, &PFC_Fan);
    //*** Converter status update END ***//

    
    //*** Load feed-forward parameters update BEGIN ***//
    if (PFC_Control.ubS > FREQUENCY_DETECT){
       DCP_AVGCC_FeedForwardUpdate(&Load_FF, &Current_Control);
    }
    //*** Load feed-forward parameters update END ***//

    
    //*** Vbus DC value update BEGIN ***//
    DPC_LPCNTRL_BusVoltageUpdate(&Control_Data, &Vout_PFC_Ramp, &PFC_VoltageControl, &PFC_Control);
    //*** Vbus DC value update END ***// 

    //*** Ipk AC value update BEGIN ***//
    DPC_LPCNTRL_ACpeakCurrentUpdate(&Current_Control, &Control_Data, &PFC_Control);
    //*** Ipk AC value update END ***//    

    //*** Ipk AC phase update BEGIN ***//
    DPC_LPCNTRL_ACphaseCurrentUpdate(&Control_Data);
    //*** Ipk AC phase update END ***// 

    

    
  //*** Voltage control loop BEGIN ***//________________________________________  
  if (PFC_VoltageControl.VoltageControl == ENABLE){

   //*** Vout Soft Start-up BEGIN ***//_________________________________________ 
    if (PFC_VoltageControl.SoftStartup == START){
      Vout_PI_t.wOut=0; //reset PI output
      Vout_PI_t.wIntegral=0; //reset PI integral
      PFC_VoltageControl.uhIpkIoutFF = 0;
      PFC_VoltageControl.SoftStartup = RUNNING;
      Control_Data.uwVoutSetPoint = Control_Data.uhAvgVout;
      if(DCP_MTH_RampInit((int32_t)Control_Data.uhAvgVout, (int32_t)Control_Data.uhVoutRef, DPC_SOFTSTARTUP_DURATION, &Vout_PFC_Ramp)){
//        eb1= 30; //error managment TBD
        }       
    }    
    if (PFC_VoltageControl.SoftStartup == RUNNING){    
        if (Control_Data.uwVoutSetPoint < Control_Data.uhVoutRef) {
          Control_Data.uwVoutSetPoint = (uint32_t)DCP_MTH_RampGeneration(&Vout_PFC_Ramp);
        }
        else {
          Control_Data.uwVoutSetPoint = Control_Data.uhVoutRef;
          PFC_VoltageControl.SoftStartup = COMPLETE;
          PFC_VoltageControl.SoftStartup = IDLE;
          LED_RED_OFF;
          LED_YELLOW_OFF;
          LED_GREEN_ON;
          PFC_VoltageControl.PFC_OK = SET;
          }
    }//SoftStartup=RUNNING
   //*** Vout Soft Start-up END ***//___________________________________________   

    
    if (PFC_VoltageControl.SoftStartup == IDLE){
      // Vout value update BEGIN //
      Control_Data.uwVoutSetPoint = Control_Data.uhVoutRef;
//      Control_Data.uhVoutBurstMax = (uint16_t)((float)Control_Data.uhVoutRef * (float)1.0769);
//      Control_Data.uhVoutBurstMin = (uint16_t)((float)Control_Data.uhVoutRef * (float)1.0256);
      // Vout value update END //
    } 
    
    if ((Control_Data.uhAvgVout >= Control_Data.uhVoutRef) && (PFC_VoltageControl.PFC_OK == RESET)){
      LED_RED_OFF;
      LED_YELLOW_OFF;
      LED_GREEN_ON;
      PFC_VoltageControl.PFC_OK = SET;
    }


   //*** Vout PI dynamic limits update BEGIN ***//______________________________
   DPC_PIs32_LimitsUpdate(&Vout_PI_t, &PFC_VoltageControl); 
   //*** Vout PI dynamic limits update END ***//________________________________
   
   
   //*** Vout PI calculation BEGIN ***//________________________________________   
   PFC_VoltageControl.wIpkPI = DPC_PIs32Type1((int32_t)Control_Data.uwVoutSetPoint, (int32_t)Control_Data.uhAvgVout , &Vout_PI_t);
   //*** Vout PI calculation END*** //__________________________________________
      
   }// if (PFC_VoltageControl.VoltageControl == ENABLE)
  //*** Voltage control loop END ***//________________________________________
  

   if (PFC_Control.ubS >= ILOAD_CHECK){
     
     if (PFC_Control.ConversionMode == DPC_PFC_MODE){
    
           //*** Load Feed-Forward BEGIN ***//__________________________________________   
              PFC_VoltageControl.uhIpkIoutFF = (uint16_t)DCP_AVGCC_FeedForwardControl(&Load_FF, (int32_t)Control_Data.uhAvgIout,
                                                        (int32_t)Control_Data.uhMovAvgIout,
                                                         &Vout_PI_t, &Control_Data);
              if (PFC_VoltageControl.LoadFFEnable){
                PFC_VoltageControl.wIpkTmp = (int16_t)PFC_VoltageControl.wIpkPI + PFC_VoltageControl.uhIpkIoutFF; //load feed-forward enabled
              }
              else{
                PFC_VoltageControl.wIpkTmp = (int16_t)PFC_VoltageControl.wIpkPI; //load feed-forward disabled
              }
           //*** Load Feed-Forward END ***//____________________________________________  
         
              
           //*** Input Voltage Feed-Forward BEGIN ***//_________________________________
             PFC_VoltageControl.uhVinKff = DPC_AVGCC_InputVoltageFeedForward(&Input_FF, Control_Data.uhVinRmsVolt, PFC_VoltageControl.wIpkTmp);

             
              if (PFC_VoltageControl.VinFFEnable){
                PFC_VoltageControl.wIpkTmp = (PFC_VoltageControl.wIpkTmp*PFC_VoltageControl.uhVinKff) >> Input_FF.ubFFShiftExp; //Input voltage feed-forward enabled
              }
           //*** Input Voltage Feed-Forward END ***//___________________________________      

      
     }//if (PFC_Control.ConversionMode == DPC_PFC_MODE)
             
     else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE){
         
              //*** Iac Soft Start-up/Update BEGIN ***//_________________________________________ 
                  if (Current_Control.IacPkSoftStart == START){
                    Current_Control.IacPkSoftStart = RUNNING;
                    PFC_VoltageControl.wIpkTmp = 0;
                    if(DCP_MTH_RampInit(0, Control_Data.uhIacRefInverter , DPC_IAC_SOFTSTARTUP_UPDATE_DURATION, &Current_Control.Iac_Ramp)){
              //        eb1= 30; //error managment TBD
                      }       
                  }    
                  if (Current_Control.IacPkSoftStart == RUNNING){    
                      if (PFC_VoltageControl.wIpkTmp < Control_Data.uhIacRefInverter) {
                        PFC_VoltageControl.wIpkTmp = (uint32_t)DCP_MTH_RampGeneration(&Current_Control.Iac_Ramp);
                      }
                      else {
                        PFC_VoltageControl.wIpkTmp = Control_Data.uhIacRefInverter;
                        Current_Control.IacPkSoftStart = COMPLETE;
                        Current_Control.IacPkSoftStart = IDLE;
                        LED_RED_OFF;
                        LED_YELLOW_OFF;
                        LED_GREEN_ON;
              //          LED_BLUE_ON;
              //          Current_Control.PFC_OK = SET;
                        }
                  }//SoftStartup=RUNNING
                 //*** Iac Soft Start-up/Update END ***//___________________________________________
                  
                   if ((Current_Control.IacPkSoftStart == IDLE) && (PFC_Control.ubS == RUN_INVERTER_MODE)){
                    // Iac value update BEGIN //
                    PFC_VoltageControl.wIpkTmp = Control_Data.uhIacRefInverter;
                    LED_RED_OFF;
                    LED_YELLOW_OFF;
                    LED_GREEN_ON;
                    // Iac value update END //
                  }             
    
    
    
     }//else if (PFC_Control.ConversionMode == DPC_INVERTER_MODE)
      
      
      //*** Voltage control --> output saturation BEGIN ***//
      if (PFC_VoltageControl.wIpkTmp <= 0) PFC_VoltageControl.wIpkTmp = 0;
//      if (PFC_VoltageControl.wIpkTmp >= PFC_VoltageControl.uwIpkMax) PFC_VoltageControl.wIpkTmp = PFC_VoltageControl.uwIpkMax;
      if (PFC_VoltageControl.wIpkTmp >= PFC_VoltageControl.wIpkMax) PFC_VoltageControl.wIpkTmp = PFC_VoltageControl.wIpkMax;
      if ((PFC_Control.ubS == RUN_PFC_MODE) || (PFC_Control.ubS == RUN_INVERTER_MODE)) PFC_VoltageControl.wIpk = PFC_VoltageControl.wIpkTmp;
      //*** Voltage control --> output saturation END ***//
      
    }// if (PFC_Control.ubS >= ILOAD_CHECK)
//TEST_2_OFF; //TVloop=23-24us with High-Speed OPT
  }//if (htim->Instance == TIM2)
//********** VOLTAGE CONTROL INTERRUPT END **********//-------------------------
 
  
  
  
//********** MULTIPLIER INTERRUPT BEGIN**********//----------------------------- 
  if (htim->Instance == TIM3){
//TEST_1_ON;
  Data_adc.uhIinFlt = Data_Set1.uhBufferADC2[I_IN_FLT_RANK_ID];
  
  Data_adc.uhIL1 = Data_Set1.uhBufferADC2[IL1_RANK_ID];
  Control_Data.uhAvgIL1 = Data_adc.uhIL1;
  
  Data_adc.uhIL2 = Data_Set1.uhBufferADC2[IL2_RANK_ID];
  Control_Data.uhAvgIL2 = Data_adc.uhIL2;
  
  Data_adc.uhIL3 = Data_Set1.uhBufferADC2[IL3_RANK_ID];
  Control_Data.uhAvgIL3 = Data_adc.uhIL3;

  if ((PFC_Control.ubS == FREQUENCY_DETECT) && (Current_Control.FrequencyCheck == RUNNING)){
    ++Current_Control.uwLineFrequencyMeasureCounter; 
   }
   else{
  if (PFC_Control.ubS == BURST_MODE){
    if (PFC_VoltageControl.BurstRising == START){
      PFC_VoltageControl.wIpk = (int32_t)Current_Control.uwIpkRefBurst; //*** No-Load condition --> Voltage loop disabled (constant current, open-loop)***//     
      PFC_VoltageControl.BurstRising = RUNNING;
    }     
  }// if (PFC_Control.ubS == PFC_BURST)

    //*** Average Current Control Loop Activation***// 
    if ((PFC_VoltageControl.BurstRising == RUNNING) || (PFC_Control.ubS == RUN_PFC_MODE) || (PFC_Control.ubS == RUN_INVERTER_MODE)){
         DPC_AVGCC_AvgCurrentControlActivation(&Current_Control, &LUT_Tables, &PFC_PhaseShedding, &Data_adc, &Data_Set1, &Data_Set2, &Control_Data, &PFC_Control, (int32_t)PFC_VoltageControl.wIpk);         
    }


  //*** Current Control Loop Mains-Synchronization ***//    
  DPC_AVGCC_AvgCurrentControlSynch(&Current_Control);
  }//else
// TEST_1_OFF; //Tmul=10us with High-Speed OPT
}//if (htim->Instance == TIM3)

//********** MULTIPLIER INTERRUPT END**********//-------------------------------



  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}  

//----------------------------------------------------------------------------//





//********** ZVD INTERRUPT BEGIN **********//-----------------------------------  
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
   
  
  if(GPIO_Pin == ZVD_VACsign_F_Pin){
    
    DPC_AVGCC_ZvdProtectionControl(&Current_Control, DPC_ZVD_RELOAD);
     
    if ((PFC_Control.ubS == CONVERTER_INRUSH) && (PFC_Relay.RelayInrushState == RUNNING)) {
      PFC_Relay.RelayInrushState = COMPLETE;
      DPC_TO_Set(DPC_TO_RELAY_COMP, DPC_TO_RELAY_COMP_TICK);
    }
    else if ((PFC_Control.ubS == FREQUENCY_DETECT) && (Current_Control.FrequencyCheck == START)){
      Current_Control.uwLineFrequencyMeasureCounter = 0;
      DPC_LPCNTRL_MulControlTimingMeasurementSet();
      Current_Control.FrequencyCheck = RUNNING;
    } 
    else {
      if ((PFC_Control.ubS == FREQUENCY_DETECT) && (Current_Control.FrequencyCheck == RUNNING)){
        Current_Control.FrequencyCheck = COMPLETE;
        Current_Control.ubLineFrequencyMeasure = (uint8_t)((uint32_t)(DPC_LINE_FREQ_DEFAULT * DPC_LINE_FREQ_COUNTER)/Current_Control.uwLineFrequencyMeasureCounter); //fx(Hz)=(50(Hz)-0.5 tolerance)*Freq_count_50Hz(1000)/Freq_count_x
        DPC_LPCNTRL_MulControlTimingDefaultSet();
     }      
    DPC_AVGCC_ZvdFollowerControl(&Current_Control, DPC_ZVD_FOLLOWER_CHECK); 
   }//else
 
 }//if(GPIO_Pin == ZVD_VACsign_F_Pin)
  
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
//********** ZVD INTERRUPT END **********//-------------------------------------   




//********** OCP INTERRUPT BEGIN **********//-----------------------------------

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
//  TEST_1_ON;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcomp);
  
  PFC_Protection.uwCh1Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_MASTER_TIMER_INDEX, MASTER_OUTPUT_LS);
  PFC_Protection.uwCh1Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_MASTER_TIMER_INDEX, MASTER_OUTPUT_HS);

  PFC_Protection.uwCh2Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE1_TIMER_INDEX, SLAVE1_OUTPUT_LS);
  PFC_Protection.uwCh2Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE1_TIMER_INDEX, SLAVE1_OUTPUT_HS);

  PFC_Protection.uwCh3Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE2_TIMER_INDEX, SLAVE2_OUTPUT_LS);
  PFC_Protection.uwCh3Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE2_TIMER_INDEX, SLAVE2_OUTPUT_HS);  
  
  
  
  if (hcomp->Instance == COMP1){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh1Low = SET;
    
    }//if (hcomp->Instance == COMP1)
  
  if (hcomp->Instance == COMP2){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh1High = SET;
    
    }//if (hcomp->Instance == COMP2)
  
  if (hcomp->Instance == COMP3){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh2Low = SET;
    
    }//if (hcomp->Instance == COMP3)  

  if (hcomp->Instance == COMP4){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh2High = SET;
    
    }//if (hcomp->Instance == COMP4)  

  if (hcomp->Instance == COMP5){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh3Low = SET;
    
    }//if (hcomp->Instance == COMP5)  
  
  if (hcomp->Instance == COMP6){
    
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, MASTER_OUTPUT_LS + MASTER_OUTPUT_HS + SLAVE1_OUTPUT_LS + SLAVE1_OUTPUT_HS + SLAVE2_OUTPUT_LS + SLAVE2_OUTPUT_HS);
             LF_HS_OFF;
             LF_LS_OFF;            
           Current_Control.StateCh1 = RESET;
           Current_Control.StateCh2 = RESET;
           Current_Control.StateCh3 = RESET;

   PFC_Protection.OverCurrentTrigger = SET;
   PFC_Protection.OCPEventCh3High = SET;
    
    }//if (hcomp->Instance == COMP6)   
  
  
//  PFC_Protection.uwCh1Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_MASTER_TIMER_INDEX, MASTER_OUTPUT_LS);
//  PFC_Protection.uwCh1Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_MASTER_TIMER_INDEX, MASTER_OUTPUT_HS);
//
//  PFC_Protection.uwCh2Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE1_TIMER_INDEX, SLAVE1_OUTPUT_LS);
//  PFC_Protection.uwCh2Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE1_TIMER_INDEX, SLAVE1_OUTPUT_HS);
//
//  PFC_Protection.uwCh3Out1State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE2_TIMER_INDEX, SLAVE2_OUTPUT_LS);
//  PFC_Protection.uwCh3Out2State = HAL_HRTIM_WaveformGetOutputState(&hhrtim1, HAL_SLAVE2_TIMER_INDEX, SLAVE2_OUTPUT_HS);  

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_COMP_TriggerCallback should be implemented in the user file
   */
//  TEST_1_OFF;
}





//void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hcomp);
//  
//  if (hcomp->Instance == COMP7){
//    
//    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2 + HRTIM_OUTPUT_TB1 + HRTIM_OUTPUT_TB2 + HRTIM_OUTPUT_TC1 + HRTIM_OUTPUT_TC2);
//             LF_HS_OFF;
//             LF_LS_OFF;            
//           Current_Control.StateCh1 = RESET;
//           Current_Control.StateCh2 = RESET;
//           Current_Control.StateCh3 = RESET;
//
//   PFC_Protection.OverCurrentTrigger = SET;
//    
//    }//if (hcomp->Instance == COMP7)
//
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_COMP_TriggerCallback should be implemented in the user file
//   */
//}
//********** OCP INTERRUPT END **********//-------------------------------------



/* USER CODE END 4 */




/**
  * @brief  Application Protection Detect in RUN status
  * @param  None
  * @retval Voltage status
  */
DPC_FAULTERROR_LIST_TypeDef DPC_ProtectionDetect(void)
{ 
DPC_FAULTERROR_LIST_TypeDef RetVal = NO_FAULT;

  if (PFC_Protection.OverCurrentTrigger){ //Inductor Over Current Protection
     RetVal |= FAULT_OCS;
     PFC_Protection.ubErrorCode = 11;
   }
    
  if (Current_Control.CurrentSensorCalibration == ERROR) {
    RetVal |= FAULT_CURR_CALIB;
    PFC_Protection.ubErrorCode = 9;
  }  
    
  if ((PFC_Protection.OverTemperatureProtection)&&(Control_Data.uhAvgTempDegrees >= (int16_t)DPC_OTSHUTDOWN)){ //Ambient Over Temperature Protection
     RetVal |= FAULT_AOT;
     PFC_Protection.ubErrorCode = 5;
   }
  
  if ((Current_Control.ubLineFrequencyValue > DPC_LINE_FREQ_MAX) || (Current_Control.ubLineFrequencyValue < DPC_LINE_FREQ_MIN)){ //AC line frequency out of range
    RetVal |= FAULT_ACF_OR;
    PFC_Protection.ubErrorCode = 7;
  }  

  if ((Control_Data.uhAvgVout > Control_Data.uhVoutOverVoltageValue)){ //Vbus OverVoltage Protection
     RetVal |= FAULT_OVL;
     PFC_Protection.ubErrorCode =1;
  }  
  else{ 
    if ( ((PFC_Control.ConversionMode == DPC_PFC_MODE) && (Control_Data.uhAvgVout < Control_Data.uhVbusUnderVoltageValuePFC) && (PFC_VoltageControl.PFC_OK == SET))
      || ((PFC_Control.ConversionMode == DPC_INVERTER_MODE) && (Control_Data.uhAvgVout < Control_Data.uhVbusUnderVoltageValueInverter)) )
    { //Vbus DC UnderVoltage Protection
          RetVal |= ERROR_DC_UV;
       PFC_Protection.ubErrorCode =2; // this is where we look and crash the Inverter.. wait for the voltage to recover 
      
       
    }
  }
  
  if((PFC_Control.ConversionMode=DPC_INVERTER_MODE)&& (Control_Data.uhVinPreSwitchRms > Control_Data.uhVinRmsMin))
  {
    
    RetVal|=ERROR_AC_PRESWITCH;
    PFC_Protection.ubErrorCode =12;
    //todo next state is start this is time to change modes and make hte pfc run

  }
  
  
  if ((Control_Data.uhVinRmsVolt > Control_Data.uhVinRmsMax) || (Control_Data.uhVinPreSwitchRms > Control_Data.uhVinRmsMax)){ //Vac OverVoltage Protection
     RetVal |= ERROR_AC_OV;
     PFC_Protection.ubErrorCode =3;
     
     
    

    }
  else{   
      if(PFC_Protection.ubInputLowVoltageTimerCount >= DPC_VIN_LOW_VOLTAGE_PROT_COUNT){ //Vac UnderVoltage Protection
        RetVal |= ERROR_AC_UV;
        PFC_Protection.ubErrorCode =4;//todo this is a problem on both sides.
      }
  }
  
  if ((Current_Control.ZvdControlStatus) && (PFC_Control.ubS >= RUN_PFC_MODE) && (PFC_Control.ubS <= BURST_MODE)) { //AC zero crossing missing
    RetVal |= ERROR_AC_ZVD;
    PFC_Protection.ubErrorCode = 6;
  }
  
  if (PFC_Control.ConversionMode == DPC_NONE_MODE) {
    RetVal |= FAULT_NONE_MODE;
    PFC_Protection.ubErrorCode = 8;
  }  
  
return RetVal;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//  TEST_1_ON;
  Vbus_ref_rx=(uint16_t)round((double)rx_buff[1]*(double)rx_buff[2]);
//  HAL_UART_Receive_IT(&huart1,rx_buff,3);
  HAL_UART_Receive_IT(&huart4,rx_buff,3);
  
}


/* USER CODE END 4 */
