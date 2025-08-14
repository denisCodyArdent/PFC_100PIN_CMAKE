/**
  ******************************************************************************
  * @file    DPC_CommonData.h
  * @brief   This file contains the headers of the Database.
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

#ifndef __DPC_COMMONDATA_H
#define __DPC_COMMONDATA_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif


#include "DPC_Application_Conf.h"
#include "DPC_Lib_Conf.h"
#include "stdbool.h"
#include "math.h"



/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/****** DPC_Loopctrl.h section Start *****/
typedef enum {
  
  IDLE = 0,
  INIT,
  START,
  RUNNING,
  COMPLETE,
  
} DPC_LPCNTRL_SubStateMachine_t;



typedef enum {
  
  DPC_NONE_MODE = 0,
  DPC_PFC_MODE,
  DPC_INVERTER_MODE,
  
} DPC_LPCNTRL_ConversionMode_t;



typedef struct{

  DPC_LPCNTRL_SubStateMachine_t BurstRising;
  DPC_LPCNTRL_SubStateMachine_t SoftStartup;
  DPC_LPCNTRL_SubStateMachine_t BusVoltageUpdate;
  FunctionalState VoltageControl;
  FlagStatus PFC_OK;

  int32_t wVoutControlEnhanceUpperTh1; // 1st Vout upper threshold for Dynamic Control Enhance
  int32_t wVoutControlEnhanceUpperTh2; // 2nd Vout upper threshold for Dynamic Control Enhance
  int32_t wVoutControlEnhanceLowerTh1; // 1st Vout lower threshold for Dynamic Control Enhance
  int32_t wVoutControlEnhanceLowerTh2; // 2nd Vout lower threshold for Dynamic Control Enhance
   
  int32_t wIpkPI; //Peak reference coming from Vout digital PI controller 
  int32_t wIpk;   //Peak reference coming from Vout Voltage loop
//  int32_t wIpkNewValue;   //New Peak reference value for Iac peak update in INVERTER MODE
  int32_t wIpkTmp;//Temporary Peak reference coming from Vout Voltage loop (avoid error due highest priority interrupt)
  uint16_t uhVinKff; //Peak reference coming from Input voltage feed-forward
//  uint32_t uwIpkMax; //Maximum Peak reference coming from Vout Voltage loop

  int32_t wIpkMax; //Maximum Peak reference for current control
  uint32_t uwIpkMin; //Minimum Peak reference for current control
  uint32_t uwIpkBurst; //Burst mode Peak reference for current control
 
  uint16_t uhIpkIoutFF; //Peak reference coming from Load feed-forward
  FunctionalState LoadFFEnable; //Load feed-forward enable
  FunctionalState VinFFEnable; //Vin feed-forward enable

}DPC_LPCNTRL_VoltageControl_t;


typedef struct{
  
  //*** Phase shedding control BEGIN ***//
  
  uint8_t ubNumOfActiveChannels;
  
  //dc side current thresholds
  uint16_t uhCurrentTh2to1_dc_Init;
  uint16_t uhCurrentTh1to2_dc_Init;
  uint16_t uhCurrentTh3to2_dc_Init;
  uint16_t uhCurrentTh2to3_dc_Init;

  uint16_t uhCurrentTh2to1_dc;
  uint16_t uhCurrentTh1to2_dc;
  uint16_t uhCurrentTh3to2_dc;
  uint16_t uhCurrentTh2to3_dc;

  
  //ac side current thresholds
  uint16_t uhCurrentTh2to1_ac_Init;
  uint16_t uhCurrentTh1to2_ac_Init;
  uint16_t uhCurrentTh3to2_ac_Init;
  uint16_t uhCurrentTh2to3_ac_Init;  
  
  uint16_t uhCurrentTh2to1_ac;
  uint16_t uhCurrentTh1to2_ac;
  uint16_t uhCurrentTh3to2_ac;
  uint16_t uhCurrentTh2to3_ac;   
  //*** Phase shedding control END ***//  

}DPC_LPCNTRL_PhaseShedding_t;


typedef struct{

  FunctionalState OverTemperatureProtection;
  FunctionalState OverCurrentProtection;  
  FlagStatus OverCurrentTrigger;
  FlagStatus OCPEventCh1High;
  FlagStatus OCPEventCh1Low;
  FlagStatus OCPEventCh2High;
  FlagStatus OCPEventCh2Low;
  FlagStatus OCPEventCh3High;
  FlagStatus OCPEventCh3Low;  
  uint8_t ubInputLowVoltageTimerCount;
  uint32_t uwOCPHighLevelCh1; //OCP Ch1 high threshold value
  uint32_t uwOCPLowLevelCh1; //OCP Ch1 low threshold value
  uint32_t uwOCPHighLevelCh2; //OCP Ch2 high threshold value
  uint32_t uwOCPLowLevelCh2; //OCP Ch2 low threshold value
  uint32_t uwOCPHighLevelCh3; //OCP Ch3 high threshold value
  uint32_t uwOCPLowLevelCh3; //OCP Ch3 low threshold value
  
  uint32_t uwCh1Out1State;
  uint32_t uwCh1Out2State;
  
  uint32_t uwCh2Out1State;
  uint32_t uwCh2Out2State;
  
  uint32_t uwCh3Out1State;
  uint32_t uwCh3Out2State;
  
  uint8_t ubErrorCode;

}DPC_LPCNTRL_Protection_t;



typedef struct{
  
  GPIO_PinState Relay;
  FunctionalState RelayControl;
  DPC_LPCNTRL_SubStateMachine_t RelayInrushState;

}DPC_LPCNTRL_Inrush_t;


typedef struct{

  FunctionalState ConversionStart; // Enable for Conversion start
  FlagStatus Flag;                 // Flag control for state machine
  uint8_t ubS;                     // Dummy state machine indicator
  uint8_t ubRunState;              // State machine indicator for converter run-state     
  DPC_LPCNTRL_ConversionMode_t ConversionMode;  // Set the conversion mode: DPC_PFC_MODE (ac-dc) or DPC_INVERTER_MODE (dc-ac)
  float VoutNewSetpoint_Volt;              // New value for Vbus update in [Volt] for DPC_PFC_MODE
  float IacRefInverterNewValue_Ampere;     //New value for Iac peak current reference uodate in [Ampere] for DPC_INVERTER_MODE

}DPC_LPCNTRL_ConverterControl_t;


typedef struct{
  
  GPIO_PinState Relay;
  FunctionalState RelayControl;
  DPC_LPCNTRL_SubStateMachine_t RelayInrushState;
  DPC_LPCNTRL_ConverterControl_t ConverterControl;

}DPC_LPCNTRL_MainsRly_t;

typedef struct{
  
  GPIO_PinState GreenLed;
  GPIO_PinState RedLed;
  GPIO_PinState YellowLed;
  GPIO_PinState BlueLed;

}DPC_LPCNTRL_Led_t;



typedef struct{

  bool FanEnable;
  GPIO_PinState FanState1;
  uint16_t uhIoutFan;    //Iload threshold for Fan activation

}DPC_LPCNTRL_Fan_t;



typedef struct{

  FunctionalState CurrentBalance;

}DPC_LPCNTRL_Feature_t;



/****** DPC_Loopctrl.h section End *****/


/****** DPC_LUT.h section Start *****/ 
typedef struct{

uint16_t uhaVinRmsSet[DPC_NUM_OF_VIN_SET];
double Vout;
double Pout;
double Eta;
double PowerFactor;
double w0;
double alpha;
double beta;
double L;
double Coss_tot;
double S_MCU;

/*** Peak Sine Reference LUT ***/
int16_t haIpkRefMax[DPC_LUT_POINTS];

/*** Normalized Sine LUT ***/
float SIN_NORM[DPC_LUT_POINTS];

}DPC_LUT_TTPPFC_t;



typedef struct{

  uint16_t uhErrorCodeIpkRefMaxLUT;
  uint16_t uhErrorCodeIRMin[DPC_NUM_OF_VIN_SET];
  uint16_t uhErrorCodeC2Min[DPC_NUM_OF_VIN_SET];
  uint16_t uhErrorCodeTzeroVdsMin[DPC_NUM_OF_VIN_SET];
  uint16_t uhErrorCodeTVoutVdsMin[DPC_NUM_OF_VIN_SET];
  uint16_t uhErrorCodeIFsenseComp[DPC_NUM_OF_VIN_SET];
  uint16_t uhErrorCodeIRsenseComp[DPC_NUM_OF_VIN_SET];

}DPC_LUT_ERROR_CHECK_t;




typedef struct{

uint16_t uhaDutyPositive[DPC_SOFT_DUTY_DURATION + 1];
uint16_t uhaDutyNegative[DPC_SOFT_DUTY_DURATION + 1];
uint16_t uhDutyIndex;
uint16_t uhFinalSoftDutyValue;
FlagStatus SoftDutyEnd;

}DPC_LUT_SoftDuty_t;
/****** DPC_LUT.h section End *****/




/****** DPC_Math.h section Start *****/
typedef struct{
  uint32_t uwAccVal;
  uint16_t uhAvgVal;
  uint16_t uhWeight;
}DPC_MTH_Average_t;

typedef struct{
  DPC_MTH_Average_t stage_1;
  DPC_MTH_Average_t stage_2; 
}DPC_MTH_Average2Stages_t;

typedef struct{
  DPC_MTH_Average_t stage_1;
  DPC_MTH_Average_t stage_2;
  DPC_MTH_Average_t stage_3; 
}DPC_MTH_Average3Stages_t;

typedef struct{
  DPC_MTH_Average_t stage_1;
  DPC_MTH_Average_t stage_2;
  DPC_MTH_Average_t stage_3;
  DPC_MTH_Average_t stage_4;  
}DPC_MTH_Average4Stages_t;

typedef struct{
  uint16_t uhaAvg[DPC_AVG_SAMPLE];
  uint32_t uwSumVal;
  uint16_t uhMovAvgVal;
  uint16_t uhNumSamples;
}DPC_MTH_MovingAverage_t;

typedef struct{
  int32_t wInitVal;
  int32_t wFinalVal;
  int32_t wAccumulator;
  int32_t wOutput;
  int32_t wStep;
  uint32_t uwDuration;
  uint8_t ubRampShiftExp;
  FlagStatus EndOfRamp;
}DPC_MTH_RampGenerator_t;

typedef struct{
  int32_t wInitVal;
  int32_t wFinalVal;
  int32_t wAccumulator;
  int32_t wOutput;
  int32_t uwStepIndex;
  double Base;
  double Exponent;
  double ExponentOfExponent;
  uint32_t uwDuration;
  uint8_t ubExpShiftExp;
  FlagStatus EndOfExp;
}DPC_MTH_ExpGenerator_t;



typedef struct{
  int32_t wPreviousValue;
  int32_t wOutputValue;
  int32_t wSlopeMax;
  int32_t wSlopeMin;
}DCP_MTH_SlopeLimiter_t;
/****** DPC_Math.h section End *****/


/****** DPC_Pid.h section Start *****/
typedef struct {

  FlagStatus resetPI;  
  FlagStatus satPI_toggle;
  FlagStatus antiwindPI_toggle;

  uint8_t ubPI_Q_FxP;
  int32_t wRef;  
  int32_t wFeed;   
  int32_t wIntegral;
  int32_t wOut;
  int32_t wOutSat;
  int32_t wOutUpperSat;
  int32_t wOutLowerSat;  
  int32_t wIntegralUpperSat;
  int32_t wIntegralLowerSat;  
  int32_t wError;
  int32_t wIntegralOut;  
  uint32_t uwK0;  
  uint32_t uwK1;
  int32_t wAntiwindupTerm;
  int32_t wAntiwindupGain;
  int32_t wResetValue;
  
}DPC_PID_PIs32_t;
/****** DPC_Pid.h section End *****/



/****** DPC_AvgeresisCurrentCtrl.h section Start *****/
typedef struct {

  uint8_t ubFFShiftExp; 
  uint8_t ubTrippingCnt;
  
  int32_t wStepUpParam; 
  int32_t wStepDownParam;
  int32_t wStepUpParamInit; 
  int32_t wStepDownParamInit;
  
  int32_t wUpperThreshold;  
  int32_t wLowerThreshold;
  int32_t wUpperThresholdParam;  
  int32_t wLowerThresholdParam;  

  
  int32_t wMovAvgValue;
  int32_t wAvgValue;
  
  int32_t wOutput;
  
}DPC_AVGCC_FeedForward_t;



typedef struct {
  
  uint8_t ubFFShiftExp;
  uint16_t uhVinNominal;
  uint16_t uhVinPrev;
  uint16_t uhVinDelta;
  uint16_t uhKffTmp;
  uint16_t uhKff;
  uint16_t uhKffMin;
  uint16_t uhKffMax;
  int32_t wIpkKff;
  int32_t wIKff;
  
}DPC_AVGCC_InputFeedForward_t;



typedef struct {
  
  uint8_t ubAvgShiftExp;
  
  int32_t wIpkRefMax;  
  int32_t wIpkRef;  
  uint32_t uwIpkRefMin;  
  uint32_t uwIpkRefBurst;
  
  int32_t wIthRefHMax; 
  int32_t wIthRefLMax;  
  int32_t wIthRefHMin;  
  int32_t wIthRefLMin;  
   
  int32_t wIRsenseCompSelected; 
  int32_t wIFsenseCompSelected;
  
  int32_t wIthRefH;  
  int32_t wIthRefL; 
  
  int32_t wDeltaIpkSelected;   
  int32_t wLowThCurrentSelected;   

  uint32_t uwIthRefH1;   
  uint32_t uwIthRefL1;   

  uint32_t uwIthRefH2; 
  uint32_t uwIthRefL2;  

  uint32_t uwIthRefH3;  
  uint32_t uwIthRefL3;  

  
  uint32_t uwRisingDeadTimeMax;  
  uint32_t uwFallingDeadTimeMax;  
  uint32_t uwRisingDeadTimeMin;  
  uint32_t uwFallingDeadTimeMin;  

  uint32_t uwRisingDeadTimeComp;  
  uint32_t uwFallingDeadTimeComp;    

  uint32_t uwRisingDeadTime;   
  uint32_t uwFallingDeadTime;  
  
  uint32_t uwRisingDeadTimeSelected; 
  uint32_t uwFallingDeadTimeSelected;  
  
  uint32_t uwRisingDeadTimeCh1; 
  uint32_t uwFallingDeadTimeCh1;    

  uint32_t uwRisingDeadTimeCh2;  
  uint32_t uwFallingDeadTimeCh2;    
  
  uint32_t uwRisingDeadTimeCh3;  
  uint32_t uwFallingDeadTimeCh3;

  
  
  uint8_t ubAvg_comp_Q_FxP;

  int32_t wIthCompH1; 
  int32_t wIthCompL1; 

  int32_t wIthCompH2;  
  int32_t wIthCompL2;  

  int32_t wIthCompH3; 
  int32_t wIthCompL3; 
  
  
  uint32_t uwIzeroOffsetIL;
  uint32_t uwIzeroOffsetIdc;
  int32_t wDeltazeroOffsetCh1;
  int32_t wDeltazeroOffsetCh2;
  int32_t wDeltazeroOffsetCh3;
  int32_t wDeltazeroOffsetTotal;
  int32_t wDeltazeroOffsetIdc;
  
  ErrorStatus CurrentSensorCh1;
  ErrorStatus CurrentSensorCh2;
  ErrorStatus CurrentSensorCh3;
  ErrorStatus CurrentSensorTotal;
  ErrorStatus CurrentSensorIout;
  ErrorStatus CurrentSensorCalibration;
  FlagStatus CurrentSensorCalibrationDone;
  
  uint16_t uhRunIndex;
  uint16_t uhSyncIndex;
  uint16_t uhResetIndexValue;
  uint16_t uhReSyncIndexValue;
  
  uint16_t uhZvdProtectionIndex;
  uint16_t uhZvdEnableIndexMin;
  uint16_t uhZvdEnableIndexMax;
  uint16_t uhZvdReloadIndexValue;
  uint16_t uhZvdFilterIndexMin;
  uint16_t uhZvdFilterIndexMax;
 
  uint16_t uhPolarityChangeIndex;
  
  uint16_t uhLowFreqPositiveTurnOnIndex;
  uint16_t uhLowFreqPositiveTurnOffIndex;
  uint16_t uhLowFreqNegativeTurnOnIndex;
  uint16_t uhLowFreqNegativeTurnOffIndex;
  uint16_t uhLowFreqPositiveStartedIndex;
  uint16_t uhLowFreqNegativeStartedIndex;



  uint16_t uhHighFreqPositiveTurnOnIndex;
  uint16_t uhHighFreqPositiveTurnOffIndex;
  uint16_t uhHighFreqNegativeTurnOnIndex;
  uint16_t uhHighFreqNegativeTurnOffIndex;  
  uint16_t uhHighFreqPositiveStartedIndex;
  uint16_t uhHighFreqNegativeStartedIndex;
  

  uint8_t ubInputVoltageRangeForZvs;
  
  
  uint16_t uhZvdTimerCounter;
  uint16_t uhZvdTimerCounterDefault;
  uint16_t uhZvdTimerCounterMin;
  uint16_t uhZvdTimerCounterMed;
  uint16_t uhZvdTimerCounterMax;
  FunctionalState ZvdFrequencyFollower;
  FunctionalState ZvdFilter;
  ErrorStatus ZvdControlStatus;
  DPC_LPCNTRL_SubStateMachine_t FrequencyCheck;
  uint32_t uwLineFrequencyMeasureCounter;
  uint8_t ubLineFrequencyMeasure;
  uint8_t ubLineFrequencyPreviousValue;
  uint8_t ubLineFrequencyValue;
  
  FlagStatus StateCh1;
  FlagStatus StateCh2;
  FlagStatus StateCh3;

  FlagStatus BoostSwitchCh1;
  FlagStatus BoostSwitchCh2;
  FlagStatus BoostSwitchCh3; 
  FlagStatus RectSwitchCh1;
  FlagStatus RectSwitchCh2;
  FlagStatus RectSwitchCh3;
  
  FlagStatus      SingleLineCycleTest;
  FlagStatus      LowFreqLowSideSwitch;
  FlagStatus      LowFreqHighSideSwitch;
  FunctionalState PositiveHalfCycle;
  FunctionalState NegativeHalfCycle;
  FunctionalState HighFreqDriving;
  FunctionalState LowFreqDriving;
  
  DPC_PID_PIs32_t Iac_PI;
  DPC_PID_PIs32_t IL1_PI;
  DPC_PID_PIs32_t IL2_PI;
  DPC_PID_PIs32_t IL3_PI;
  DCP_MTH_SlopeLimiter_t IrefLimited;
  
    
  uint32_t uwTswPeriod;
  uint32_t uwPhaseShiftCh2Mode2;
  uint32_t uwPhaseShiftCh2Mode3;
  uint32_t uwPhaseShiftCh3;
  
  uint32_t uwTonMax;
  uint32_t uwTonMin;
  
  uint32_t uwTonCh1;
  uint32_t uwToffCh1;
  
  uint32_t uwTonCh2;
  uint32_t uwToffCh2;
  
  uint32_t uwTonCh3;
  uint32_t uwToffCh3;
  
  FunctionalState CurrentControl;
  DPC_LPCNTRL_SubStateMachine_t DutySoftStart;
  FlagStatus DutySoftStartPolarityType;
  DPC_LPCNTRL_SubStateMachine_t NegativeSoftStartup;
  DPC_MTH_RampGenerator_t DutyCycle_Ramp;
  uint16_t uhDutyCycleRampDuration;
  uint16_t uhDutyCycleRampEnableWindow;
  uint32_t uwSoftDutyInitValue;
  uint32_t uwSoftFinalValue;
  
  DPC_MTH_ExpGenerator_t DutyCycle_Exp;
  DPC_LUT_SoftDuty_t SoftDuty;
  
  DPC_LPCNTRL_SubStateMachine_t IacPkSoftStart;
  DPC_LPCNTRL_SubStateMachine_t IacPkUpdate;
  DPC_MTH_RampGenerator_t Iac_Ramp;
  
}DPC_AVGCC_Avg3Channels_t;
/****** DPC_AvgeresisCurrentCtrl.h section End *****/






//*** COMMON DATA of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC BEGIN ----------- ***/

typedef struct{
  //***ADC2
  uint16_t uhBufferADC2[ADC_SET1_LENGTH];
  
}DPC_CMNDAT_DataSet1_t;


typedef struct{
  //***ADC3
  uint16_t uhBufferADC3[2];
  
}DPC_CMNDAT_DataSet2_t;

typedef struct{
  //***ADC4
  uint16_t uhBufferADC4[2];

}DPC_CMNDAT_DataSet3_t;

typedef struct{
  //***ADC2
  uint16_t uhIL1;
  uint16_t uhIL2;
  uint16_t uhIL3;
  uint16_t uhVout;
  uint16_t uhIout;
  uint16_t uhTemp;
  uint16_t uhVinL1;
  uint16_t uhVinL2;
  //***ADC3
  uint16_t uhIin;
  uint16_t uhIinFlt;
  
  uint16_t uhVin; //line-neutral

  //*** ADC4
   uint16_t uhVinPreSwitch;
  
  //*** FOR DEBUG BEGIN***//
  uint16_t uhIL1Debug;
  uint16_t uhIL2Debug;
  uint16_t uhIL3Debug;
  uint16_t uhVoutDebug;
  uint16_t uhIoutDebug;
  uint16_t uhTempDebug;
  uint16_t uhVinL1Debug;
  uint16_t uhVinL2Debug;
  //***ADC3
  uint16_t uhIinDebug;
  uint16_t uhIinFltDebug;
  
  uint16_t uhVinDebug; //line-neutral
  //***ADC3
  uint16_t uhVinPreSwitchDebug;
  //*** FOR DEBUG END***//
  
}DPC_CMNDAT_PFC_RawData_t;


typedef struct{
  
  //***ADC2
  uint16_t uhAvgIL1;
  uint16_t uhAvgIL2;
  uint16_t uhAvgIL3;
  uint16_t uhAvgVout;
  float AvgVoutVolt;
  uint16_t uhAvgIout;
  uint16_t uhMovAvgIout;
  uint16_t uhAvgTemp;
  uint16_t uhAvgTempDegrees;
  uint16_t uhVinL1;
  uint16_t uhVinL2;
  //***ADC3
  uint16_t uhIin;
  uint16_t uhIinFlt;
  
  //*** ADC4
  uint16_t uhVinPreSwitch;

  uint16_t uhVinPkVolt; // Actual Peak input voltage in [V]
  uint16_t uhVinPkToVout; // Actual Peak input voltage reported to Vout sense
  uint16_t uhDeltaVinPkVoutRelayToVout; // Minimum difference between Vinpk and Vout [V] for relay turn-on reported to Vout sense
  uint16_t uhVoutMinRelay; // Minimum Vout for relay turn-on
  uint16_t uhVinRms; // Actual RMS input voltage in [bits]
  uint16_t uhVinRmsVolt; // Actual RMS input voltage in [V]
  
  uint16_t uhVinRmsMin; // Minimum RMS input voltage in [V]
  uint16_t uhVinRmsMax; // Maximum RMS input voltage in [V]
  uint16_t uhVinRmsDer; // Derating RMS input voltage in [V]
  uint16_t uhVinPreSwitchRms;//RMS prre switch for inverter and also pfc 
  uint16_t uhVinRmsFactor; // Conversion to Rms factor
  
  uint16_t uhVinDropoutThresholdDetect; // Input voltage threshold for Drop-Out detect
  uint16_t uhVinDropoutThresholdEnd;    // Input voltage threshold for Drop-Out end
  
  uint16_t uhVoutRef; // Output voltage reference reported to Vout sense Editable for DEBUG
  uint32_t uwVoutSetPoint; // Output voltage reference reported to Vout sense set by VoutRef NOT EDITABLE
  uint16_t uhVoutNewSetpoint; // New value for vout value update
  
  
  uint16_t uhVoutBurstMax; // High-threshold voltage for burst mode control
  uint16_t uhVoutBurstMin; // Low-threshold voltage for burst mode control
  uint16_t uhVoutOverVoltageValue; // DC out over-voltage level for protection control
  uint16_t uhVbusUnderVoltageValuePFC; // DC bus under-voltage level for protection control in PFC MODE
  uint16_t uhVbusUnderVoltageValueInverter; // DC bus under-voltage level for protection control in INVERTER MODE 
  
  uint16_t uwIload; // Actual Output (load) current in [bits]
  
  uint16_t uhIacRefInverter;              //Actual Iac peak current reference in INVERTER MODE
  float IacRefInverterAmpere;              //Actual Iac peak current reference in INVERTER MODE
  uint16_t uhIacRefInverterNewValue;      //New value for Iac peak current reference UPDATE in INVERTER MODE
  uint16_t uhIdcLoadConnected;
  uint16_t uhIdcLoadDisconnected;
  uint16_t uhIdcLoadStartupMaxValue;
  
  float PhaseAngleAdjDegrees; //AC phase angle adjustment for AC Current reference
  float PhaseAngleAdjDegrees_NewValue; //AC phase angle adjustment for AC Current reference
  uint16_t uhPhaseAngleAdjIndex;
  uint16_t uhPhaseAngleAdjIndex_NewValue;
  
  float ILtotGain;
  float ILGain;
  float IdcGain;
  float VdcGain;
  float VacGain;
  

}DPC_CMNDAT_PFC_ControlData_t;


void DPC_CMNDAT_GetDataAllSet(uint16_t* pDataSet1, uint16_t* pDataSet2,uint16_t* pDataSet3,  DPC_CMNDAT_PFC_RawData_t* pDataAllSet, FunctionalState GetDataMode); 

//*** COMMON DATA of 3CH INTERLEAVED STEVAL-BIDIRCB-PFC END ------------- ***/




#endif    /*__DPC_DATACOLLECTOR_H*/


/************************ (C) COPYRIGHT STMicroelectronics 2020 END OF FILE****/
