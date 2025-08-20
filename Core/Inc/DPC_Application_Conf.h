/**
******************************************************************************
* @file    DPC_Application_Conf.h
* @brief   This file contains the DPC Library Configuration.
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
#ifndef __DPC_APPLICATION_CONF_H
#define __DPC_APPLICATION_CONF_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


///__Start_________________________________________________________________BEATRICE_______________________________________________________________________


//******************************************************************************
// CUSTOMIZABLE PARAMETERS BEGIN
//******************************************************************************

///*** I/O ----------------------------------------- 
///// AC�Parameters�(1�)
#define    DPC_VIN_MAX            270    // Max input voltage [Expressed in Volt]
#define    DPC_VIN_MIN_DER        230    // Derating input voltage for power limitation [Expressed in Volt]
#define    DPC_VIN_MIN             180   // Min input voltage for India range [Expressed in Volt]
///// DC�Parameters�(2�)
#define    DPC_VBUS_REF_DEFAULT      400// NominalDefault output voltage [Expressed in Volt]
#define    DPC_POUT 	            1000    // Nominal output power [Expressed in Watts]
#define    DPC_EFF 	            0.98    // Target minimum efficiency at full load
#define    DPC_PF 	            0.99    // Target minimum PF at full load

///*** Sensing (2�) -------------------------------
//// AC�Section�(1�)
#define    DPC_VAC_GAIN             8.875   // Input voltage sensing gain [Expressed in bits/Volts]
#define    DPC_IL_TOT_GAIN          50.4    // Total Inductor current sensing sensitivity [Expressed in bits/Ampere]
#define    DPC_IZERO_OFFSET_IL      2048           // Inductor current sensing zero-current offset [Expressed in bits]
#define    DPC_IL_GAIN              50.4    // Single Inductor current sensing sensitivity [Expressed in bits/Ampere]
//// DC�Section�(2�)
#define    DPC_VBUS_GAIN           8.875   // Vbus voltage sensing gain [Expressed in bits/Volts]
#define    DPC_IZERO_OFFSET_IDC     2048           // DC current sensing zero-current offset [Expressed in bits]
#define    DPC_IDC_GAIN             50.4    // DC current sensing sensitivity [Expressed in bits/Ampere]

///*** CTRL (3�) -------------------------------
//// Voltage�Control (1�)
#define    DPC_KP_VDC               0.052064	// Proportional gain of Vbus voltage PI regulator
#define    DPC_KI_VDC        	    10.480219	// Integral gain of Vbus voltage PI regulator
//// Vbus update (2�)
#define    DPC_VBUS_REF_MAX 	     460.0     	// Max voltage value for Vbus update control [Expressed in Volt]
#define    DPC_VBUS_REF_MIN 	     380.0     	// Min voltage value for Vbus update control [Expressed in Volt]
//// Current�Control (3�)
#define    DPC_SWITCHING_FREQUENCY  65000        // Switching frequency of driving signals [Expressed in Hz]
#define    DPC_IPK_REF_MAX           14        // Upper current limit for voltage-loop output [Expressed in Ampere]
#define    DPC_IPK_REF_MIN            0.0        // Lower current limit for voltage-loop output [Expressed in Ampere]
#define    DPC_IPK_REF_BURST          1.0       // Peak current reference during burst-mode operation (open-loop) [Expressed in Ampere]
#define    DPC_KP_IAC             0.854364	 // Proportional gain of Iac current PI regulator
#define    DPC_KI_IAC 	          10201.05       // Integral gain of Iac current PI regulator

///*** Protection (3�) -------------------------------
//// AC�Section�(1�)
#define    DPC_OCP_EN                      true  // Inductor over-current protection (OCP) Enable [Expressed in Boolean]
#define    DPC_OCP_VALUE                   19.0  // Inductor over-current protection (OCP) value, to avoid inductor saturation [Expressed in Ampere]
//// DC�Section�(2�)
#define    DPC_VBUS_OVP 	          440.0  // Vbus over-voltage protection (OVP) value [Expressed in Volt]
#define    DPC_VBUS_UVP_PFC               350.0  // Vbus under-voltage protection (UVP) value in PFC mode [Expressed in Volt]
#define    DPC_VBUS_UVP_INVERTER          360.0  // Vbus under-voltage protection (UVP) value in INVERTER mode [Expressed in Volt]

//******************************************************************************
// CUSTOMIZABLE PARAMETERS END
//******************************************************************************





//----------------------------------------------------------------------------------------------






//******************************************************************************
// NOT CUSTOMIZABLE PARAMETERS BEGIN
#define     DPC_VIN_NOM            230    // Nominal input voltage [Expressed in Volt]
#define     DPC_LINE_FREQ_MIN       46    // Minimum input line frequency [Expressed in Hertz]
#define     DPC_LINE_FREQ_NOM       50    // Nominal input line frequency [Expressed in Hertz]
#define     DPC_LINE_FREQ_MAX       64    // Maximum input line frequency [Expressed in Hertz]
#define     DPC_LINE_FREQ_COUNTER 1000    // line frequency counter value at 50Hz [Expressed in bits]
#define     DPC_LINE_FREQ_DEFAULT 50.5    // line frequency default value for first measure init [Expressed in Hertz]//******************************************************************************
#define DPC_PHASE_INVERTER_SINGLE         true       // Inverter single-phase mode Enable [Expressed in Boolean] This is number of synced stages 

//// Inrush Current�Control (3�)
#define    DPC_TIME_ON_RELAY         1188	// Delay time for relay activation [Expressed in milliseconds]
#define    DPC_DELAY_RELAY              8       // Release time of relay mechanical contacts [Expressed in milliseconds]
#define    DPC_DELTA_VINPK_VOUT_RELAY  20       // Minimum difference between input (peak) and output voltage to allow the relay turn-on [Expressed in Volts]
//// Start-up�Control (4�)
#define    DPC_SOFTSTARTUP_DURATION       1155	// Time duration of PFC soft start-up with load [Expressed in milliseconds]
#define    DPC_IDC_MAX_LOAD_STARTUP_PERC 14.285 // Max allowed Idc current percentage of max Idc for soft start-up with load [Expressed in %] 
#define    DPC_IDC_LOAD_CONNECTED_PERC    1.428	// Idc current percentage of max Idc for load mode activation [Expressed in %] 
//// Fan�Control (5�)
#define    DPC_FAN_ENABLE            true       // Fan control Enable [Expressed in Boolean]
#define    DPC_ILOAD_FAN_PERC         30       // Idc current threshold percentage of max Idc for fan activation [Expressed in %] 
//// Load Feed-Forward�Control (6�)
#define    DPC_FF_SHIFT                 5        // Exponential shift for load feed-forward calculation [Expressed in bits] 
#define    DPC_FF_TRIPPING              2        // Counter for load feed-forward filtering [Expressed in bits] 
#define    DPC_FF_UPPER_THRESHOLD      42        // Upper threshold for load feed-forward activation [Expressed in bits] 
#define    DPC_FF_LOWER_THRESHOLD      22        // Lower threshold for load feed-forward activation [Expressed in bits]
#define    DPC_FF_UP                   roundf(26.0f * (54.61333333f/(float)DPC_IDC_GAIN) * (230.0f/(float)DPC_VIN_NOM) * ((float)DPC_VBUS_REF_DEFAULT/400.0f) * ((float)DPC_IL_TOT_GAIN/25.44484848f))     // Adjustment factor for load feed-forward step-up in EU range
#define    DPC_FF_DOWN                 DPC_FF_UP                                                                                                                                                           // Adjustment factor for load feed-forward step-down in EU range [Expressed in bits]  

//// Phase-Shedding�Control (7�)
#define    DPC_IDC_1TO2             (uint16_t)roundf((float)DPC_POUT / (float)DPC_VBUS_REF_DEFAULT * (float)0.31428f * (float)DPC_IDC_GAIN) //300 // Load current threshold for 1ch-to-2ch transition [Expressed in bits]
#define    DPC_IDC_2TO1             (uint16_t)roundf((float)DPC_IDC_1TO2 * 0.85f) //255	// Load current threshold for 2ch-to-1ch transition [Expressed in bits]
#define    DPC_IDC_2TO3             (uint16_t)roundf((float)DPC_IDC_1TO2 * 2.0f) //601	// Load current threshold for 2ch-to-3ch transition [Expressed in bits]
#define    DPC_IDC_3TO2             (uint16_t)roundf((float)DPC_IDC_2TO3 * 0.85f) //511	// Load current threshold for 3ch-to-2ch transition [Expressed in bits]
#define    DPC_IAC_PK_1TO2          (uint16_t)roundf((float)DPC_POUT / (float)DPC_VIN_MIN_DER * (float)sqrtf(2.0f) * (float)0.31428f * (float)DPC_IL_TOT_GAIN) //344// Iac peak current threshold for 1ch-to-2ch transition [Expressed in bits]
#define    DPC_IAC_PK_2TO1          (uint16_t)roundf((float)DPC_IAC_PK_1TO2 * 0.85f) //292// Iac peak current threshold for 2ch-to-1ch transition [Expressed in bits]
#define    DPC_IAC_PK_2TO3          (uint16_t)roundf((float)DPC_IAC_PK_1TO2 * 2.0f) //688// Iac peak current threshold for 2ch-to-3ch transition [Expressed in bits]
#define    DPC_IAC_PK_3TO2          (uint16_t)roundf((float)DPC_IAC_PK_2TO3 * 0.85f) //585// Iac peak current threshold for 3ch-to-2ch transition [Expressed in bits]


#define    DPC_VIN_LOW_VOLTAGE_RESET_COUNT  200  // Input low-voltage reset counts value in Vin measurement [Expressed in bits]
#define    DPC_VIN_LOW_VOLTAGE_PROT_COUNT    50  // Input low-voltage max allowed counts in Vin measurement [Expressed in bits]
#define    DPC_VIN_DROPOUT_DETECT            25  // Input voltage threshold for drop-out detect [Expressed in Volts]


//// Others�(3�)
#define    DPC_OTP_EN                      true  // Ambient over-temperature protection (OTP) Enable [Expressed in Boolean]
#define    DPC_OTSHUTDOWN                    55	 // Ambient over-temperature protection (OTP) value [Expressed in Celsius degrees]
#define    DPC_DUTY_MAX                    0.99  ///150  // Max duty cycle [Expressed in bits]
#define    DPC_DUTY_MIN                    0.01  ///150  // Min duty cycle [Expressed in bits]





#define     DPC_VBUS_UPDATE_DURATION 5000 // Time duration of PFC Vout (Vbus) update [Expressed in milliseconds]
#define     DPC_VBURST_UPDATE_COEFF_MIN 1.0256 // Coeff. for min Vout (Vbus) burst update [Expressed in bits]
#define     DPC_VBURST_UPDATE_COEFF_MAX 1.0769 // Coeff. for min Vout (Vbus) burst update [Expressed in bits]

#define     DPC_TO_INRUSH_TICK     DPC_TIME_ON_RELAY
#define     DPC_TO_RELAY_COMP_TICK    DPC_DELAY_RELAY
#define     DPC_TO_PFC_START_TICK   100
#define     DPC_TO_PFC_STOP_TICK   1000
#define     DPC_TO_DROP_OUT_TICK     25

//*** Current thresholds parameters BEGIN ***//  
#define    DPC_ITH_REFH_MAX         4000   // IthRefHMax,
#define    DPC_ITH_REFL_MAX         4000   // IthRefLMax,
#define    DPC_ITH_REFH_MIN            0   // IthRefHMin,
#define    DPC_ITH_REFL_MIN            0   // IthRefLMin, 
#define    DPC_IR_COMP               100   
#define    DPC_IR_OFFSET               0   
#define    DPC_IF_OFFSET              20   
#define    DPC_CMP_OFFSET             30   
//*** Current thresholds parameters END ***//

//*** Converter start mode parameters BEGIN ***// 
#define     DPC_MANUAL_START          DISABLE
#define     DPC_AUTO_START            ENABLE
//*** Converter start mode parameters END ***//

//*** Converter driving mode parameters BEGIN ***// 
#define     DPC_SINGLE_CYCLE_DRIVING  SET
#define     DPC_NORMAL_CYCLE_DRIVING  RESET
//*** Converter driving mode parameters END ***//

#define     DPC_PI_EXP                 10  //exponential 2^(DPC_VDC_PI_EXP) PI parameters multiplier factor to allows integer calculation without losing resolution.

//*** Voltage PI parameters BEGIN ***// 
#define     DPC_VDC_CTRL_FREQ          1000  //Control frequency of the voltage loop
#define     DPC_KP_VDC_MICRO           (uint32_t)(roundf((float)DPC_KP_VDC * powf(2.0f, (float)DPC_PI_EXP)))	                    // Proportional gain of Vbus voltage PI regulator scaled for integer PI
#define     DPC_KI_VDC_MICRO 	       (uint32_t)(roundf((float)DPC_KI_VDC * powf(2.0f, (float)DPC_PI_EXP) / (float)DPC_VDC_CTRL_FREQ)) // Integral gain of Vbus voltage PI regulator scaled for integer PI
#define     DPC_VOLTAGE_PI_SAT_ENABLE  SET
#define     DPC_VOLTAGE_PI_SAT_DISABLE  RESET
#define     DPC_VOLTAGE_PI_ANTIWINDUP_ENABLE  SET
#define     DPC_VOLTAGE_PI_ANTIWINDUP_DISABLE  RESET
#define     DPC_VOLTAGE_PI_ANTIWINDUP_GAIN  1
#define     DPC_VOLTAGE_PI_RESET_VALUE      0
//*** Voltage PI parameters END ***//

//*** Current PI parameters BEGIN ***// 
#define     DPC_IAC_CTRL_FREQ        40000  //Control frequency of the Current loop
#define     DPC_KP_IAC_MICRO         (uint32_t)(roundf((float)DPC_KP_IAC * powf(2.0f, (float)DPC_PI_EXP)))                            // Proportional gain of Iac current PI regulator scaled for integer PI
#define     DPC_KI_IAC_MICRO 	     (uint32_t)(roundf((float)DPC_KI_IAC * powf(2.0f, (float)DPC_PI_EXP) / (float)DPC_IAC_CTRL_FREQ)) // Integral gain of Iac current PI regulator scaled for integer PI
#define     DPC_CURRENT_PI_SAT_ENABLE  SET
#define     DPC_CURRENT_PI_SAT_DISABLE  RESET
#define     DPC_CURRENT_PI_ANTIWINDUP_ENABLE  SET
#define     DPC_CURRENT_PI_ANTIWINDUP_DISABLE  RESET
#define     DPC_CURRENT_PI_ANTIWINDUP_GAIN  1
#define     DPC_CURRENT_PI_RESET_VALUE      0
//*** Current PI parameters END ***//

//*** Duty Soft Start mode parameters BEGIN ***// 
#define     DPC_POS_DUTY_SOFTSTART  SET
#define     DPC_NEG_DUTY_SOFTSTART  RESET
#define     DPC_SOFT_DUTY_DURATION  5
#define     DPC_SOFT_DUTY_ENABLE_WINDOW  2
#define     DPC_SOFT_DUTY_PARAM_1        1.0
//*** Duty Soft Start mode parameters END ***//

#define     DPC_ZVD_CHECK  SET
#define     DPC_ZVD_RELOAD  RESET
#define     DPC_ZVD_FOLLOWER_CHECK  SET
#define     DPC_ZVD_FOLLOWER_RELOAD  RESET

#define     DPC_DEV_MODE  ENABLE  //Development mode --> converter signals value set by user
#define     DPC_ADC_MODE  DISABLE //ADC mode --> converter signals value set by ADC reading

#define     DPC_LOAD_FF_EN       ENABLE  // Load feed-forward Enable [Expressed in Boolean]
#define     DPC_VINP_FF_EN       ENABLE  // Input voltage feed-forward Enable [Expressed in Boolean]


#define     DPC_TO_INRUSH     TO_INDX1
#define     DPC_TO_RELAY_COMP TO_INDX2
#define     DPC_TO_PFC_START  TO_INDX3
#define     DPC_TO_PFC_STOP   TO_INDX4
#define     DPC_TO_DROP_OUT   TO_INDX5

//*** LUT control parameters BEGIN ***//
#define    DPC_LUT_POINTS            800   // Number of LUT points
#define    DPC_LUT_PERIOD_POINTS     800   // Number of LUT points for one AC grid line-cycle
#define    DPC_LUT_PERIOD_TIM_CNT   1000   // Default LUT Timer counter at 50Hz with DPC_NUM_LUT_POINTS = 200.
#define    PI                  3.1415926   // Pi Greek
 // Number of converter channels [Expressed in bits]
#define    DPC_NCH                     1    // this must be 3 for non single phase
#define    DPC_ISENSE_DELAY       350E-9
#define    DPC_NUM_OF_VIN_SET          6
#define    DPC_SQRT_2          1.4142135
//*** LUT control parameters END ***//


//*** Data processing parameters BEGIN ***//
//ILtot
#define    DPC_ILtot_WEIGHT_1         2    // 1� weight for multistage averaging of ILtot [Expressed in bits]
//IL1
#define    DPC_IL1_WEIGHT_1          32    // 1� weight for multistage averaging of IL1 [Expressed in bits]
//IL2
#define    DPC_IL2_WEIGHT_1          32    // 1� weight for multistage averaging of IL2 [Expressed in bits]
//IL3
#define    DPC_IL3_WEIGHT_1          32    // 1� weight for multistage averaging of IL3 [Expressed in bits]
//Vin
#define    DPC_VIN_WEIGHT_1           2    // 1� weight for multistage averaging of Vin [Expressed in bits]
#define    DPC_VIN_WEIGHT_2           8    // 2� weight for multistage averaging of Vin [Expressed in bits]
#define    DPC_VIN_WEIGHT_3          16    // 3� weight for multistage averaging of Vin [Expressed in bits]
//Vout
#define    DPC_VOUT_WEIGHT_1           2    // 1� weight for multistage averaging of Vout [Expressed in bits]
#define    DPC_VOUT_WEIGHT_2           4    // 2� weight for multistage averaging of Vout [Expressed in bits]
#define    DPC_VOUT_WEIGHT_3           8    // 3� weight for multistage averaging of Vout [Expressed in bits]
//Iout
#define    DPC_IOUT_WEIGHT_1           1    // 1� weight for multistage averaging of Iout [Expressed in bits]
#define    DPC_IOUT_WEIGHT_2           2    // 2� weight for multistage averaging of Iout [Expressed in bits]
#define    DPC_IOUT_WEIGHT_3           4    // 3� weight for multistage averaging of Iout [Expressed in bits]
#define    DPC_IOUT_WEIGHT_4           8    // 4� weight for multistage averaging of Iout [Expressed in bits]
#define    DPC_AVG_SAMPLE              8    // weight for moving average of Iout [Expressed in bits]
#define    DPC_IZERO_ILOAD          2048//2038//2069///1    // Iload zero-offset compensation [Expressed in bits]
//Ambient Temperature
#define    DPC_TEMP_WEIGHT_1          64    // 1� weight for multistage averaging of Ambient Temperature [Expressed in bits]
//*** Data processing parameters END ***//


//*** Control Data parameters BEGIN ***//
#define    DPC_VIN_MIN_TOLERANCE          97    // Tolerance for Min Vin control [Expressed in %]
#define    DPC_VIN_MAX_TOLERANCE         110    // Tolerance for Max Vin control [Expressed in %]
#define    DPC_VIN_MIN_DER_TOLERANCE     100    // Tolerance for Derating Vin control [Expressed in %]
#define    DPC_VIN_TOLERANCE_DIV         100    // Tolerance divider for Vin control [Expressed in %]
#define    DPC_VIN_PK_FACTOR            1414    // Peak Vin factor multiplied by DPC_VIN_GAIN_DIV [Expressed in bits]
#define    DPC_VIN_GAIN_DIV             1000    // Vin gain divider factor [Expressed in bits]
#define    DPC_VIN_DROPOUT_DIV           100    // Vin dropout divider factor [Expressed in bits]
#define    DPC_VIN_DROPOUT_END_FACTOR    110    // Vin dropout-end factor [Expressed in bits]

#define    DPC_VOUT_CTRL_LOWER_TH1_ENHANCE_FACTOR    99 // Vout first low threshold enhancement factor for voltage control [Expressed in bits]
#define    DPC_VOUT_CTRL_HIGHER_TH1_ENHANCE_FACTOR  101 // Vout first high threshold enhancement factor for voltage control [Expressed in bits]
#define    DPC_VOUT_CTRL_LOWER_TH2_ENHANCE_FACTOR    98 // Vout second low threshold enhancement factor for voltage control [Expressed in bits]
#define    DPC_VOUT_CTRL_HIGHER_TH2_ENHANCE_FACTOR  102 // Vout second high threshold enhancement factor for voltage control [Expressed in bits]
#define    DPC_VOUT_CTRL_ENHANCE_DIV                100 // Vout divider factor for voltage control enhancement [Expressed in bits]
#define    DPC_VOUT_GAIN_DIV             1000    // Vout gain divider factor [Expressed in bits]
#define    DPC_AC_PHASE_ANGLE_ADJ        0.0     // //AC phase angle adjustment for AC Current reference in [degrees]
#define    DPC_AC_PHASE_ANGLE_ADJ_MAX    5.0     // //Max AC phase angle adjustment for AC Current reference in [degrees]
#define    DPC_AC_PHASE_ANGLE_ADJ_MIN   -5.0     // //Min AC phase angle adjustment for AC Current reference in [degrees]
#define    DPC_AC_PHASE_ANGLE_PERIOD   360.0     // //AC phase angle period for AC Current reference in [degrees]
//*** Control Data parameters END ***// 



//*** Dead-times parameters BEGIN ***//
#define    DPC_RISING_DEAD_TIME_MAX   300   //uint32_t RisingDeadTimeMax, T_zero_Vds_max max rising dead-time
#define    DPC_FALLING_DEAD_TIME_MAX  300   //uint32_t FallingDeadTimeMax, T_Vout_Vds_max max falling dead-time
#define    DPC_RISING_DEAD_TIME_MIN    45   //uint32_t RisingDeadTimeMin, T_zero_Vds_min min rising dead-time (30=200ns)
#define    DPC_FALLING_DEAD_TIME_MIN   45   //uint32_t FallingDeadTimeMin, T_Vout_Vds_min min falling dead-time (30=200ns)
#define    DPC_RISING_DEAD_TIME_COMP  100   //uint32_t RisingDeadTimeComp, T_zero_Vds_comp rising dead-time correction
#define    DPC_FALLING_DEAD_TIME_COMP 350   //uint32_t FallingDeadTimeComp, T_Vout_Vds_comp falling dead-time correction
//*** Dead-times parameters END ***//

#define    DPC_PHASESHIFT_CH2_2       180	// Phase shift of the second channel (slave 1) with 2 channels enabled [Expressed in degrees] 
#define    DPC_PHASESHIFT_CH2_3       120	// Phase shift of the second channel (slave 1) with 3 channels enabled [Expressed in degrees]
#define    DPC_PHASESHIFT_CH3         240	// Phase shift of the third channel (slave 2) with 3 channels enabled [Expressed in degrees]
#define    DPC_PHASE_PERIOD           360	// Phase period related to the switching cycle [Expressed in degrees]

//*** MCU parameters BEGIN ***//
#define    DPC_VDD_MCU               3.3   // MCU supply voltage [Expressed in Volt]
#define    DPC_ADC_DAC_NBIT           12   // Number of bits for ADC and DAC resolution [Expressed in bits]
#define    DPC_DAC_ZERO_CURRENT     2048
#define    DPC_HRTIM_FREQ          150e6
#define    DPC_TDEAD_PRESC             1
#define    DPC_PHY_TO_DIG_GAIN_SCALE   ((powf(2.0f,(float)DPC_ADC_DAC_NBIT))/((float)DPC_VDD_MCU))
//*** MCU parameters END ***//


#define    DPC_SLOPE_MAX                 ((float)DPC_IL_TOT_GAIN * (float)1.0f * 0.75f) // Iac reference max slope Expressed in [bits/Amps]
#define    DPC_IPK_REF_INVERTER_DEFAULT  DPC_IPK_REF_BURST  // Default Iac peak reference level in [Ampere]
#define    DPC_IPK_REF_INVERTER_MIN      DPC_IPK_REF_MIN// Min Iac peak reference level in [Ampere]
#define    DPC_IPK_REF_INVERTER_MAX      DPC_IPK_REF_MAX// Max Iac peak reference level for INVERTER MODE in [Ampere]
#define    DPC_IAC_SOFTSTARTUP_UPDATE_DURATION  5000

//******************************************************************************
// NOT CUSTOMIZABLE PARAMETERS END
//******************************************************************************






///_________________________________________________________________________BASIC APPLICATION CONFIGURATOR______________________________________



///_________________________________________________________________________ADV APPLICATION CONFIGURATOR______________________________________


///_________________________________________________________________________CONTROL CONFIGURATOR______________________________________
//

/* Exported functions ------------------------------------------------------- */

#endif //__DPC_APPLICATION_CONF_H