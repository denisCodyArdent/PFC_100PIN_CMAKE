/**
******************************************************************************
* @file    DPC_Lib_Conf.h
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
#ifndef __DPC_LIB_CONF_H
#define __DPC_LIB_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

///__Start_________________________________________________________________STEVAL_BIDIRCB_PFC_______________________________________________________________________

//Main State machines section begin
#define   CONVERTER_INRUSH  1
#define   FREQUENCY_DETECT  2
#define   ILOAD_CHECK       3
#define   SOFT_STARTUP      4
#define   RUN_PFC_MODE      5
#define   RUN_INVERTER_MODE 6
#define   BURST_MODE        8
#define   DROP_OUT          9
#define   HALT             20
#define   FAULT            21
//Main State machines section end


//*** Driving Section BEGIN ***//-----------------------------------------------
// High-frequency legs (HRTIM) //
//Ch1
#define  MASTER_OUTPUT_LS       HRTIM_OUTPUT_TA1  //Master channel Low-Side output
#define  MASTER_OUTPUT_HS       HRTIM_OUTPUT_TA2  //Master channel High-Side output
//Ch2
#define  SLAVE1_OUTPUT_LS       HRTIM_OUTPUT_TB1  //1st Slave channel Low-Side output
#define  SLAVE1_OUTPUT_HS       HRTIM_OUTPUT_TB2  //1st Slave channel High-Side output
//Ch3
#define  SLAVE2_OUTPUT_LS       HRTIM_OUTPUT_TE1  //2nd Slave channel Low-Side output
#define  SLAVE2_OUTPUT_HS       HRTIM_OUTPUT_TE2  //2nd Slave channel High-Side output
// Low-frequency leg (GPIO) //
#define  LF_OUTPUT_LS_port      ILF4_LS_GPIO_Port //Low-frequency Low-Side output GPIO port
#define  LF_OUTPUT_LS_pin       ILF4_LS_Pin       //Low-frequency Low-Side output GPIO pin
#define  LF_OUTPUT_HS_port      ILF4_HS_GPIO_Port //Low-frequency Low-Side output GPIO port
#define  LF_OUTPUT_HS_pin       ILF4_HS_Pin       //Low-frequency Low-Side output GPIO pin
//*** Driving Section END ***//-------------------------------------------------


//*** LL Timer ID Section BEGIN ***//-------------------------------------------
//Ch1
#define  LL_MASTER_TIMER_ID       LL_HRTIM_TIMER_A  //Master Timer ID for LL functions
//Ch2
#define  LL_SLAVE1_TIMER_ID       LL_HRTIM_TIMER_B  //1st Slave Timer ID for LL functions
//Ch3
#define  LL_SLAVE2_TIMER_ID       LL_HRTIM_TIMER_E  //2nd Slave Timer ID for LL functions
//*** LL Timer ID Section END ***//---------------------------------------------


//*** HAL Timer ID Section BEGIN ***//------------------------------------------
//Ch1
#define  HAL_MASTER_TIMER_ID       HRTIM_TIMERID_TIMER_A  //Master Timer ID for HAL functions
//Ch2
#define  HAL_SLAVE1_TIMER_ID       HRTIM_TIMERID_TIMER_B  //1st Slave Timer ID for HAL functions
//Ch3
#define  HAL_SLAVE2_TIMER_ID       HRTIM_TIMERID_TIMER_E  //2nd Slave Timer ID for HAL functions
//*** HAL Timer ID Section END ***//--------------------------------------------


//*** HAL Timer INDEX Section BEGIN ***//---------------------------------------
//Ch1
#define  HAL_MASTER_TIMER_INDEX       HRTIM_TIMERINDEX_TIMER_A  //Master Timer ID for HAL functions
//Ch2
#define  HAL_SLAVE1_TIMER_INDEX       HRTIM_TIMERINDEX_TIMER_B  //1st Slave Timer ID for HAL functions
//Ch3
#define  HAL_SLAVE2_TIMER_INDEX       HRTIM_TIMERINDEX_TIMER_E  //2nd Slave Timer ID for HAL functions
//*** HAL Timer INDEX Section END ***//-----------------------------------------


//*** ADC Section BEGIN ***//------------------------------------------
//Data set 1
#define  ADC_SET1_ID       hadc2        //1st Data set ID for ADC functions
#define  ADC_SET1_LENGTH       9        //1st Data set lenght (Num of ADC conversion)
#define  IL1_RANK_ID           0        //1st Data set Rank 1 data label 
#define  IL2_RANK_ID           1        //1st Data set Rank 2 data label 
#define  IL3_RANK_ID           2        //1st Data set Rank 3 data label 
#define  VOUT_RANK_ID          3        //1st Data set Rank 4 data label 
#define  IOUT_RANK_ID          4        //1st Data set Rank 5 data label 
#define  TEMP_RANK_ID          5        //1st Data set Rank 6 data label 
#define  VIN_L1_RANK_ID        6        //1st Data set Rank 7 data label 
#define  VIN_L2_RANK_ID        7        //1st Data set Rank 8 data label 
#define  I_IN_FLT_RANK_ID      8        //1st Data set Rank 9 data label
//*** ADC Section END ***//--------------------------------------------





//*** Macro command Section BEGIN ***//-----------------------------------------
#define  LED_GREEN_ON   LL_GPIO_ResetOutputPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
#define  LED_GREEN_OFF  LL_GPIO_SetOutputPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
#define  LED_GREEN_STATE   HAL_GPIO_ReadPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

#define  LED_RED_ON     LL_GPIO_ResetOutputPin(RED_LED_GPIO_Port, RED_LED_Pin);
#define  LED_RED_OFF    LL_GPIO_SetOutputPin(RED_LED_GPIO_Port, RED_LED_Pin);
#define  LED_RED_STATE     HAL_GPIO_ReadPin(RED_LED_GPIO_Port, RED_LED_Pin);

#define  LED_YELLOW_ON  LL_GPIO_ResetOutputPin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);
#define  LED_YELLOW_OFF LL_GPIO_SetOutputPin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);
#define  LED_YELLOW_STATE  HAL_GPIO_ReadPin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);

#define  LED_BLUE_ON  LL_GPIO_ResetOutputPin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#define  LED_BLUE_OFF LL_GPIO_SetOutputPin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#define  LED_BLUE_STATE  HAL_GPIO_ReadPin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

#define  FAN_ON         LL_GPIO_SetOutputPin(FAN_GPIO_Port, FAN_Pin);
#define  FAN_OFF        LL_GPIO_ResetOutputPin(FAN_GPIO_Port, FAN_Pin);
#define  FAN_STATE      HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin);

#define  RELAY_ON       LL_GPIO_SetOutputPin(RELAY_GPIO_Port, RELAY_Pin);
#define  RELAY_OFF      LL_GPIO_ResetOutputPin(RELAY_GPIO_Port, RELAY_Pin);
#define  RELAY_STATE    HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin);

#define  LF_HS_ON       LL_GPIO_SetOutputPin(LF_OUTPUT_HS_port, LF_OUTPUT_HS_pin);
#define  LF_HS_OFF      LL_GPIO_ResetOutputPin(LF_OUTPUT_HS_port, LF_OUTPUT_HS_pin);
#define  LF_HS_STATE    HAL_GPIO_ReadPin(LF_OUTPUT_HS_port, LF_OUTPUT_HS_pin);

#define  LF_LS_ON       LL_GPIO_SetOutputPin(LF_OUTPUT_LS_port, LF_OUTPUT_LS_pin);
#define  LF_LS_OFF      LL_GPIO_ResetOutputPin(LF_OUTPUT_LS_port, LF_OUTPUT_LS_pin);
#define  LF_LS_STATE    HAL_GPIO_ReadPin(LF_OUTPUT_LS_port, LF_OUTPUT_LS_pin);

#define  TEST_1_ON      LL_GPIO_SetOutputPin(TEST_1_GPIO_Port, TEST_1_Pin);
#define  TEST_1_OFF     LL_GPIO_ResetOutputPin(TEST_1_GPIO_Port, TEST_1_Pin);
#define  TEST_1_TOGGLE  LL_GPIO_TogglePin(TEST_1_GPIO_Port, TEST_1_Pin);

#define  TEST_2_ON      LL_GPIO_SetOutputPin(TEST_2_GPIO_Port, TEST_2_Pin);
#define  TEST_2_OFF     LL_GPIO_ResetOutputPin(TEST_2_GPIO_Port, TEST_2_Pin);
#define  TEST_2_TOGGLE  LL_GPIO_TogglePin(TEST_2_GPIO_Port, TEST_2_Pin);

#define  TEST_3_ON      LL_GPIO_SetOutputPin(TEST_3_GPIO_Port, TEST_3_Pin);
#define  TEST_3_OFF     LL_GPIO_ResetOutputPin(TEST_3_GPIO_Port, TEST_3_Pin);
#define  TEST_3_TOGGLE  LL_GPIO_TogglePin(TEST_3_GPIO_Port, TEST_3_Pin);
//*** Macro command Section END ***//-------------------------------------------






///__End_________________________________________________________________STEVAL_BIDIRCB_PFC_______________________________________________________________________



/* Exported functions ------------------------------------------------------- */

#endif //__DPC_LIB_CONF_H