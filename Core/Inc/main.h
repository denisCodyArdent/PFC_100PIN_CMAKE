/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_1_Pin GPIO_PIN_13
#define TEST_1_GPIO_Port GPIOC
#define TEST_2_Pin GPIO_PIN_14
#define TEST_2_GPIO_Port GPIOC
#define TEST_3_Pin GPIO_PIN_15
#define TEST_3_GPIO_Port GPIOC
#define I_IN_TOT_F_Pin GPIO_PIN_0
#define I_IN_TOT_F_GPIO_Port GPIOC
#define I_L2_F_Pin GPIO_PIN_1
#define I_L2_F_GPIO_Port GPIOC
#define I_LOAD_F_Pin GPIO_PIN_2
#define I_LOAD_F_GPIO_Port GPIOC
#define I_AVG_L1_Pin GPIO_PIN_3
#define I_AVG_L1_GPIO_Port GPIOC
#define VACN_F_Pin GPIO_PIN_0
#define VACN_F_GPIO_Port GPIOA
#define I_L1_F_Pin GPIO_PIN_1
#define I_L1_F_GPIO_Port GPIOA
#define ILF4_HS_Pin GPIO_PIN_2
#define ILF4_HS_GPIO_Port GPIOA
#define I_L1_FA3_Pin GPIO_PIN_3
#define I_L1_FA3_GPIO_Port GPIOA
#define I_AVG_L3_Pin GPIO_PIN_4
#define I_AVG_L3_GPIO_Port GPIOA
#define VACN_FA5_Pin GPIO_PIN_5
#define VACN_FA5_GPIO_Port GPIOA
#define ZVDout_test_Pin GPIO_PIN_6
#define ZVDout_test_GPIO_Port GPIOA
#define VACL_F_Pin GPIO_PIN_7
#define VACL_F_GPIO_Port GPIOA
#define VOUT_F_Pin GPIO_PIN_4
#define VOUT_F_GPIO_Port GPIOC
#define I_AVG_L2_Pin GPIO_PIN_5
#define I_AVG_L2_GPIO_Port GPIOC
#define I_L2_FB0_Pin GPIO_PIN_0
#define I_L2_FB0_GPIO_Port GPIOB
#define ZVD_VACsign_F_Pin GPIO_PIN_1
#define ZVD_VACsign_F_GPIO_Port GPIOB
#define ZVD_VACsign_F_EXTI_IRQn EXTI1_IRQn
#define VACL_FB2_Pin GPIO_PIN_2
#define VACL_FB2_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_8
#define R1_GPIO_Port GPIOE
#define R2_Pin GPIO_PIN_9
#define R2_GPIO_Port GPIOE
#define R3_Pin GPIO_PIN_10
#define R3_GPIO_Port GPIOE
#define R4_Pin GPIO_PIN_11
#define R4_GPIO_Port GPIOE
#define C1_Pin GPIO_PIN_12
#define C1_GPIO_Port GPIOE
#define C2_Pin GPIO_PIN_13
#define C2_GPIO_Port GPIOE
#define C3_Pin GPIO_PIN_14
#define C3_GPIO_Port GPIOE
#define C4_Pin GPIO_PIN_15
#define C4_GPIO_Port GPIOE
#define ILF4_LS_Pin GPIO_PIN_10
#define ILF4_LS_GPIO_Port GPIOB
#define I_L3_F_Pin GPIO_PIN_11
#define I_L3_F_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOB
#define I_L3_FB13_Pin GPIO_PIN_13
#define I_L3_FB13_GPIO_Port GPIOB
#define I_LTOT_Pin GPIO_PIN_14
#define I_LTOT_GPIO_Port GPIOB
#define TEMP_Pin GPIO_PIN_15
#define TEMP_GPIO_Port GPIOB
#define InMainsL_Pin GPIO_PIN_8
#define InMainsL_GPIO_Port GPIOD
#define InMainsA_Pin GPIO_PIN_9
#define InMainsA_GPIO_Port GPIOD
#define DAB_EN_Pin GPIO_PIN_10
#define DAB_EN_GPIO_Port GPIOD
#define Essential_RLY_Pin GPIO_PIN_12
#define Essential_RLY_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_13
#define FAN_GPIO_Port GPIOD
#define MAINS_SW_Pin GPIO_PIN_14
#define MAINS_SW_GPIO_Port GPIOD
#define Disp_Led_Pin GPIO_PIN_15
#define Disp_Led_GPIO_Port GPIOD
#define SYNC_3_Pin GPIO_PIN_6
#define SYNC_3_GPIO_Port GPIOC
#define YELLOW_LED_Pin GPIO_PIN_7
#define YELLOW_LED_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_12
#define RED_LED_GPIO_Port GPIOA
#define IsecFault_Pin GPIO_PIN_0
#define IsecFault_GPIO_Port GPIOD
#define SYNC_2_Pin GPIO_PIN_4
#define SYNC_2_GPIO_Port GPIOB
#define SYNC_1_Pin GPIO_PIN_5
#define SYNC_1_GPIO_Port GPIOB
#define BLUE_LED_Pin GPIO_PIN_6
#define BLUE_LED_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_7
#define RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
