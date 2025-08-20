/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    comp.c
  * @brief   This file provides code for the configuration
  *          of the COMP instances.
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
/* Includes ------------------------------------------------------------------*/
#include "comp.h"

/* USER CODE BEGIN 0 */
extern single_phase;
/* USER CODE END 0 */

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp5;
COMP_HandleTypeDef hcomp6;
COMP_HandleTypeDef hcomp7;

/* COMP1 init function */
void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}
/* COMP2 init function */
void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}
/* COMP3 init function */
void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO2;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}
/* COMP4 init function */
void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}
/* COMP5 init function */
void MX_COMP5_Init(void)
{

  /* USER CODE BEGIN COMP5_Init 0 */

  /* USER CODE END COMP5_Init 0 */

  /* USER CODE BEGIN COMP5_Init 1 */

  /* USER CODE END COMP5_Init 1 */
  hcomp5.Instance = COMP5;
  hcomp5.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp5.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH1;
  hcomp5.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp5.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp5.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp5.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;
  if (HAL_COMP_Init(&hcomp5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP5_Init 2 */

  /* USER CODE END COMP5_Init 2 */

}
/* COMP6 init function */
void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */

  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */

  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp6.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH2;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */

  /* USER CODE END COMP6_Init 2 */

}
/* COMP7 init function */
void MX_COMP7_Init(void)
{

  /* USER CODE BEGIN COMP7_Init 0 */

  /* USER CODE END COMP7_Init 0 */

  /* USER CODE BEGIN COMP7_Init 1 */

  /* USER CODE END COMP7_Init 1 */
  hcomp7.Instance = COMP7;
  hcomp7.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp7.Init.InputMinus = COMP_INPUT_MINUS_DAC2_CH1;
  hcomp7.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp7.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp7.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp7.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP7_Init 2 */

  /* USER CODE END COMP7_Init 2 */

}

void HAL_COMP_MspInit(COMP_HandleTypeDef* compHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(compHandle->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspInit 0 */

  /* USER CODE END COMP1_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    */
    GPIO_InitStruct.Pin = I_L1_F_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L1_F_GPIO_Port, &GPIO_InitStruct);

    /* COMP1 interrupt Init */
    HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP1_MspInit 1 */

  /* USER CODE END COMP1_MspInit 1 */
  }
  else if(compHandle->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspInit 0 */

  /* USER CODE END COMP2_MspInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**COMP2 GPIO Configuration
    PA3     ------> COMP2_INP
    */
    GPIO_InitStruct.Pin = I_L1_FA3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L1_FA3_GPIO_Port, &GPIO_InitStruct);

    /* COMP2 interrupt Init */
    HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP2_MspInit 1 */

  /* USER CODE END COMP2_MspInit 1 */
  }
  else if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspInit 0 */

  /* USER CODE END COMP3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**COMP3 GPIO Configuration
    PC1     ------> COMP3_INP
    */
    GPIO_InitStruct.Pin = I_L2_F_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L2_F_GPIO_Port, &GPIO_InitStruct);

    /* COMP3 interrupt Init */
    HAL_NVIC_SetPriority(COMP1_2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
  /* USER CODE BEGIN COMP3_MspInit 1 */

  /* USER CODE END COMP3_MspInit 1 */
  }
  else if(compHandle->Instance==COMP4)
  {
  /* USER CODE BEGIN COMP4_MspInit 0 */

  /* USER CODE END COMP4_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    */
    GPIO_InitStruct.Pin = I_L2_FB0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L2_FB0_GPIO_Port, &GPIO_InitStruct);

    /* COMP4 interrupt Init */
    HAL_NVIC_SetPriority(COMP4_5_6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP4_5_6_IRQn);
  /* USER CODE BEGIN COMP4_MspInit 1 */

  /* USER CODE END COMP4_MspInit 1 */
  }
  else if(compHandle->Instance==COMP5)
  {
  /* USER CODE BEGIN COMP5_MspInit 0 */

  /* USER CODE END COMP5_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP5 GPIO Configuration
    PB13     ------> COMP5_INP
    */
    GPIO_InitStruct.Pin = I_L3_FB13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L3_FB13_GPIO_Port, &GPIO_InitStruct);

    /* COMP5 interrupt Init */
    HAL_NVIC_SetPriority(COMP4_5_6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP4_5_6_IRQn);
  /* USER CODE BEGIN COMP5_MspInit 1 */

  /* USER CODE END COMP5_MspInit 1 */
  }
  else if(compHandle->Instance==COMP6)
  {
  /* USER CODE BEGIN COMP6_MspInit 0 */

  /* USER CODE END COMP6_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    */
    GPIO_InitStruct.Pin = I_L3_F_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_L3_F_GPIO_Port, &GPIO_InitStruct);

    /* COMP6 interrupt Init */
    HAL_NVIC_SetPriority(COMP4_5_6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(COMP4_5_6_IRQn);
  /* USER CODE BEGIN COMP6_MspInit 1 */

  /* USER CODE END COMP6_MspInit 1 */
  }
  else if(compHandle->Instance==COMP7)
  {
  /* USER CODE BEGIN COMP7_MspInit 0 */

  /* USER CODE END COMP7_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**COMP7 GPIO Configuration
    PB14     ------> COMP7_INP
    */
    GPIO_InitStruct.Pin = I_LTOT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I_LTOT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP7_MspInit 1 */

  /* USER CODE END COMP7_MspInit 1 */
  }
}

void HAL_COMP_MspDeInit(COMP_HandleTypeDef* compHandle)
{

  if(compHandle->Instance==COMP1)
  {
  /* USER CODE BEGIN COMP1_MspDeInit 0 */

  /* USER CODE END COMP1_MspDeInit 0 */

    /**COMP1 GPIO Configuration
    PA1     ------> COMP1_INP
    */
    HAL_GPIO_DeInit(I_L1_F_GPIO_Port, I_L1_F_Pin);

    /* COMP1 interrupt Deinit */
  /* USER CODE BEGIN COMP1:COMP1_2_3_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP1_2_3_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn); */
  /* USER CODE END COMP1:COMP1_2_3_IRQn disable */

  /* USER CODE BEGIN COMP1_MspDeInit 1 */

  /* USER CODE END COMP1_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP2)
  {
  /* USER CODE BEGIN COMP2_MspDeInit 0 */

  /* USER CODE END COMP2_MspDeInit 0 */

    /**COMP2 GPIO Configuration
    PA3     ------> COMP2_INP
    */
    HAL_GPIO_DeInit(I_L1_FA3_GPIO_Port, I_L1_FA3_Pin);

    /* COMP2 interrupt Deinit */
  /* USER CODE BEGIN COMP2:COMP1_2_3_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP1_2_3_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn); */
  /* USER CODE END COMP2:COMP1_2_3_IRQn disable */

  /* USER CODE BEGIN COMP2_MspDeInit 1 */

  /* USER CODE END COMP2_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP3)
  {
  /* USER CODE BEGIN COMP3_MspDeInit 0 */

  /* USER CODE END COMP3_MspDeInit 0 */

    /**COMP3 GPIO Configuration
    PC1     ------> COMP3_INP
    */
    HAL_GPIO_DeInit(I_L2_F_GPIO_Port, I_L2_F_Pin);

    /* COMP3 interrupt Deinit */
  /* USER CODE BEGIN COMP3:COMP1_2_3_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP1_2_3_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn); */
  /* USER CODE END COMP3:COMP1_2_3_IRQn disable */

  /* USER CODE BEGIN COMP3_MspDeInit 1 */

  /* USER CODE END COMP3_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP4)
  {
  /* USER CODE BEGIN COMP4_MspDeInit 0 */

  /* USER CODE END COMP4_MspDeInit 0 */

    /**COMP4 GPIO Configuration
    PB0     ------> COMP4_INP
    */
    HAL_GPIO_DeInit(I_L2_FB0_GPIO_Port, I_L2_FB0_Pin);

    /* COMP4 interrupt Deinit */
  /* USER CODE BEGIN COMP4:COMP4_5_6_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP4_5_6_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP4_5_6_IRQn); */
  /* USER CODE END COMP4:COMP4_5_6_IRQn disable */

  /* USER CODE BEGIN COMP4_MspDeInit 1 */

  /* USER CODE END COMP4_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP5)
  {
  /* USER CODE BEGIN COMP5_MspDeInit 0 */

  /* USER CODE END COMP5_MspDeInit 0 */

    /**COMP5 GPIO Configuration
    PB13     ------> COMP5_INP
    */
    HAL_GPIO_DeInit(I_L3_FB13_GPIO_Port, I_L3_FB13_Pin);

    /* COMP5 interrupt Deinit */
  /* USER CODE BEGIN COMP5:COMP4_5_6_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP4_5_6_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP4_5_6_IRQn); */
  /* USER CODE END COMP5:COMP4_5_6_IRQn disable */

  /* USER CODE BEGIN COMP5_MspDeInit 1 */

  /* USER CODE END COMP5_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP6)
  {
  /* USER CODE BEGIN COMP6_MspDeInit 0 */

  /* USER CODE END COMP6_MspDeInit 0 */

    /**COMP6 GPIO Configuration
    PB11     ------> COMP6_INP
    */
    HAL_GPIO_DeInit(I_L3_F_GPIO_Port, I_L3_F_Pin);

    /* COMP6 interrupt Deinit */
  /* USER CODE BEGIN COMP6:COMP4_5_6_IRQn disable */
    /**
    * Uncomment the line below to disable the "COMP4_5_6_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(COMP4_5_6_IRQn); */
  /* USER CODE END COMP6:COMP4_5_6_IRQn disable */

  /* USER CODE BEGIN COMP6_MspDeInit 1 */

  /* USER CODE END COMP6_MspDeInit 1 */
  }
  else if(compHandle->Instance==COMP7)
  {
  /* USER CODE BEGIN COMP7_MspDeInit 0 */

  /* USER CODE END COMP7_MspDeInit 0 */

    /**COMP7 GPIO Configuration
    PB14     ------> COMP7_INP
    */
    HAL_GPIO_DeInit(I_LTOT_GPIO_Port, I_LTOT_Pin);

  /* USER CODE BEGIN COMP7_MspDeInit 1 */

  /* USER CODE END COMP7_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
