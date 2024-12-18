/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32u0xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief LCD MSP Initialization
* This function configures the hardware resources used in this example
* @param hlcd: LCD handle pointer
* @retval None
*/
void HAL_LCD_MspInit(LCD_HandleTypeDef* hlcd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hlcd->Instance==LCD)
  {
  /* USER CODE BEGIN LCD_MspInit 0 */

  /* USER CODE END LCD_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_LCD_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**LCD GPIO Configuration
    PC4     ------> LCD_SEG22
    PC5     ------> LCD_SEG23
    PB1     ------> LCD_SEG6
    PE7     ------> LCD_SEG45
    PE8     ------> LCD_SEG46
    PE9     ------> LCD_SEG47
    PB11     ------> LCD_SEG11
    PB14     ------> LCD_SEG14
    PB15     ------> LCD_SEG15
    PD8     ------> LCD_SEG28
    PD9     ------> LCD_SEG29
    PD12     ------> LCD_SEG32
    PD13     ------> LCD_SEG33
    PC6     ------> LCD_SEG24
    PC8     ------> LCD_SEG26
    PC9     ------> LCD_SEG27
    PA8     ------> LCD_COM0
    PA9     ------> LCD_COM1
    PA10     ------> LCD_COM2
    PC10     ------> LCD_SEG48
    PC11     ------> LCD_SEG49
    PD0     ------> LCD_SEG34
    PD1     ------> LCD_SEG35
    PD3     ------> LCD_SEG36
    PD4     ------> LCD_SEG37
    PD5     ------> LCD_SEG38
    PD6     ------> LCD_SEG39
    PB9     ------> LCD_COM3
    */
    GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG13_Pin|SEG14_Pin
                          |SEG15_Pin|SEG16_Pin|SEG23_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SEG2_Pin|SEG6_Pin|SEG7_Pin|SEG8_Pin
                          |COM3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SEG3_Pin|SEG4_Pin|SEG5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SEG9_Pin|SEG10_Pin|SEG11_Pin|SEG12_Pin
                          |SEG17_Pin|SEG18_Pin|SEG19_Pin|SEG20_Pin
                          |SEG21_Pin|SEG22_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = COM0_Pin|COM1_Pin|COM2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN LCD_MspInit 1 */

  /* USER CODE END LCD_MspInit 1 */
  }

}

/**
* @brief LCD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hlcd: LCD handle pointer
* @retval None
*/
void HAL_LCD_MspDeInit(LCD_HandleTypeDef* hlcd)
{
  if(hlcd->Instance==LCD)
  {
  /* USER CODE BEGIN LCD_MspDeInit 0 */

  /* USER CODE END LCD_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LCD_CLK_DISABLE();

    /**LCD GPIO Configuration
    PC4     ------> LCD_SEG22
    PC5     ------> LCD_SEG23
    PB1     ------> LCD_SEG6
    PE7     ------> LCD_SEG45
    PE8     ------> LCD_SEG46
    PE9     ------> LCD_SEG47
    PB11     ------> LCD_SEG11
    PB14     ------> LCD_SEG14
    PB15     ------> LCD_SEG15
    PD8     ------> LCD_SEG28
    PD9     ------> LCD_SEG29
    PD12     ------> LCD_SEG32
    PD13     ------> LCD_SEG33
    PC6     ------> LCD_SEG24
    PC8     ------> LCD_SEG26
    PC9     ------> LCD_SEG27
    PA8     ------> LCD_COM0
    PA9     ------> LCD_COM1
    PA10     ------> LCD_COM2
    PC10     ------> LCD_SEG48
    PC11     ------> LCD_SEG49
    PD0     ------> LCD_SEG34
    PD1     ------> LCD_SEG35
    PD3     ------> LCD_SEG36
    PD4     ------> LCD_SEG37
    PD5     ------> LCD_SEG38
    PD6     ------> LCD_SEG39
    PB9     ------> LCD_COM3
    */
    HAL_GPIO_DeInit(GPIOC, SEG0_Pin|SEG1_Pin|SEG13_Pin|SEG14_Pin
                          |SEG15_Pin|SEG16_Pin|SEG23_Pin);

    HAL_GPIO_DeInit(GPIOB, SEG2_Pin|SEG6_Pin|SEG7_Pin|SEG8_Pin
                          |COM3_Pin);

    HAL_GPIO_DeInit(GPIOE, SEG3_Pin|SEG4_Pin|SEG5_Pin);

    HAL_GPIO_DeInit(GPIOD, SEG9_Pin|SEG10_Pin|SEG11_Pin|SEG12_Pin
                          |SEG17_Pin|SEG18_Pin|SEG19_Pin|SEG20_Pin
                          |SEG21_Pin|SEG22_Pin);

    HAL_GPIO_DeInit(GPIOA, COM0_Pin|COM1_Pin|COM2_Pin);

  /* USER CODE BEGIN LCD_MspDeInit 1 */

  /* USER CODE END LCD_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
