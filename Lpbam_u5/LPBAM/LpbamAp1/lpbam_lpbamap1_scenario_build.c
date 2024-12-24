/* USER CODE BEGIN Header */
/**
  **********************************************************************************************************************
  * @file   lpbam_lpbamap1_scenario_build.c
  * @author MCD Application Team
  * @brief  Provides LPBAM LpbamAp1 application Scenario scenario build services
  **********************************************************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********************************************************************************************************************
  */
/* USER CODE END Header */
/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "lpbam_lpbamap1.h"

/* Private variables -------------------------------------------------------------------------------------------------*/
/* LPBAM variables declaration */
/* USER CODE BEGIN LpbamAp1_Scenario_Descs 0 */

/* USER CODE END LpbamAp1_Scenario_Descs 0 */

/* USER CODE BEGIN TIMER_Q_Start_1_Desc */

/* USER CODE END TIMER_Q_Start_1_Desc */
static LPBAM_LPTIM_StartFullDesc_t TIMER_Q_Start_1_Desc;

/* USER CODE BEGIN TIMER_Q_PWM_1_Desc */

/* USER CODE END TIMER_Q_PWM_1_Desc */
static LPBAM_LPTIM_PWMFullDesc_t TIMER_Q_PWM_1_Desc;

/* USER CODE BEGIN TIMER_Q_PWM_2_Desc */

/* USER CODE END TIMER_Q_PWM_2_Desc */
static LPBAM_LPTIM_PWMFullDesc_t TIMER_Q_PWM_2_Desc;

/* USER CODE BEGIN ADC_Q_Conversion_data_1_Desc */

/* USER CODE END ADC_Q_Conversion_data_1_Desc */
static LPBAM_ADC_ConvDataDesc_t ADC_Q_Conversion_data_1_Desc;

/* USER CODE BEGIN LpbamAp1_Scenario_Descs 1 */

/* USER CODE END LpbamAp1_Scenario_Descs 1 */

/* Exported variables ------------------------------------------------------------------------------------------------*/
/* LPBAM queues declaration */
DMA_QListTypeDef TIMER_Q;
DMA_QListTypeDef ADC_Q;

/* External variables ------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern uint16_t Data_Sequence[320];
/* USER CODE END EV */

/* Private function prototypes ---------------------------------------------------------------------------------------*/
static void MX_TIMER_Q_Build(void);
static void MX_ADC_Q_Build(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ------------------------------------------------------------------------------------------------*/
/**
  * @brief LpbamAp1 application Scenario scenario build
  * @param None
  * @retval None
  */
void MX_LpbamAp1_Scenario_Build(void)
{
  /* USER CODE BEGIN LpbamAp1_Scenario_Build 0 */

  /* USER CODE END LpbamAp1_Scenario_Build 0 */

  /* LPBAM build TIMER queue */
  MX_TIMER_Q_Build();
  /* LPBAM build ADC queue */
  MX_ADC_Q_Build();

  /* USER CODE BEGIN LpbamAp1_Scenario_Build 1 */

  /* USER CODE END LpbamAp1_Scenario_Build 1 */
}

/* Private functions -------------------------------------------------------------------------------------------------*/

/**
  * @brief  LpbamAp1 application Scenario scenario TIMER queue build
  * @param  None
  * @retval None
  */
static void MX_TIMER_Q_Build(void)
{
  /* LPBAM build variable */
  LPBAM_DMAListInfo_t pDMAListInfo_LPTIM = {0};
  LPBAM_LPTIM_StartFullAdvConf_t pStartFull_LPTIM = {0};
  LPBAM_LPTIM_PWMFullAdvConf_t pPWMFull_LPTIM = {0};

  /**
    * TIMER queue Start_1 build
    */
  pDMAListInfo_LPTIM.QueueType = LPBAM_LINEAR_ADDRESSING_Q;
  pDMAListInfo_LPTIM.pInstance = LPDMA1;
  pStartFull_LPTIM.StartMode = LPBAM_LPTIM_START_CONTINUOUS;
  pStartFull_LPTIM.WakeupIT = LPBAM_LPTIM_IT_NONE;
  if (ADV_LPBAM_LPTIM_Start_SetFullQ(LPTIM1, &pDMAListInfo_LPTIM, &pStartFull_LPTIM, &TIMER_Q_Start_1_Desc, &TIMER_Q) != LPBAM_OK)
  {
    Error_Handler();
  }

  /**
    * TIMER queue PWM_1 build
    */
  pPWMFull_LPTIM.UpdatePeriod = ENABLE;
  pPWMFull_LPTIM.PeriodValue = 127;
  pPWMFull_LPTIM.UpdatePulse = ENABLE;
  pPWMFull_LPTIM.PulseValue = 63;
  pPWMFull_LPTIM.UpdateRepetition = ENABLE;
  pPWMFull_LPTIM.RepetitionValue = 255;
  if (ADV_LPBAM_LPTIM_PWM_SetFullQ(LPTIM1, LPBAM_LPTIM_CHANNEL_1, &pDMAListInfo_LPTIM, &pPWMFull_LPTIM, &TIMER_Q_PWM_1_Desc, &TIMER_Q) != LPBAM_OK)
  {
    Error_Handler();
  }

  /**
    * TIMER queue PWM_2 build
    */
  pPWMFull_LPTIM.PeriodValue = 511;
  pPWMFull_LPTIM.PulseValue = 255;
  pPWMFull_LPTIM.RepetitionValue = 63;
  if (ADV_LPBAM_LPTIM_PWM_SetFullQ(LPTIM1, LPBAM_LPTIM_CHANNEL_1, &pDMAListInfo_LPTIM, &pPWMFull_LPTIM, &TIMER_Q_PWM_2_Desc, &TIMER_Q) != LPBAM_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  LpbamAp1 application Scenario scenario ADC queue build
  * @param  None
  * @retval None
  */
static void MX_ADC_Q_Build(void)
{
  /* LPBAM build variable */
  LPBAM_DMAListInfo_t pDMAListInfo_ADC = {0};
  LPBAM_ADC_DataAdvConf_t pConvData_ADC = {0};

  /**
    * ADC queue Conversion_data_1 build
    */
  pDMAListInfo_ADC.QueueType = LPBAM_LINEAR_ADDRESSING_Q;
  pDMAListInfo_ADC.pInstance = LPDMA1;
  pConvData_ADC.DMAContinuousRequests = DISABLE;
  pConvData_ADC.Size = 320;
  pConvData_ADC.pData = (uint32_t*)&Data_Sequence[0];
  if (ADV_LPBAM_ADC_Conversion_SetDataQ(ADC4, &pDMAListInfo_ADC, &pConvData_ADC, &ADC_Q_Conversion_data_1_Desc, &ADC_Q) != LPBAM_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN LpbamAp1_Scenario_Build */

/* USER CODE END LpbamAp1_Scenario_Build */
