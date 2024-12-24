/* USER CODE BEGIN Header */
/**
  **********************************************************************************************************************
  * @file    lpbam_lpbamap1_scenario_config.c
  * @author  MCD Application Team
  * @brief   Provide LPBAM LpbamAp1 application Scenario configuration services.
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

/* Private typedef ---------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define --------------------------------------------------------------------------------------------------*/
#define TIMER_Q_IDX (0U)
#define ADC_Q_IDX (1U)
#define DMA_TIMEOUT_DURATION (0x1000U)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -----------------------------------------------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* External variables ------------------------------------------------------------------------------------------------*/
/* IP handler declaration */
extern ADC_HandleTypeDef hadc4;
extern LPTIM_HandleTypeDef hlptim1;

/* LPBAM queues declaration */
extern DMA_QListTypeDef TIMER_Q;
extern DMA_QListTypeDef ADC_Q;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private function prototypes ---------------------------------------------------------------------------------------*/

/* LPBAM ADC4 management APIs */
static void MX_ADC4_Init(void);
static void MX_ADC4_MspInit(ADC_HandleTypeDef *hadc4);
static void MX_ADC4_MspDeInit(ADC_HandleTypeDef *hadc4);
static void MX_ADC4_DeInit(void);

/* LPBAM LPTIM1 management APIs */
static void MX_LPTIM1_Init(void);
static void MX_LPTIM1_MspInit(LPTIM_HandleTypeDef *hlptim1);
static void MX_LPTIM1_MspDeInit(LPTIM_HandleTypeDef *hlptim1);
static void MX_LPTIM1_DeInit(void);

/* LPBAM autonomous mode management APIs */
static void MX_AutonomousMode_Init(void);
static void MX_AutonomousMode_DeInit(void);

/* LPBAM queue linking/unlinking APIs */
static void MX_TIMER_Q_Link(DMA_HandleTypeDef *hdma);
static void MX_TIMER_Q_UnLink(DMA_HandleTypeDef *hdma);
static void MX_ADC_Q_Link(DMA_HandleTypeDef *hdma);
static void MX_ADC_Q_UnLink(DMA_HandleTypeDef *hdma);

/* LPBAM DMA user callback APIs */
static void MX_ADC_Q_DMA_TC_Callback(DMA_HandleTypeDef *hdma);
/* LPBAM DMA NVIC API */
static void MX_DMA_NVIC_Config(DMA_HandleTypeDef *hdma, uint32_t PreemptPriority, uint32_t SubPriority);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ----------------------------------------------------------------------------------------------*/
/**
  * @brief  LpbamAp1 application Scenario scenario initialization
  * @param  None
  * @retval None
  */
void MX_LpbamAp1_Scenario_Init(void)
{
  /* LPBAM ADC4 initialization */
  MX_ADC4_Init();

  /* LPBAM LPTIM1 initialization */
  MX_LPTIM1_Init();

  /* Autonomous Mode initialization */
  MX_AutonomousMode_Init();

  /* USER CODE BEGIN LpbamAp1_Scenario_Init */

  /* USER CODE END LpbamAp1_Scenario_Init */
}

/**
  * @brief LpbamAp1 application Scenario scenario de-initialization
  * @param None
  * @retval None
  */
void MX_LpbamAp1_Scenario_DeInit(void)
{
  /* LPBAM ADC4 De-initialization */
  MX_ADC4_DeInit();

  /* LPBAM LPTIM1 De-initialization */
  MX_LPTIM1_DeInit();

  /* Autonomous mode de-initialization */
  MX_AutonomousMode_DeInit();

  /* USER CODE BEGIN LpbamAp1_Scenario_DeInit */

  /* USER CODE END LpbamAp1_Scenario_DeInit */
}

/**
  * @brief LpbamAp1 application Scenario scenario link
  * @param None
  * @retval None
  */
void MX_LpbamAp1_Scenario_Link(DMA_HandleTypeDef *hdma)
{
  /* USER CODE BEGIN LpbamAp1_Scenario_Link 0 */

  /* USER CODE END LpbamAp1_Scenario_Link 0 */

  /* Link TIMER queue to DMA channel */
  MX_TIMER_Q_Link(&hdma[TIMER_Q_IDX]);

  /* USER CODE BEGIN LINK TIMER_Q_IDX */

  /* USER CODE END LINK TIMER_Q_IDX */

  /* Link ADC queue to DMA channel */
  MX_ADC_Q_Link(&hdma[ADC_Q_IDX]);

  /* USER CODE BEGIN LINK ADC_Q_IDX */

  /* USER CODE END LINK ADC_Q_IDX */

  /* USER CODE BEGIN LpbamAp1_Scenario_Link 1 */

  /* USER CODE END LpbamAp1_Scenario_Link 1 */
}

/**
  * @brief LpbamAp1 application Scenario scenario unlink
  * @param hdma :Pointer to a DMA_HandleTypeDef structure that contains the configuration information for the specified
  *              DMA Channel
  * @retval None
  */
void MX_LpbamAp1_Scenario_UnLink(DMA_HandleTypeDef *hdma)
{
  /* USER CODE BEGIN LpbamAp1_Scenario_UnLink 0 */

  /* USER CODE END LpbamAp1_Scenario_UnLink 0 */

  /* LPBAM unLink TIMER queue to DMA channel */
  MX_TIMER_Q_UnLink(&hdma[TIMER_Q_IDX]);

  /* USER CODE BEGIN UNLINK TIMER_Q_IDX */

  /* USER CODE END UNLINK TIMER_Q_IDX */

  /* LPBAM unLink ADC queue to DMA channel */
  MX_ADC_Q_UnLink(&hdma[ADC_Q_IDX]);

  /* USER CODE BEGIN UNLINK ADC_Q_IDX */

  /* USER CODE END UNLINK ADC_Q_IDX */

  /* USER CODE BEGIN LpbamAp1_Scenario_UnLink 1 */

  /* USER CODE END LpbamAp1_Scenario_UnLink 1 */
}

/**
  * @brief LpbamAp1 application Scenario scenario start
  * @retval None
  */
void MX_LpbamAp1_Scenario_Start(DMA_HandleTypeDef *hdma)
{
  /* LPBAM start DMA channel in linked-list mode */
  if (HAL_DMAEx_List_Start(&hdma[TIMER_Q_IDX]) != HAL_OK)
  {
    Error_Handler();
  }

  /* LPBAM start DMA channel in linked-list mode */
  if (HAL_DMAEx_List_Start(&hdma[ADC_Q_IDX]) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN LpbamAp1_Scenario_Start */

  /* USER CODE END LpbamAp1_Scenario_Start */
}

/**
  * @brief LpbamAp1 application Scenario scenario stop
  * @retval None
  */
void MX_LpbamAp1_Scenario_Stop(DMA_HandleTypeDef *hdma)
{
  /* LPBAM stop DMA channel in linked-list mode */
  if ((hdma[TIMER_Q_IDX].State == HAL_DMA_STATE_BUSY) && (hdma[TIMER_Q_IDX].LinkedListQueue->FirstCircularNode != 0U))
  {
    if (HAL_DMA_Abort(&hdma[TIMER_Q_IDX]) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Check if DMA channel interrupt is enabled */
  if ((hdma[TIMER_Q_IDX].State == HAL_DMA_STATE_BUSY) && (__HAL_DMA_GET_IT_SOURCE(&hdma[TIMER_Q_IDX], DMA_IT_TC) == 0U))
  {
    if (HAL_DMA_PollForTransfer(&hdma[TIMER_Q_IDX], HAL_DMA_FULL_TRANSFER, DMA_TIMEOUT_DURATION) != HAL_OK)
    {
      Error_Handler();
    }
  }
  /* LPBAM stop DMA channel in linked-list mode */
  if ((hdma[ADC_Q_IDX].State == HAL_DMA_STATE_BUSY) && (hdma[ADC_Q_IDX].LinkedListQueue->FirstCircularNode != 0U))
  {
    if (HAL_DMA_Abort(&hdma[ADC_Q_IDX]) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* Check if DMA channel interrupt is enabled */
  if ((hdma[ADC_Q_IDX].State == HAL_DMA_STATE_BUSY) && (__HAL_DMA_GET_IT_SOURCE(&hdma[ADC_Q_IDX], DMA_IT_TC) == 0U))
  {
    if (HAL_DMA_PollForTransfer(&hdma[ADC_Q_IDX], HAL_DMA_FULL_TRANSFER, DMA_TIMEOUT_DURATION) != HAL_OK)
    {
      Error_Handler();
    }
  }

  /* USER CODE BEGIN LpbamAp1_Scenario_Stop */

  /* USER CODE END LpbamAp1_Scenario_Stop */
}

/**
  * @brief LpbamAp1 application Scenario autonomous mode init
  * @param None
  * @retval None
  */
static void MX_AutonomousMode_Init(void)
{
  /* Enable LPDMA1 Sleep Clock */
  __HAL_RCC_LPDMA1_CLK_SLEEP_ENABLE();
  /* Enable LPDMA1 Autonomous Mode */
  __HAL_RCC_LPDMA1_CLKAM_ENABLE();

  /* Enable SRAM4 Sleep Clock */
  __HAL_RCC_SRAM4_CLK_SLEEP_ENABLE();
  /* Enable SRAM4 Autonomous Mode */
  __HAL_RCC_SRAM4_CLKAM_ENABLE();

  /* Enable ADC4 Sleep Clock */
  __HAL_RCC_ADC4_CLK_SLEEP_ENABLE();
  /* Enable ADC4 Autonomous Mode */
  __HAL_RCC_ADC4_CLKAM_ENABLE();

  /* Enable LPTIM1 Sleep Clock */
  __HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE();
  /* Enable LPTIM1 Autonomous Mode */
  __HAL_RCC_LPTIM1_CLKAM_ENABLE();

  /* USER CODE BEGIN AutonomousMode_Init */

  /* USER CODE END AutonomousMode_Init */
}

/**
  * @brief  LpbamAp1 application Scenario autonomous mode deinit
  * @param  None
  * @retval None
  */
static void MX_AutonomousMode_DeInit(void)
{
  /* Disable LPDMA1 Sleep Clock */
  __HAL_RCC_LPDMA1_CLK_SLEEP_DISABLE();
  /* Disable LPDMA1 Autonomous Mode */
  __HAL_RCC_LPDMA1_CLKAM_DISABLE();

  /* Disable SRAM4 Sleep Clock */
  __HAL_RCC_SRAM4_CLK_SLEEP_DISABLE();
  /* Disable SRAM4 Autonomous Mode */
  __HAL_RCC_SRAM4_CLKAM_DISABLE();

  /* Disable ADC4 Sleep Clock */
  __HAL_RCC_ADC4_CLK_SLEEP_DISABLE();
  /* Disable ADC4 Autonomous Mode */
  __HAL_RCC_ADC4_CLKAM_DISABLE();

  /* Disable LPTIM1 Sleep Clock */
  __HAL_RCC_LPTIM1_CLK_SLEEP_DISABLE();
  /* Disable LPTIM1 Autonomous Mode */
  __HAL_RCC_LPTIM1_CLKAM_DISABLE();

  /* USER CODE BEGIN AutonomousMode_DeInit */

  /* USER CODE END AutonomousMode_DeInit */
}

/**
  * @brief ADC4 initialization.
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{
  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /* Set ADC4 instance */
  hadc4.Instance = ADC4;

  /* Register ADC msp callbacks */
  if (HAL_ADC_RegisterCallback(&hadc4, HAL_ADC_MSPINIT_CB_ID, MX_ADC4_MspInit) != HAL_OK)
  {
    Error_Handler();
  }
  /* Register ADC msp callbacks */
  if (HAL_ADC_RegisterCallback(&hadc4, HAL_ADC_MSPDEINIT_CB_ID, MX_ADC4_MspDeInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_AUTOFF_DPD;
  hadc4.Init.LowPowerAutoWait = ENABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC4_EXTERNALTRIG_LPTIM1_CH1;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.VrefProtection = ADC_VREF_PPROT_NONE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC4_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configuration in LPBAM context
  */
  /* USER CODE BEGIN ADC4_Init Calibration */

  /* USER CODE END ADC4_Init Calibration */
  if (ADV_LPBAM_ADC_EnableDMARequests(ADC4) != LPBAM_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief ADC4 de-initialization.
  * @param None
  * @retval None
  */
static void MX_ADC4_DeInit(void)
{
  /* USER CODE BEGIN ADC4_DeInit 0 */

  /* USER CODE END ADC4_DeInit 0 */

  /* Set ADC4 instance */
  hadc4.Instance = ADC4;

  /* UnRegister ADC msp callbacks */
  if (HAL_ADC_UnRegisterCallback(&hadc4, HAL_ADC_MSPINIT_CB_ID) != HAL_OK)
  {
    Error_Handler();
  }
  /* Init ADC4 peripheral */
  if (HAL_ADC_DeInit(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /* UnRegister ADC msp callbacks */
  if (HAL_ADC_UnRegisterCallback(&hadc4, HAL_ADC_MSPDEINIT_CB_ID) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC4_DeInit 1 */

  /* USER CODE END ADC4_DeInit 1 */
}

/**
  * @brief ADC4 MSP initialization.
  * @retval None
  */
static void MX_ADC4_MspInit(ADC_HandleTypeDef* adcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspInit 0 */
	  HAL_PWREx_EnableVddA();
  /* USER CODE END ADC4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC4 clock enable */
    __HAL_RCC_ADC4_CLK_ENABLE();

    /* ADC4 interrupt Init */
    HAL_NVIC_SetPriority(ADC4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC4_IRQn);
  /* USER CODE BEGIN ADC4_MspInit 1 */

  /* USER CODE END ADC4_MspInit 1 */
  }
}

/**
  * @brief ADC4 MSP de-initialization.
  * @retval None
  */
static void MX_ADC4_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if (adcHandle->Instance==ADC4)
  {
  /* USER CODE BEGIN ADC4_MspDeInit 0 */

  /* USER CODE END ADC4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC4_CLK_DISABLE();

    /* ADC4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC4_IRQn);
  /* USER CODE BEGIN ADC4_MspDeInit 1 */

  /* USER CODE END ADC4_MspDeInit 1 */
  }
}

/**
  * @brief LPTIM1 initialization.
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{
  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  LPTIM_OC_ConfigTypeDef sConfig1 = {0};

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */

  /* Set LPTIM1 instance */
  hlptim1.Instance = LPTIM1;

  /* Register LPTIM msp callbacks */
  if (HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_MSPINIT_CB_ID, MX_LPTIM1_MspInit) != HAL_OK)
  {
    Error_Handler();
  }
  /* Register LPTIM msp callbacks */
  if (HAL_LPTIM_RegisterCallback(&hlptim1, HAL_LPTIM_MSPDEINIT_CB_ID, MX_LPTIM1_MspDeInit) != HAL_OK)
  {
    Error_Handler();
  }
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.Period = 127;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig1.Pulse = 63;
  sConfig1.OCPolarity = LPTIM_OCPOLARITY_HIGH;
  if (HAL_LPTIM_OC_ConfigChannel(&hlptim1, &sConfig1, LPTIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configuration in LPBAM context
  */
  if (ADV_LPBAM_LPTIM_EnableDMARequests(LPTIM1, LPBAM_LPTIM_CHANNEL_1) != LPBAM_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief LPTIM1 de-initialization.
  * @param None
  * @retval None
  */
static void MX_LPTIM1_DeInit(void)
{
  /* USER CODE BEGIN LPTIM1_DeInit 0 */

  /* USER CODE END LPTIM1_DeInit 0 */

  /* Set LPTIM1 instance */
  hlptim1.Instance = LPTIM1;

  /* UnRegister LPTIM msp callbacks */
  if (HAL_LPTIM_UnRegisterCallback(&hlptim1, HAL_LPTIM_MSPINIT_CB_ID) != HAL_OK)
  {
    Error_Handler();
  }
  /* Init LPTIM1 peripheral */
  if (HAL_LPTIM_DeInit(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* UnRegister LPTIM msp callbacks */
  if (HAL_LPTIM_UnRegisterCallback(&hlptim1, HAL_LPTIM_MSPDEINIT_CB_ID) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN LPTIM1_DeInit 1 */

  /* USER CODE END LPTIM1_DeInit 1 */
}

/**
  * @brief LPTIM1 MSP initialization.
  * @retval None
  */
static void MX_LPTIM1_MspInit(LPTIM_HandleTypeDef* lptimHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(lptimHandle->Instance==LPTIM1)
  {
  /* USER CODE BEGIN LPTIM1_MspInit 0 */

  /* USER CODE END LPTIM1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* LPTIM1 clock enable */
    __HAL_RCC_LPTIM1_CLK_ENABLE();
  /* USER CODE BEGIN LPTIM1_MspInit 1 */

  /* USER CODE END LPTIM1_MspInit 1 */
  }
}

/**
  * @brief LPTIM1 MSP de-initialization.
  * @retval None
  */
static void MX_LPTIM1_MspDeInit(LPTIM_HandleTypeDef* lptimHandle)
{

  if (lptimHandle->Instance==LPTIM1)
  {
  /* USER CODE BEGIN LPTIM1_MspDeInit 0 */

  /* USER CODE END LPTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM1_CLK_DISABLE();
  /* USER CODE BEGIN LPTIM1_MspDeInit 1 */

  /* USER CODE END LPTIM1_MspDeInit 1 */
  }
}

/**
  * @brief  TIMER queue link
  * @retval None
  */
static void MX_TIMER_Q_Link(DMA_HandleTypeDef *hdma)
{
  /* Enable LPDMA1 clock */
  __HAL_RCC_LPDMA1_CLK_ENABLE();

  hdma->InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  hdma->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_NORMAL;
  if (HAL_DMAEx_List_Init(hdma) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(hdma, &TIMER_Q) != HAL_OK)
  {
    Error_Handler();
  }

  /* Register DMA channel error callbacks */
}

/**
  * @brief  TIMER queue unlink
  * @retval None
  */
static void MX_TIMER_Q_UnLink(DMA_HandleTypeDef *hdma)
{
  /* UnLink TIMER queue to DMA channel */
  if (HAL_DMAEx_List_UnLinkQ(hdma) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA linked list de-init */
  if (HAL_DMAEx_List_DeInit(hdma) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  ADC queue link
  * @retval None
  */
static void MX_ADC_Q_Link(DMA_HandleTypeDef *hdma)
{
  /* Enable LPDMA1 clock */
  __HAL_RCC_LPDMA1_CLK_ENABLE();

  hdma->InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma->InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma->InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  hdma->InitLinkedList.LinkedListMode = DMA_LINKEDLIST_NORMAL;
  if (HAL_DMAEx_List_Init(hdma) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMAEx_List_LinkQ(hdma, &ADC_Q) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);

  /* Register DMA channel error callbacks */
  if (HAL_DMA_RegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID, MX_ADC_Q_DMA_TC_Callback) != HAL_OK)
  {
    Error_Handler();
  }
  MX_DMA_NVIC_Config(hdma, 0, 0);
}

/**
  * @brief  ADC queue unlink
  * @retval None
  */
static void MX_ADC_Q_UnLink(DMA_HandleTypeDef *hdma)
{
  /* UnLink ADC queue to DMA channel */
  if (HAL_DMAEx_List_UnLinkQ(hdma) != HAL_OK)
  {
    Error_Handler();
  }

  /* Register DMA channel transfer complete callbacks */
  if (HAL_DMA_UnRegisterCallback(hdma, HAL_DMA_XFER_CPLT_CB_ID) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA linked list de-init */
  if (HAL_DMAEx_List_DeInit(hdma) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  ADC queue dma transfer complete callbacks
  * @retval None
  */
static void MX_ADC_Q_DMA_TC_Callback(DMA_HandleTypeDef *hdma)
{
  /* USER CODE BEGIN ADC_DMA_TC_Callback */
	HAL_LPTIM_PWM_Stop(&hlptim1, LPBAM_LPTIM_CHANNEL_1);
  /* USER CODE END ADC_DMA_TC_Callback */
}

/* USER CODE BEGIN LpbamAp1_Scenario_Config */

/* USER CODE END LpbamAp1_Scenario_Config */

/**
  * @brief DMA channel NVIC configuration
  * @retval None
  */
static void MX_DMA_NVIC_Config(DMA_HandleTypeDef *hdma, uint32_t PreemptPriority, uint32_t SubPriority)
{
  IRQn_Type irq = LPDMA1_Channel0_IRQn;

  /* Check DMA channel instance */
  switch ((uint32_t)hdma->Instance)
  {
    case (uint32_t)LPDMA1_Channel0: /* DMA channel_0 */
    {
      irq = LPDMA1_Channel0_IRQn;
      break;
    }

    case (uint32_t)LPDMA1_Channel1: /* DMA channel_1 */
    {
      irq = LPDMA1_Channel1_IRQn;
      break;
    }

    case (uint32_t)LPDMA1_Channel2: /* DMA channel_2 */
    {
      irq = LPDMA1_Channel2_IRQn;
      break;
    }

    case (uint32_t)LPDMA1_Channel3: /* DMA channel_3 */
    {
      irq = LPDMA1_Channel3_IRQn;
      break;
    }
  }

  /* Enable NVIC for DMA channel */
  HAL_NVIC_SetPriority(irq, PreemptPriority, SubPriority);
  HAL_NVIC_EnableIRQ(irq);
}
