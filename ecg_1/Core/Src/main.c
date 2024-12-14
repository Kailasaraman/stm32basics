/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#define dlr  0x50;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
#define ADS1293_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)  // Adjust pin as needed
#define ADS1293_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

#define DRDYB_PIN          GPIO_PIN_10  // Adjust pin as needed
#define DRDYB_PORT         GPIOB

// ADS1293 Register Addresses
#define REG_CH_CNFG        0x2F
#define REG_DRDYB_SRC      0x27
#define REG_MASK_DRDYB     0x20
#define REG_DATA_LOOP      0x50
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */




  void ADS1293_WriteRegister(uint8_t reg, uint8_t value) {
      uint8_t txData[2] = {reg & 0x7F, value}; // MSB = 0 for write
      ADS1293_CS_LOW();
      HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
      ADS1293_CS_HIGH();
  }

  /**
   * @brief Read a single byte from an ADS1293 register.
   */
  uint8_t ADS1293_ReadRegister(uint8_t reg) {
      uint8_t txData[2] = {reg | 0x80, 0x00}; // MSB = 1 for read
      uint8_t rxData[2] = {0};
      ADS1293_CS_LOW();
      HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, HAL_MAX_DELAY);
      ADS1293_CS_HIGH();
      return rxData[1];
  }

  /**
   * @brief Initialize the ADS1293 for streaming mode.
   */
  void ADS1293_Init(void) {
      // Stop data conversion
      ADS1293_WriteRegister(0x00, 0x00);

      // Enable Channel 1 PACE, Channel 1 ECG, and Channel 2 ECG for loop read-back mode
      ADS1293_WriteRegister(REG_CH_CNFG, 0x32);

      // Configure DRDYB source to Channel 1 PACE
      ADS1293_WriteRegister(REG_DRDYB_SRC, 0x01);

      // Start data conversion
      ADS1293_WriteRegister(0x00, 0x01);
  }

  /**
   * @brief Read data from the ADS1293 DATA_LOOP register.
   * @param buffer Pointer to buffer to store data.
   * @param length Number of bytes to read.
   */
  void ADS1293_ReadData(uint8_t *buffer, uint8_t length) {
      uint8_t reg = REG_DATA_LOOP | 0x80; // MSB = 1 for read
      ADS1293_CS_LOW();
      HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive(&hspi1, buffer, length, HAL_MAX_DELAY);
      ADS1293_CS_HIGH();
  }


  void UART_PrintData(uint8_t *data, uint8_t length) {
      char msg[64];
      for (uint8_t i = 0; i < length; i++) {
          snprintf(msg, sizeof(msg), "0x%02X\r\n", data[i]);
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      }
      snprintf(msg, sizeof(msg), "\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }




  ADS1293_Init();

  uint8_t dataBuffer[8] = {0};


 /* void ADS1293_WriteRegister(uint8_t regAddress, uint8_t data)
  {
      uint8_t txBuffer[2];
      txBuffer[0] = regAddress;      // Register address
      txBuffer[1] = data;            // Data to write
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Assert CSB
      HAL_SPI_Transmit(&hspi1, txBuffer, 2, HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Deassert CSB
  }




  // Function to read a block of data from ADS1293 in streaming mode
  void ADS1293_ReadStream(uint8_t *dataBuffer, uint16_t length)
  {
      uint8_t regAddress = DATA_LOOP_REG;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Assert CSB
      HAL_SPI_Transmit(&hspi1, &regAddress, 1, HAL_MAX_DELAY); // Send the address
      HAL_SPI_Receive(&hspi1, dataBuffer, length, HAL_MAX_DELAY); // Read the data
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Deassert CSB
  }



  // Step 1: Write 0x49 to the CH_CNFG register
  ADS1293_WriteRegister(CH_CNFG_REG, 0x49);

  // Buffer to store the data from DATA_LOOP register
  uint8_t dataBuffer[6];
  void UART2_SendDataHex(uint8_t *data, uint16_t length)
  {
      char buffer[32]; // Temporary buffer for formatted data
      for (uint16_t i = 0; i < length; i++)
      {
          // Format each byte as two-character hex and send it
          sprintf(buffer, "%02X ", data[i]);
          HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
      }

      char newline[] = "\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)newline, strlen(newline), HAL_MAX_DELAY);
  }*/




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  while (HAL_GPIO_ReadPin(DRDYB_PORT, DRDYB_PIN) == GPIO_PIN_SET);

	          // Read 12 bytes of data from DATA_LOOP register
	          ADS1293_ReadData(dataBuffer, 8);
	          UART_PrintData(dataBuffer, 8);

	  HAL_Delay(1000);


	 /* HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, co[0], sizeof(co[0]), HAL_MAX_DELAY);
	  HAL_SPI_Transmit(&hspi1, co[1], sizeof(co[1]), HAL_MAX_DELAY);
	  //HAL_Delay(4);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, ch[0], sizeof(ch[0]), HAL_MAX_DELAY);
	  HAL_SPI_Transmit(&hspi1, ch[1], sizeof(ch[1]), HAL_MAX_DELAY);

	//  HAL_Delay(4);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, dr[0], sizeof(dr[0]), HAL_MAX_DELAY);
	  HAL_SPI_Transmit(&hspi1, dr[1], sizeof(dr[1]), HAL_MAX_DELAY);
	  //HAL_Delay(4);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi1, st[0], sizeof(st[0]), HAL_MAX_DELAY);
	  HAL_SPI_Transmit(&hspi1, st[1], sizeof(st[1]), HAL_MAX_DELAY);
	 // HAL_Delay(4);
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	  HAL_SPI_Transmit(&hspi1, dlr, sizeof(dlr), HAL_MAX_DELAY);
//HAL_SPI_TransmitReceive(&hspi1, &dlr, recv, sizeof(recv), HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, (uint8_t *)recv, sizeof(recv), HAL_MAX_DELAY);

	 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	  //HAL_SPI_Receive(&hspi1, 0x32, sizeof(0x32), HAL_MAX_DELAY);
	  //uint8_t val =
	// HAL_UART_Transmit(&huart2, (uint8_t*)recv,sizeof(recv), HAL_MAX_DELAY);
	 //HAL_SPI-
	  //HAL_UART_Transmit(huart, pData, Size, Timeout)*/



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_button_Pin */
  GPIO_InitStruct.Pin = User_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_LED_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
