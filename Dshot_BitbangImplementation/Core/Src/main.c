/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DshotBitbang.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t value;
uint16_t rpm;
volatile uint8_t TXFull;

typedef enum {
    STATE_TX_INIT,              // Preparing to start TX
    STATE_TX_TRANSFERING,              // TX transfer in progress
    STATE_RX_INIT,              // RX transfer in progress
    STATE_RX_TRANSFERING,  // RX done, preparing for next TX (or back to IDLE)
} AppState_t;

volatile AppState_t appState = STATE_TX_INIT; // Global state variable

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

/* USER CODE BEGIN PV */
uint32_t DMA_Buffer[FRAME_BIT_NUMBER * BIT_SECTION_NUMBER] = {0};
uint32_t RX_Buffer[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma->Instance == DMA2_Stream5)
    {
    	/*
        GPIOE->MODER &= ~GPIO_MODER_MODER9;
        GPIOE->PUPDR |= GPIO_PUPDR_PUPDR9_0;
        DMA2_Stream5->CR |= DMA_SxCR_DIR_1;
        HAL_TIM_Base_Start(&htim1);
        HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)RxBuffer, (uint32_t)&(GPIOE->IDR), 105);
		//TIM1->EGR |= TIM_EGR_UG;
		HAL_TIM_Base_Start(&htim1);*/
    	if (appState == STATE_TX_TRANSFERING) appState = STATE_RX_INIT;
    	else if (appState == STATE_RX_TRANSFERING) appState = STATE_TX_INIT;


    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  hdma_tim1_up.XferCpltCallback = HAL_DMA_TxCpltCallback;


  MemBuffer_init(DMA_Buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  appState = STATE_TX_INIT;
  while (1)
  {
	  	value = 48;

	  	DshotFrameToMemBuffer(getDshotFrame(value), DMA_Buffer);
	  	rpm = read_BDshot_response(get_BDshot_response(RX_Buffer, 9));
		switch (appState){
			case STATE_TX_INIT:
				//Tat timer
				HAL_TIM_Base_Stop(&htim1);

				//Chuyen GPIO sang output, no pullup pulldown
				GPIOE->MODER &= ~GPIO_MODER_MODER9;
	            GPIOE->MODER |= GPIO_MODER_MODER9_0; // Output mode (01)
	            //Tat DMA
			    __HAL_DMA_DISABLE(&hdma_tim1_up);

				//Cai dat cho DMA
				hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
				hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
				hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
				hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				HAL_DMA_Init(&hdma_tim1_up);

				//Bat dma va timer
				HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)DMA_Buffer, (uint32_t)&(GPIOE->BSRR), FRAME_BIT_NUMBER * BIT_SECTION_NUMBER);
				TIM1->EGR |= TIM_EGR_UG;
				HAL_TIM_Base_Start(&htim1);
				appState = STATE_TX_TRANSFERING;
				break;

			case STATE_TX_TRANSFERING:
				break;

			case STATE_RX_INIT:
				//Tat timer
				HAL_TIM_Base_Stop(&htim1);
				//Chuyen gpio sang input mode, pull up
				GPIOE->MODER &= ~GPIO_MODER_MODER9;
	            GPIOE->PUPDR &= ~GPIO_PUPDR_PUPDR9; // Clear pull-up/down bits
	            GPIOE->PUPDR |= GPIO_PUPDR_PUPDR9_0; // Example: Enable Pull-up (01)

				//Cai dat lai DMA
				hdma_tim1_up.Init.Direction = DMA_PERIPH_TO_MEMORY;
				hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
				hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
				hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
				hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
				HAL_DMA_Init(&hdma_tim1_up);

				//Bat dma va timer
				HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)&(GPIOE->IDR), (uint32_t)RX_Buffer, 200);
				TIM1->EGR |= TIM_EGR_UG;
				HAL_TIM_Base_Start(&htim1);
				appState = STATE_RX_TRANSFERING;
				GPIOE->ODR |= GPIO_ODR_OD9;
				break;

			case STATE_RX_TRANSFERING:
				break;


		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 168000/BitRate/BitSectionNumber -1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
