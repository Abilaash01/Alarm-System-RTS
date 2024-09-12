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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"


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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for ReadInputTask */
osThreadId_t ReadInputTaskHandle;
const osThreadAttr_t ReadInputTask_attributes = {
  .name = "ReadInputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WriteInputTask */
osThreadId_t WriteInputTaskHandle;
const osThreadAttr_t WriteInputTask_attributes = {
  .name = "WriteInputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RedLight */
osThreadId_t RedLightHandle;
const osThreadAttr_t RedLight_attributes = {
  .name = "RedLight",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GreenLight */
osThreadId_t GreenLightHandle;
const osThreadAttr_t GreenLight_attributes = {
  .name = "GreenLight",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadSensorTask */
osThreadId_t ReadSensorTaskHandle;
const osThreadAttr_t ReadSensorTask_attributes = {
  .name = "ReadSensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
extern char key;
char hold[5];
char ast[5];
size_t currentIndex = 0;
int flag = 0;
extern GPIO_PinState motionStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartRedTask(void *argument);
void StartGreenTask(void *argument);
void readsensor(void *argument);

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
    SSD1306_Init();
    SSD1306_GotoXY (0,0);
    //SSD1306_Puts ("Voltage:", &Font_11x18, 1);
    SSD1306_Puts ("Enter Code:", &Font_11x18, 1);
    SSD1306_GotoXY (0, 30);
    SSD1306_UpdateScreen();
    SSD1306_UpdateScreen();
    HAL_Delay (500);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadInputTask */
  ReadInputTaskHandle = osThreadNew(StartDefaultTask, NULL, &ReadInputTask_attributes);

  /* creation of WriteInputTask */
  WriteInputTaskHandle = osThreadNew(StartTask02, NULL, &WriteInputTask_attributes);

  /* creation of RedLight */
  RedLightHandle = osThreadNew(StartRedTask, NULL, &RedLight_attributes);

  /* creation of GreenLight */
  GreenLightHandle = osThreadNew(StartGreenTask, NULL, &GreenLight_attributes);

  /* creation of ReadSensorTask */
  ReadSensorTaskHandle = osThreadNew(readsensor, NULL, &ReadSensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	/* D10 to D7 as input pins for row 0 to row 3. D6 to D3 as output for column pins C1 to C3*/
//	  sprintf(hold, "%c", key);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100);
//	  SSD1306_GotoXY (0, 30);
//	  SSD1306_UpdateScreen();
//	  SSD1306_Puts (hold, &Font_11x18, 1);
//	  SSD1306_UpdateScreen();
//	  HAL_Delay (500);
//  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KC0_Pin KC3_Pin KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KR1_Pin */
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KR0_Pin */
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the ReadInputTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	key = Get_Key();
	if(currentIndex == 4) {
		currentIndex = 0;
		memset(hold, 0, sizeof hold);
		memset(ast, 0, sizeof ast);
		ast[currentIndex] = '*';
		hold[currentIndex++] = key;
	}
	else {
		ast[currentIndex] = '*';
		hold[currentIndex++] = key;
	}

	if (hold[0] == '1' && hold[1] == '2' && hold[2] == '3' && hold[3] == '4') {
		flag = !flag;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
//		osDelay(3000);
	}

//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
   	// HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100);
    osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the WriteInputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if(currentIndex == 4) {
		  SSD1306_GotoXY (0, 0);
		  SSD1306_Puts ("Enter Code:", &Font_11x18, 1);
		  SSD1306_GotoXY (0, 30);
		  SSD1306_Puts ("             ", &Font_11x18, 1);
		  SSD1306_UpdateScreen();
	  } else {
		  SSD1306_GotoXY (currentIndex + 6, 30);
		  SSD1306_UpdateScreen();
		  SSD1306_Puts (ast, &Font_11x18, 1);
		  SSD1306_UpdateScreen();
	  }

	  osDelay(1);
//	  SSD1306_GotoXY (currentIndex + 4, 30);
//	  SSD1306_UpdateScreen();
//	  SSD1306_Puts (hold, &Font_11x18, 1);
//	  SSD1306_UpdateScreen();
//	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartRedTask */
/**
* @brief Function implementing the RedLight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRedTask */
void StartRedTask(void *argument)
{
  /* USER CODE BEGIN StartRedTask */
  /* Infinite loop */
  for(;;)
  {
	if(!flag) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    	osDelay(1);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		osDelay(1);
	}
  }
  /* USER CODE END StartRedTask */
}

/* USER CODE BEGIN Header_StartGreenTask */
/**
* @brief Function implementing the GreenLight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenTask */
void StartGreenTask(void *argument)
{
  /* USER CODE BEGIN StartGreenTask */
  /* Infinite loop */
  for(;;)
  {
	  if(!flag) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		  osDelay(1);
	  } else {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		  osDelay(1);
	  }
  }
  /* USER CODE END StartGreenTask */
}

/* USER CODE BEGIN Header_readsensor */
/**
* @brief Function implementing the ReadSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readsensor */
void readsensor(void *argument)
{
  /* USER CODE BEGIN readsensor */
	for(;;)
	  {
//		char statusStr[2];
//	    GPIO_PinState motionStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
//	    snprintf(statusStr, sizeof(statusStr), "%d", motionStatus == 1 ? 1 : 0);
//	    HAL_UART_Transmit(&huart2, (uint8_t *)statusStr, strlen(statusStr), 100);
//
//	    if(motionStatus == 1) {
//			// Motion detected, turn on the buzzer
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
//	    } else {
//	    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
//	    }
//
//	    // Add a delay to prevent the loop from running too fast
//	    osDelay(100); // Adjust the delay as needed

		if(flag) {
			osDelay(3000);
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		}

		osDelay(50);
	  }
  /* USER CODE END readsensor */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
