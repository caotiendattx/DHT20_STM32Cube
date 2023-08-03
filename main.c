/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	uint8_t UARTbuf[100];
	uint8_t count = 0;
	float temp;
	float humm;
	DHT20_Custom_Init();
	uint8_t buf5[1];
	buf5[0] =  0x00;
	if(HAL_I2C_Master_Transmit(&hi2c3, (0x38 << 1), buf5, 1, HAL_MAX_DELAY) == HAL_ERROR){

		    strcpy((char*)UARTbuf, "HAL_ERROR! Sending 1 Byte Error \r\n");
		   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	}
	if(HAL_I2C_Master_Transmit(&hi2c3, (0x38 << 1), buf5, 1, HAL_MAX_DELAY) == HAL_BUSY){

		    strcpy((char*)UARTbuf, "HAL_BUSY! Sending 1 Byte Error \r\n");
		   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	}
	if(HAL_I2C_Master_Transmit(&hi2c3, (0x38 << 1), buf5, 1, HAL_MAX_DELAY) == HAL_TIMEOUT){

		    strcpy((char*)UARTbuf, "HAL_TIMEOUT! Sending 1 Byte Error \r\n");
		   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	}
	if(HAL_I2C_Master_Transmit(&hi2c3, (0x38 << 1), buf5, 1, HAL_MAX_DELAY) == HAL_OK){

		    strcpy((char*)UARTbuf, "HAL_OK! Sending 1 Byte Error \r\n");
		   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	}
	if(!isConnected()){
	    strcpy((char*)UARTbuf, "Connection Error! \r\n");
	   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	}

	  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GetTick() - lastRead() >= 1000)
	  {
		  uint8_t status = read();

	    if ((count % 10) == 0)
	    {
	      count = 0;
	      strcpy((char*)UARTbuf, "Type\tHumidity (%)\tTemp (°C)\tStatus\r\n\0");
	      HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	    }
	    count++;

	    strcpy((char*)UARTbuf, "DHT20 \t");
	   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	    // DISPLAY DATA, sensor has only one decimal.
	   	humm = getHumidity();
	   	sprintf(UARTbuf, "%u.%02u", (unsigned int)humm, (unsigned int)((humm - (unsigned int)humm) * 100));
	   	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	    strcpy((char*)UARTbuf, "\t\t");
	    HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);

	    temp = getTemperature();
	 	sprintf(UARTbuf, "%u.%02u", (unsigned int)temp, (unsigned int)((temp - (unsigned int)temp) * 100));
	 	HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	    strcpy((char*)UARTbuf, "\t\t");
	    HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	    switch (status)
	    {
	      case DHT20_OK:
	    		strcpy((char*)UARTbuf, "OK");
	    		HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_ERROR_CHECKSUM:
	    	  strcpy((char*)UARTbuf, "Checksum error");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_ERROR_CONNECT:
	    	  strcpy((char*)UARTbuf, "Connect error");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_MISSING_BYTES:
	    	  strcpy((char*)UARTbuf, "Missing bytes");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_ERROR_BYTES_ALL_ZERO:
	    	  strcpy((char*)UARTbuf, "All bytes read zero");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_ERROR_READ_TIMEOUT:
	    	  strcpy((char*)UARTbuf, "Read time out");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      case DHT20_ERROR_LASTREAD:
	    	  strcpy((char*)UARTbuf, "Error read too fast");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	      default:
	    	  strcpy((char*)UARTbuf, "Unknown error");
	    	  HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	        break;
	    }

		strcpy((char*)UARTbuf, "\r\n\0");
		HAL_UART_Transmit(&huart2, UARTbuf, strlen((char*)UARTbuf), HAL_MAX_DELAY);
	  }
	                    // wait for a second
	  HAL_Delay(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
