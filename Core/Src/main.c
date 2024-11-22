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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_RIGHT_Pin GPIO_PIN_14//red
#define LED_LEFT_Pin GPIO_PIN_12//green
#define LED_UP_Pin GPIO_PIN_13//orange
#define LED_DOWN_Pin GPIO_PIN_15 //blue

#define ACC_GYRO_ADDR  0x6B << 1  // Акселерометр/Гіроскоп (<< 1 для HAL)
#define MAG_ADDR       0x1E << 1  // Магнетометр (<< 1 для HAL)

// Регістри
#define OUT_X_L_G      0x18  // Гіроскоп X-axis Low byte
#define OUT_X_L_XL     0x28  // Акселерометр X-axis Low byte
#define OUT_X_L_M      0x28  // Магнетометр X-axis Low byte

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t currentClick = 0;
uint32_t prevClick = 0;
uint8_t mode = 0;
uint8_t MAX_NUMBER_MODE = 5;

int16_t acc[3], gyro[3], mag[3];
int16_t LIMIT = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void UpdateLEDs();
void ClearLEDs();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 200);
	return ch;
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
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LSM9DS1_Init();


      while (1) {
          // Зчитування даних
          LSM9DS1_ReadData(acc, gyro, mag);

          // Вивід у форматі UART



//          HAL_Delay(1000);


	  ClearLEDs();

	  switch(mode)
	  {
	  	  case 0:
	  		 printf("ACC: X:%d, Y:%d, Z:%d\r\n", acc[0], acc[1], acc[2]);
//	  		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  		 break;
	  	  case 1:
	  		  printf("GYRO: X:%d, Y:%d, Z:%d\r\n", gyro[0], gyro[1], gyro[2]);
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	  		  break;
	  	  case 2:
	  		  printf("MAG: X:%d, Y:%d, Z:%d\r\n", mag[0], mag[1], mag[2]);
//	  		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  		  break;
	  	  case 3:
          	  printf("ACC: X:%d, Y:%d, Z:%d\r\n", acc[0], acc[1], acc[2]);
          	  printf("GYRO: X:%d, Y:%d, Z:%d\r\n", gyro[0], gyro[1], gyro[2]);
			  printf("MAG: X:%d, Y:%d, Z:%d\r\n", mag[0], mag[1], mag[2]);

//			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			  break;
	  	  case 4:
	  		printf("LED\n\r");
	  		//printf("ACC: X:%d, Y:%d, Z:%d\r\n", acc[0], acc[1], acc[2]);
	  		UpdateLEDs();
	  	  default:
	  		  break;
	  }
	  HAL_Delay(200);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Функція для запису в регістр
void I2C_Write(uint16_t devAddr, uint8_t regAddr, uint8_t data) {
    uint8_t temp[2] = {regAddr, data};
    HAL_I2C_Master_Transmit(&hi2c1, devAddr, temp, 2, HAL_MAX_DELAY);
}

// Функція для читання з регістрів
void I2C_Read(uint16_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
    HAL_I2C_Master_Transmit(&hi2c1, devAddr, &regAddr, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, devAddr, data, size, HAL_MAX_DELAY);
}

// Функція для ініціалізації LSM9DS1
void LSM9DS1_Init() {
    // Ініціалізація гіроскопа
    I2C_Write(ACC_GYRO_ADDR, 0x10, 0xC0);  // CTRL_REG1_G: 238Hz, 2000dps

    // Ініціалізація акселерометра
    I2C_Write(ACC_GYRO_ADDR, 0x20, 0xC0);  // CTRL_REG6_XL: 238Hz, ±8g

    // Ініціалізація магнетометра
    I2C_Write(MAG_ADDR, 0x20, 0xFC);      // CTRL_REG1_M: Ultra-high performance
}

// Зчитування даних акселерометра, гіроскопа і магнетометра
void LSM9DS1_ReadData(int16_t *acc, int16_t *gyro, int16_t *mag) {
    uint8_t buffer[6];

    // Зчитування акселерометра
    I2C_Read(ACC_GYRO_ADDR, OUT_X_L_XL, buffer, 6);
    acc[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
    acc[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
    acc[2] = (int16_t)(buffer[4] | (buffer[5] << 8));

    // Зчитування гіроскопа
    I2C_Read(ACC_GYRO_ADDR, OUT_X_L_G, buffer, 6);
    gyro[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
    gyro[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
    gyro[2] = (int16_t)(buffer[4] | (buffer[5] << 8));

    // Зчитування магнетометра
    I2C_Read(MAG_ADDR, OUT_X_L_M, buffer, 6);
    mag[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
    mag[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
    mag[2] = (int16_t)(buffer[4] | (buffer[5] << 8));
}

// Вивід даних через UART



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	currentClick = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_0 && (currentClick - prevClick > 200))
	{
		/*
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		 */

		prevClick = currentClick;
		mode = (mode + 1) % MAX_NUMBER_MODE;
	}
}

void UpdateLEDs() {
    if (acc[0] > LIMIT && acc[1] > LIMIT) {
        HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_RESET);
    } else if (acc[0] < -LIMIT && acc[1] < -LIMIT) {
        HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_SET);
    }else if(acc[0] < -LIMIT && acc[1] > LIMIT) {
    	HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_RESET);
    }else if(acc[0] > LIMIT && acc[1] < -LIMIT) {
    	HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_RESET);
    }
}

void ClearLEDs() {
	HAL_GPIO_WritePin(GPIOD, LED_UP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_DOWN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_RIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, LED_LEFT_Pin, GPIO_PIN_RESET);
}
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
