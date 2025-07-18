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

/* USER CODE BEGIN PV */
// Глобальные переменные
volatile uint16_t pulse_count = 0;
volatile uint8_t digits[4] = {0};
const uint8_t period_number = 3;
volatile uint32_t current_time = 0;
volatile uint32_t previous_time = 0;
volatile double filtered_rpm = 0.0;      // Отфильтрованное значение RPM
const float alpha = 0.2;                 // Коэффициент фильтрации (0.1-0.3)
const uint16_t timeout_ms = 2000;
volatile uint32_t last_pulse_time = 0;

//исправленные паттерны цифр для этой платы (общий катод)
const uint8_t digit_pattern[10] = {
    0x7B, // 0 - 01111011
    0x0A, // 1 - 00001010
    0xB3, // 2 - 10110011
    0x9B, // 3 - 10011011
    0xCA, // 4 - 11001010
    0xD9, // 5 - 11011001
    0xF9, // 6 - 11111001
    0x0B, // 7 - 00001011
    0xFB, // 8 - 11111011
    0xDB  // 9 - 11011011
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Update_Display(void);
void EXTI4_15_IRQHandler(void);
void Frequency_calculation(void);
void Test_Segments(void);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  Test_Segments();

  // Инициализация дисплея (все нули)
  digits[0] = 0;
  digits[1] = 0;
  digits[2] = 0;
  digits[3] = 0;
  Update_Display();
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (pulse_count >= period_number) {
		  Frequency_calculation();
		  Update_Display();
	  }
	  else if (last_pulse_time > 0 && (HAL_GetTick() - last_pulse_time) > timeout_ms) {
	          filtered_rpm = 0.0;
	          digits[0] = 0;
	          digits[1] = 0;
	          digits[2] = 0;
	          digits[3] = 0;
	          Update_Display();
	          last_pulse_time = 0;  // Сброс флага таймаута
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OE_Pin|Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OE_Pin Latch_Pin */
  GPIO_InitStruct.Pin = OE_Pin|Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Обновление индикаторов
void Update_Display(void) {
    uint8_t data[4];
    // Порядок передачи: LED1, LED2, LED3, LED4 (единицы → тысячи)
    data[0] = digit_pattern[digits[3]]; // Единицы (LED4)
    data[1] = digit_pattern[digits[2]]; // Десятки (LED3)
    data[2] = digit_pattern[digits[1]]; // Сотни (LED2)
    data[3] = digit_pattern[digits[0]]; // Тысячи (LED1)

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // LATCH = 0
    HAL_SPI_Transmit(&hspi1, data, 4, 100);               // Передача данных
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // LATCH = 1
}

// Обработчик прерывания для PB7
void EXTI4_15_IRQHandler(void) {
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) {
        pulse_count++;
        last_pulse_time = HAL_GetTick();
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    }
}

// Тест сегментов
void Test_Segments(void) {
    uint8_t test_data[4] = {0};

    // Все сегменты выключены
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, test_data, 4, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1000);

    // Поочередное включение сегментов
    for(uint8_t seg = 0; seg < 8; seg++) {
        for(uint8_t i = 0; i < 4; i++) {
            test_data[i] = (1 << seg);
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, test_data, 4, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        HAL_Delay(500);
    }
}

//Функция расчета частоты
void Frequency_calculation(void) {
    previous_time = current_time;
    current_time = HAL_GetTick();

    // Расчет мгновенного значения RPM
    double frequency_rpm = 60000.0 / ((current_time - previous_time) / pulse_count);
    pulse_count = 0;

    // Применение EMA-фильтра
    if (filtered_rpm == 0.0) {
        filtered_rpm = frequency_rpm;  // Инициализация при первом измерении
    } else {
        filtered_rpm = alpha * frequency_rpm + (1 - alpha) * filtered_rpm;
    }

    // Ограничение значения (0-9999)
    uint16_t display_rpm = (filtered_rpm > 9999) ? 9999 : (uint16_t)filtered_rpm;

    // Преобразование в цифры
    digits[0] = display_rpm / 1000;          // Тысячи
    digits[1] = (display_rpm % 1000) / 100;  // Сотни
    digits[2] = (display_rpm % 100) / 10;    // Десятки
    digits[3] = display_rpm % 10;            // Единицы
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
