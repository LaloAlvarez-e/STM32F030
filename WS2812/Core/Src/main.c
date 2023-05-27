/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define HIGH_CODE (0x7FC0) /*111111111000000*/
//#define LOW_CODE (0x7C00)  /*111110000000000*/
#define HIGH_CODE (0x01FF) /*111111111000000*/
#define LOW_CODE (0x001F)  /*111110000000000*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint16_t u16ValueLed[2] =
{
 LOW_CODE,
 HIGH_CODE,
};


uint32_t u32Delay = 1000;
uint32_t u32Colors[5] =
{
 0x0,
 0xFFD000,
 0xFFD000,
 0x00FF00,
 0x0000FF,
};

uint32_t u32ColorsMax[5] =
{
 0x0,
 0xFFD000,
 0xFFD000,
 0x00FF00,
 0x0000FF,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void WritePixel(uint8_t Red, uint8_t Green, uint8_t Blue);
void WritePixelColor(uint32_t Color);


void WritePixel(uint8_t Red, uint8_t Green, uint8_t Blue)
{
    uint8_t u8Temp;

    for(int i=0; i<8; i++)
    {
        u8Temp = Green & 0x80;
        Green <<= 1;
        u8Temp >>=7;
        SPI1->DR = u16ValueLed[u8Temp];
        while((SPI1->SR & SPI_SR_FTLVL) == SPI_SR_FTLVL);
    }

    for(int i=0; i<8; i++)
    {
        u8Temp = Red & 0x80;
        Red <<= 1;
        u8Temp >>=7;
        SPI1->DR = u16ValueLed[u8Temp];
        while((SPI1->SR & SPI_SR_FTLVL) == SPI_SR_FTLVL);
    }

    for(int i=0; i<8; i++)
    {
        u8Temp = Blue & 0x80;
        Blue <<= 1;
        u8Temp >>=7;
        SPI1->DR = u16ValueLed[u8Temp];
        while((SPI1->SR & SPI_SR_FTLVL) == SPI_SR_FTLVL);
    }
}


void WritePixelColor(uint32_t Color)
{
    uint8_t Green;
    uint8_t Red;
    uint8_t Blue;
    uint8_t u8Temp;

    Blue = Color & 0xFF;
    Color >>= 8;
    Green = Color & 0xFF;
    Color >>= 8;
    Red = Color & 0xFF;
    WritePixel(Red, Green, Blue);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t Red = 0;
	uint32_t Green = 0xFF;
	uint32_t Blue = 0;
	uint32_t RedDir = 0;
	uint32_t GreenDir = 0;
	uint32_t BlueDir = 0;
	uint32_t Color;
	uint32_t u32Count = 0;
	uint32_t u32Sub = 0;
	uint32_t u32Delay = 0;
	uint32_t u32Intensidad = 0;
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
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  SPI1->CR1 |= SPI_CR1_SPE;

  srand(SysTick->VAL);
  for(int i=0; i<60; i++)
  {
      WritePixelColor(0xFFFFFF);
  }
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  u32Count++;
	  if((u32Count & 0x0F) == 0)
	  {
		  for(int i=0; i<60; i++)
		  {
			  WritePixelColor(0xFFDF00);
		  }
	      HAL_Delay(2000);
	  }
	  else if ((u32Count & 0x07) == 0)
	  {
		  Color = rand();
		  srand(SysTick->VAL + Color);
		  for(int i=0; i<60; i++)
		  {

			  Color = rand() >> (12 + 16);
			  Color %= 5;
			  Color = u32ColorsMax[Color];
			  WritePixelColor(Color);
		  }
	      HAL_Delay(2000);

	  }
	  else
	  {
		  u32Sub = rand() >> (30);
		  u32Sub &= 1;
		  u32Delay = rand();
		  srand(SysTick->VAL + u32Delay);
		  u32Delay = rand() >> (12 + 16);
		  u32Delay %= 1;
		  u32Delay += 1;
		  u32Delay *= 1000;
		  if(u32Sub & 1)
		  {
			  Color = rand();
			  srand(SysTick->VAL + Color);
			  for(int i=0; i<60; i++)
			  {
				  Color = rand() >> (12+ 16);
				  Color %= 5;
				  Color = u32Colors[Color];
				  u32Intensidad = rand() >> (10+ 16);
				  u32Intensidad %= 8;

					Red = Color >> 16;
					Red &= 0xFF;
					Red >>= u32Intensidad;
					Green = Color >> 8;
					Green &= 0xFF;
					Green >>= u32Intensidad;
					Blue = Color >> 0;
					Blue &= 0xFF;
					Blue >>= u32Intensidad;
					Color = Red << 16;
					Color |= Green << 8;
					Color |= Blue << 0;
				  WritePixelColor(Color);
			  }
		  }
		  else
		  {
			  for(int i=0; i<60; i++)
			  {
				  WritePixelColor(0);
			  }
		  }
	      HAL_Delay(u32Delay);
	  }
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
  hspi1.Init.DataSize = SPI_DATASIZE_15BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 10;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
