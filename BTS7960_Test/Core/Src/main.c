/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

typedef enum
{
  FORWARD,
  REWIND,
  STOP,
  TURNOFF,
  M_ENABLE
}MOTORMODE;

ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG	 1		//Change here, 1 for Test code/ 0 for Production code
#define HIGH	 1
#define LOW	 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t switchPressed = LOW;
volatile char txBuffer[30];
volatile MOTORMODE currentDir = TURNOFF, previousDir = TURNOFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ControlMotor(MOTORMODE);
;
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
  volatile uint32_t fbTotalFwd, fbTotalRwd;
  volatile uint16_t fbAvgFwd, fbAvgRwd;
  volatile uint32_t fbSamples;
  volatile uint16_t fbInstFwd, fbInstRwd;
  volatile uint8_t adcCutoff;
  const uint8_t adcInterval = 70;
  const uint8_t adcOffset = 10; 	// Additional Offset While Motor rewinds
  volatile unsigned long currentMillis, previousMillisADCTime, previousMillisUSARTTime;

#if DEBUG
  volatile uint8_t adcThresh = 200;
#else
  volatile uint8_t adcThresh = 30;
#endif

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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  ControlMotor(M_ENABLE);
  sprintf(txBuffer, "DoorStep Controller: V4.1\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 10);
  HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      currentMillis = HAL_GetTick();

      ////////////// Feedback Sampling Routine //////////////
      sConfig.Channel = ADC_CHANNEL_5;
      HAL_ADC_ConfigChannel(&hadc, &sConfig);
      HAL_ADC_Start(&hadc);
      if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
	{
	  fbInstFwd = HAL_ADC_GetValue(&hadc);
	}

      sConfig.Channel = ADC_CHANNEL_2;
      HAL_ADC_ConfigChannel(&hadc, &sConfig);
      HAL_ADC_Start(&hadc);
      if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
	{
	  fbInstRwd = HAL_ADC_GetValue(&hadc);
	}
      HAL_ADC_Stop(&hadc);
      fbTotalFwd += fbInstFwd;
      fbTotalRwd += fbInstRwd;
      fbSamples++;
      if(currentMillis - previousMillisADCTime > adcInterval)
	{
	  previousMillisADCTime = currentMillis;
	  fbAvgFwd = (fbTotalFwd/fbSamples);
	  fbAvgRwd = (fbTotalRwd/fbSamples);
	  fbTotalFwd = 0;
	  fbTotalRwd = 0;
	  fbSamples = 0;
	}

      ////////////// Feedback Threshold Checking Routine //////////////
      if ((fbAvgFwd > adcThresh) || (fbAvgRwd > (adcThresh + adcOffset)))
	{
	  ControlMotor(STOP);
	  previousDir = currentDir;
	  currentDir = STOP;
	  HAL_GPIO_WritePin(GPIOA, USR_LED_Pin, GPIO_PIN_RESET);
	}

      if(currentMillis - previousMillisUSARTTime > 500)
	{
	  previousMillisUSARTTime = currentMillis;
	  sprintf(txBuffer, "FWD: %d RWD: %d\r\n", fbAvgFwd, fbAvgRwd);
	  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 10);
	}

      ////////////// Switch Press Routine //////////////
      if(switchPressed == HIGH)
	{
	  HAL_Delay(200);		//Switch Debouncing Delay
	  switchPressed = LOW;
	  if(HAL_GPIO_ReadPin(SWITCH_INPUT_GPIO_Port, SWITCH_INPUT_Pin) == HIGH)
	    {
	      if (currentDir != FORWARD)
		{
		  currentDir = FORWARD;
		  ControlMotor(FORWARD);
		}
	    }
	  else
	    {
	      if (currentDir != REWIND)
		{
		  currentDir = REWIND;
		  ControlMotor(REWIND);
		}
	    }
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//////////// INTR handling in stm32f0xx_it.c ////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //	if(GPIO_Pin == SWITCH_INPUT_Pin)
  //	{
  switchPressed = HIGH;
  //	}
}

void ControlMotor(MOTORMODE direction)
{
  if (direction == FORWARD)		// Turn Motor Forward
    {
      HAL_GPIO_WritePin(GPIOA, MOTOR_PIN1_Pin|MOTOR_PIN2_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOA, MOTOR_PIN1_Pin|USR_LED_Pin, GPIO_PIN_SET);
    }
  else if (direction == REWIND)		// Turn Motor Backward
    {
      HAL_GPIO_WritePin(GPIOA, MOTOR_PIN1_Pin|MOTOR_PIN2_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(GPIOA, MOTOR_PIN2_Pin|USR_LED_Pin, GPIO_PIN_SET);
    }
  else if (direction == STOP)		// Turn Off Motor
    {
      HAL_GPIO_WritePin(GPIOA, MOTOR_PIN1_Pin|MOTOR_PIN2_Pin, GPIO_PIN_RESET);
    }
  else if (direction == TURNOFF)	//Shut Down Motor
    {
      HAL_GPIO_WritePin(GPIOA, MOTOR_EN1_Pin|MOTOR_EN2_Pin|MOTOR_PIN1_Pin|MOTOR_PIN2_Pin, GPIO_PIN_RESET);
    }
  else if (direction == M_ENABLE)		//Enable Motor
    {
      HAL_GPIO_WritePin(GPIOA, MOTOR_EN1_Pin|MOTOR_EN2_Pin, GPIO_PIN_SET);
    }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
