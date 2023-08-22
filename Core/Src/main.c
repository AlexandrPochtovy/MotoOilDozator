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
#include "crc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stddef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIMIT_COUNT	100//65535
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*******************************************************************************
 * pompe and chain config
 * wheel contains 4 pulses per revolution
 * wheel / chain = 7 / 3
 * chain contains 28 / 3 pulses per revolution
 * median pompe's nominal flowrate 1.246ml/sec
 *  */
const uint32_t pompeDoze_mkl 	= 1246;	//pompe's flowrate in mkl in nominal power without PWM
const uint32_t pinQuant 			= 130;	//chain pins quantity
const uint32_t wheelLen_sm 		= 210;	//wheel circumference length
const uint8_t wheelPulse			= 4;		//wheel pulses per cycle
const uint32_t pinV_mkl 			= 20;		//dose of oil per pin
uint8_t dozeCycle 						= 1;		//number of chain passes during lubrication

uint32_t injectionDistance_km;	//distance between oil injections
uint32_t injectionPulse;				//pulses between oil injections
uint32_t pulseTotalCount;				//actual pulse value
uint32_t pulseLastCount;				//last pulse value
uint32_t pulseDelta;						//delta pulse value during TIM16 period
uint16_t TIM3_upLimit;						//TIM3 pulse count limit
uint16_t TIM1_upLimit;						//TIM1 pulse count limit
uint16_t Td;
uint32_t pompePWM ;				//pompe pwm
uint16_t windowsInject;		//TIM3 CC1 reg for window's oil inject detect
uint8_t recalc = 0;
WorkMode_t mode = normal;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	pulseTotalCount = (TIM1->CNT * TIM3->ARR) + TIM3->CNT;
  	injectionPulse = 100000 * wheelPulse * injectionDistance_km / wheelLen_sm;
  	TIM1_upLimit = injectionPulse / (uint32_t)(LIMIT_COUNT + 1);
  	if (pulseTotalCount <= (TIM1_upLimit * (uint32_t)(LIMIT_COUNT + 1))) {
  		TIM3_upLimit = LIMIT_COUNT;
  	} else {
  		TIM3_upLimit = TIM1_upLimit - injectionPulse % (uint32_t)(LIMIT_COUNT + 1);
  	}
  	TIM1->ARR = TIM1_upLimit;
  	TIM3->ARR = TIM3_upLimit;
  	switch (mode) {
			case normal:
		  	if (recalc) {
		  		dozeCycle = 1;
		  		Td = TIM16->ARR;
		  		do {
		  			pompePWM = (6 * pinQuant * pinV_mkl * pulseDelta) / (28 * dozeCycle * Td * pompeDoze_mkl / TIM14->ARR) / 100;
		  			dozeCycle += 1;
		  		} while (pompePWM > TIM14->ARR);
		  		recalc = 0;
		  	}
				break;
			case rain:
				if (recalc) {
					dozeCycle = 1;
					Td = TIM16->ARR;
					do {
						pompePWM = 16 * (6 * pinQuant * pinV_mkl * pulseDelta) / (28 * dozeCycle * Td * pompeDoze_mkl / TIM14->ARR) / 1000;
						dozeCycle += 1;
						} while (pompePWM > TIM14->ARR);
					recalc = 0;
				}
				break;
			case dust:
				pompePWM = 2000;
				break;
			default:
				mode = normal;
				break;
		}
  	windowsInject = 28 * dozeCycle / 3;
  	LL_TIM_OC_SetCompareCH1(TIM14, pompePWM);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(32000000);
  LL_SetSystemCoreClock(32000000);
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
