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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CYCLE_INJECT_MIN 2
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
const uint32_t wheelLen_sm = 210;			//wheel circumference length in centimeters
const uint8_t wheelPulse = 4;					//wheel pulses per cycle
uint32_t injectionDistance_km = 100;	//distance between oil injections
uint16_t pinV_mcl = 20;								//dose of oil per pin
uint16_t pompeDoze_mcl = 1246;				//pompe's flow-rate in microliters in nominal power without PWM (100% PWM)
uint16_t pinQuant = 130;							//chain pins quantity
uint16_t rainMullFactor = 60;					//rain multiplying factor in percent
const uint16_t addTimeInjection_ms = 100;	//add time injection for overload chain, milliseconds

uint32_t pulseTotalCount;				//actual pulse value
uint32_t pulseLastCount;				//last pulse value
uint32_t pulseDelta;						//delta pulse value during TIM16 period
uint8_t dozeCycle = 1;					//number of chain passes during lubrication
uint32_t pulsesBetweenInjection;//pulses between oil injections
uint16_t pompePWM;							//pompe pwm
uint16_t timeInjection;					//TIM16 CC1 reg for window's oil inject detect
uint8_t recalc = 0;							//flag for calculate values

KeyWorkMode_t keyMode = Normal;
TimerMode_t pulseMode = measure;
TM1637_t display = { .Clock_Pin = TM1637_CLK_Pin, .Clock_Port =
		TM1637_CLK_GPIO_Port, .Data_Pin = TM1637_DIO_Pin, .Data_Port =
		TM1637_DIO_GPIO_Port, .brightness = 0x03, .delay = 5, .delayNeed = 0 };

/************************************************************
 * 								MENU FOR TM1637														*
 ************************************************************/
const char dist[4] = {0x38, 0x79, 0xd4, 0x04}; 	//distance text "LEn.d"
const char dose[4] = {0x38, 0x79, 0xd4, 0x04}; 	//dose for one pin text "Pin.d"
const char pompe[4] = {0x73, 0x3f, 0xb7, 0x5e};  //pompe dose text "PON.d"
const char chain[4] = {0x39, 0x74, 0xf7, 0x5e};	//chain quantity pin text "Cha.d"
const char coeff[4] = {0x39, 0x3f, 0x79, 0x71};	//rain mull factor text "COEF"
const char timeAdd[4] = {0x77, 0x5e, 0xde, 0x73};//time add injection text "Add.P"

//create: menu name			next					prev					parent			child					sel				enter	text menu
MENU_ITEM(Dist_name, 		DozePin_name, TimeAdd_name, NULL_MENU, DistInj_val, 	Menu_Out, NULL, dist);
MENU_ITEM(DozePin_name, Pompe_name, 	Dist_name, 		NULL_MENU, DozePin_val, 	Menu_Out, NULL, dose);
MENU_ITEM(Pompe_name, 	Chain_name, 	DozePin_name, NULL_MENU, Pompe_val, 		Menu_Out, NULL, pompe);
MENU_ITEM(Chain_name, 	RainC_name, 	Pompe_name, 	NULL_MENU, Chain_val, 		Menu_Out, NULL, chain);
MENU_ITEM(RainC_name, 	TimeAdd_name, Chain_name, 	NULL_MENU, RainC_val,			Menu_Out, NULL, coeff);
MENU_ITEM(TimeAdd_name, Dist_name, 		RainC_name, 	NULL_MENU, TimeAdd_val, 	Menu_Out, NULL, timeAdd);

MENU_ITEM(DistInj_val, 	NULL_MENU, 		NULL_MENU, 		Dist_name, 		NULL_MENU, 	NULL, 		NULL, NULL);
MENU_ITEM(DozePin_val, 	NULL_MENU, 		NULL_MENU, 		DozePin_name, NULL_MENU, 	NULL, 		NULL, NULL);
MENU_ITEM(Pompe_val, 		NULL_MENU, 		NULL_MENU, 		Pompe_name, 	NULL_MENU, 	NULL, 		NULL, NULL);
MENU_ITEM(Chain_val, 		NULL_MENU, 		NULL_MENU, 		Chain_name, 	NULL_MENU, 	NULL, 		NULL, NULL);
MENU_ITEM(RainC_val, 		NULL_MENU, 		NULL_MENU, 		RainC_name, 	NULL_MENU, 	NULL, 		NULL, NULL);
MENU_ITEM(TimeAdd_val, 	NULL_MENU, 		NULL_MENU, 		TimeAdd_name, NULL_MENU, 	NULL, 		NULL, NULL);


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
int main(void) {
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
	HardwareInit();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CRC_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */
	pulsesBetweenInjection = 100000 * wheelPulse * injectionDistance_km / wheelLen_sm;
	Menu_Navigate(&Dist_name);
	uint32_t pulsesdistance = pulsesBetweenInjection;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (recalc) {
			dozeCycle = CYCLE_INJECT_MIN;
			do {
				dozeCycle += 1;
				pompePWM = 	(6 * pinQuant * pinV_mcl * pulseDelta) /
							(7 * wheelPulse * dozeCycle  * (TIM16->ARR / 1000) * pompeDoze_mcl / TIM14->ARR);
			} while (pompePWM > TIM14->ARR);
			timeInjection = 7 * wheelPulse * dozeCycle  * (TIM16->ARR / 1000) / (3 * pulseDelta);
			TIM16->CCR1 = timeInjection;
			recalc = 0;
		}
		switch (keyMode) {
		case Normal:
			pulsesdistance = pulsesBetweenInjection;
			break;
		case Rain:
			pulsesdistance = 100 * pulsesBetweenInjection / (100 + rainMullFactor);
			break;
		case Dust:
			pompePWM = 2000;
			break;
		case Key1:
			Menu_Navigate(MENU_PREVIOUS);
			break;
		case Key2:
			Menu_Navigate(MENU_NEXT);
			break;
		case Key3:
			Menu_Navigate(MENU_PARENT);
			break;
		case Key4:
			Menu_Navigate(MENU_CHILD);
			break;
		default:
			keyMode = Normal;
			break;
		}
		pulseTotalCount = (TIM1->CNT * TIM3->ARR) + TIM3->CNT;
		TIM3->CCR1 = pulsesdistance % (uint32_t) (TIM3->ARR + 1);
		if (pulseTotalCount > (TIM1->CNT * TIM3->ARR)) {
			LL_TIM_EnableIT_CC1(TIM3);
		}

		/*	TM1637 section	*/

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
	}
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1) {

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_Init1msTick(32000000);
	LL_SetSystemCoreClock(32000000);
}

/* USER CODE BEGIN 4 */
void Menu_Out(void){
	TM1637_SendArray(&display, (uint8_t *)Menu_GetCurrentMenu()->Text, 1);
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
