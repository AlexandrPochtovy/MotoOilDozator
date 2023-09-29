/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stddef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
__STATIC_INLINE size_t SetPWM (size_t actual, size_t SP, size_t limit) {
	if ((SP > actual) && (actual < limit)) {
		return actual += 1;
	}
	if ((SP < actual) && (actual > 1)) {
		return actual -= 1;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern volatile uint8_t keyMode;
extern TimerMode_t pulseMode;
extern const uint32_t wheelLen_mm;

extern uint32_t pulseTotalCount;	//actual pulse metter
extern uint32_t pulseLastCount;	//last pulse count for speed calculate
extern uint32_t pulseDelta;			//delta pulse in TIM16 period
extern uint16_t timeInjection;
extern uint32_t pompePWM;
extern uint8_t recalc;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	keyMode = (uint8_t)GPIOA->IDR & 0x0F;
  /* USER CODE END EXTI0_1_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0 */

    /* USER CODE END LL_EXTI_LINE_0 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1 */

    /* USER CODE END LL_EXTI_LINE_1 */
  }
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */
	keyMode = (uint8_t)GPIOA->IDR & 0x0F;
  /* USER CODE END EXTI2_3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */

    /* USER CODE END LL_EXTI_LINE_2 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */

    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    /* USER CODE BEGIN LL_EXTI_LINE_4 */
    if (LL_GPIO_IsInputPinSet(Level_GPIO_Port, Level_Pin)) {
    	LL_TIM_DisableCounter(TIM17);
    }
    if (!LL_GPIO_IsInputPinSet(Level_GPIO_Port, Level_Pin)) {
    	LL_TIM_EnableCounter(TIM17);
    }
    /* USER CODE END LL_EXTI_LINE_4 */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */
	TIM1->SR &= ~(TIM_SR_UIF | TIM_SR_COMIF | TIM_SR_TIF | TIM_SR_BIF);
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
	TIM1->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF |
								TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF);
  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	volatile uint32_t StatusReg = TIM3->SR;
	if ((StatusReg & TIM_SR_CC1IF) && (keyMode != Dust)) {//reset counters
		LL_TIM_DisableIT_CC1(TIM3);		//disable interrupt TIM3
		TIM1->CNT = 0;								//clear count
		TIM3->CNT = 0;								//clear count
		TIM14->CCR1 = 0;							//TIM14 clear PWM for smooth start
		TIM14->CCER |= TIM_CCER_CC1E;	//TIM14 enable PWM out
		TIM14->CR1 |= TIM_CR1_CEN;		//enable TIM14
		TIM16->CR1 &= ~TIM_CR1_CEN;		//disable TIM16
		TIM16->CR1 |= TIM_CR1_URS;		//update TIM16 disable
		TIM16->ARR = timeInjection;		//set injection time
		TIM16->CNT = 0;								//clear count
		TIM16->EGR |= TIM_EGR_UG;			//generate UEV for TIM16 for reload ARR value
		TIM16->CR1 &= ~(TIM_CR1_URS);	//update TIM16 enable
		TIM16->CR1 |= TIM_CR1_CEN;		//enable TIM16
		pulseMode = injection;				//switch TIM16 mode to injection's time
		}
	TIM3->SR = 0x00;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */
	volatile uint32_t StatusReg = TIM14->SR;
	if (StatusReg & TIM_SR_UIF) {
		if ((TIM14->CCR1 < pompePWM) && (pompePWM < TIM14->ARR)) {
			TIM14->CCR1 += 2;
		}
	}
	TIM14->SR = 0x00;
  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  /* USER CODE BEGIN TIM16_IRQn 1 */
	volatile uint32_t StatusReg = TIM16->SR;
	if (StatusReg & TIM_SR_UIF) {
		if (pulseMode == measure) {
			if (pulseTotalCount > pulseLastCount) {
				pulseDelta = pulseTotalCount - pulseLastCount;
				recalc = 1;
			}
			pulseLastCount = pulseTotalCount;
		}
		if (pulseMode == injection) {
			LL_TIM_DisableCounter(TIM14);
			LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
			TIM16->CR1 &= ~TIM_CR1_CEN;		//disable TIM16
			TIM16->CR1 |= TIM_CR1_URS;		//update TIM16 disable
			TIM16->ARR = 10000;						//set injection time
			TIM16->CNT = 0;								//clear count
			TIM16->EGR |= TIM_EGR_UG;			//generate UEV for TIM16 for reload ARR value
			TIM16->CR1 &= ~(TIM_CR1_URS);	//update TIM16 enable
			TIM16->CR1 |= TIM_CR1_CEN;		//enable TIM16
			pulseMode = measure;
		}
	}
	TIM16->SR = 0x00;
  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
