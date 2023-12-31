/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx_ll_crc.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key1_Pin LL_GPIO_PIN_0
#define Key1_GPIO_Port GPIOA
#define Key1_EXTI_IRQn EXTI0_1_IRQn
#define Key2_Pin LL_GPIO_PIN_1
#define Key2_GPIO_Port GPIOA
#define Key2_EXTI_IRQn EXTI0_1_IRQn
#define Key3_Pin LL_GPIO_PIN_2
#define Key3_GPIO_Port GPIOA
#define Key3_EXTI_IRQn EXTI2_3_IRQn
#define Key4_Pin LL_GPIO_PIN_3
#define Key4_GPIO_Port GPIOA
#define Key4_EXTI_IRQn EXTI2_3_IRQn
#define Level_Pin LL_GPIO_PIN_4
#define Level_GPIO_Port GPIOA
#define Level_EXTI_IRQn EXTI4_15_IRQn
#define ModeDust_Pin LL_GPIO_PIN_5
#define ModeDust_GPIO_Port GPIOA
#define ModeDust_EXTI_IRQn EXTI4_15_IRQn
#define SpeedPulse_Pin LL_GPIO_PIN_6
#define SpeedPulse_GPIO_Port GPIOA
#define ModeRain_Pin LL_GPIO_PIN_7
#define ModeRain_GPIO_Port GPIOA
#define ModeRain_EXTI_IRQn EXTI4_15_IRQn
#define Pompe1_Pin LL_GPIO_PIN_1
#define Pompe1_GPIO_Port GPIOB
#define TM1637_CLK_Pin LL_GPIO_PIN_9
#define TM1637_CLK_GPIO_Port GPIOA
#define TM1637_DIO_Pin LL_GPIO_PIN_10
#define TM1637_DIO_GPIO_Port GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
