/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
#define LIMIT6_Pin GPIO_PIN_0
#define LIMIT6_GPIO_Port GPIOC
#define LIMIT6_EXTI_IRQn EXTI0_IRQn
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOC
#define LIMIT1_Pin GPIO_PIN_3
#define LIMIT1_GPIO_Port GPIOC
#define LIMIT1_EXTI_IRQn EXTI3_IRQn
#define STEP3_Pin GPIO_PIN_0
#define STEP3_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define LIMIT2_Pin GPIO_PIN_10
#define LIMIT2_GPIO_Port GPIOB
#define LIMIT2_EXTI_IRQn EXTI15_10_IRQn
#define STEP7_Pin GPIO_PIN_14
#define STEP7_GPIO_Port GPIOB
#define STEP4_Pin GPIO_PIN_6
#define STEP4_GPIO_Port GPIOC
#define STEP1_Pin GPIO_PIN_7
#define STEP1_GPIO_Port GPIOC
#define LIMIT5_Pin GPIO_PIN_8
#define LIMIT5_GPIO_Port GPIOC
#define LIMIT5_EXTI_IRQn EXTI9_5_IRQn
#define DIR6_Pin GPIO_PIN_10
#define DIR6_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_11
#define EN_GPIO_Port GPIOA
#define DIR7_Pin GPIO_PIN_12
#define DIR7_GPIO_Port GPIOA
#define DIR5_Pin GPIO_PIN_10
#define DIR5_GPIO_Port GPIOC
#define DIR4_Pin GPIO_PIN_11
#define DIR4_GPIO_Port GPIOC
#define DIR3_Pin GPIO_PIN_12
#define DIR3_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOD
#define LIMIT3_Pin GPIO_PIN_5
#define LIMIT3_GPIO_Port GPIOB
#define LIMIT3_EXTI_IRQn EXTI9_5_IRQn
#define STEP2_Pin GPIO_PIN_6
#define STEP2_GPIO_Port GPIOB
#define LIMIT4_Pin GPIO_PIN_7
#define LIMIT4_GPIO_Port GPIOB
#define LIMIT4_EXTI_IRQn EXTI9_5_IRQn
#define STEP5_Pin GPIO_PIN_8
#define STEP5_GPIO_Port GPIOB
#define STEP6_Pin GPIO_PIN_9
#define STEP6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
