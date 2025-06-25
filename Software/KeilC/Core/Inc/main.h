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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define PULSE_1_Pin GPIO_PIN_0
#define PULSE_1_GPIO_Port GPIOA
#define DIR_1_Pin GPIO_PIN_1
#define DIR_1_GPIO_Port GPIOA
#define PULSE_2_Pin GPIO_PIN_2
#define PULSE_2_GPIO_Port GPIOA
#define DIR_2_Pin GPIO_PIN_3
#define DIR_2_GPIO_Port GPIOA
#define PULSE_3_Pin GPIO_PIN_4
#define PULSE_3_GPIO_Port GPIOA
#define DIR_3_Pin GPIO_PIN_5
#define DIR_3_GPIO_Port GPIOA
#define Limit_SW_1_Pin GPIO_PIN_6
#define Limit_SW_1_GPIO_Port GPIOA
#define Limit_SW_2_Pin GPIO_PIN_7
#define Limit_SW_2_GPIO_Port GPIOA
#define Limit_SW_3_Pin GPIO_PIN_0
#define Limit_SW_3_GPIO_Port GPIOB
#define Limit_SW_5_Pin GPIO_PIN_1
#define Limit_SW_5_GPIO_Port GPIOB
#define EN_1_Pin GPIO_PIN_15
#define EN_1_GPIO_Port GPIOA
#define DIR_6_Pin GPIO_PIN_3
#define DIR_6_GPIO_Port GPIOB
#define PULSE_6_Pin GPIO_PIN_4
#define PULSE_6_GPIO_Port GPIOB
#define DIR_5_Pin GPIO_PIN_5
#define DIR_5_GPIO_Port GPIOB
#define PULSE_5_Pin GPIO_PIN_6
#define PULSE_5_GPIO_Port GPIOB
#define DIR_4_Pin GPIO_PIN_7
#define DIR_4_GPIO_Port GPIOB
#define PULSE_4_Pin GPIO_PIN_8
#define PULSE_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
