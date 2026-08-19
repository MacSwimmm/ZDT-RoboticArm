/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define LED_BOARD_Pin GPIO_PIN_1
#define LED_BOARD_GPIO_Port GPIOA

/* Robotic-arm input mapping from the PCB pin assignment. */
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOB
#define ENC1_E_Pin GPIO_PIN_0
#define ENC1_E_GPIO_Port GPIOD

#define ENC2_A_Pin GPIO_PIN_7
#define ENC2_A_GPIO_Port GPIOE
#define ENC2_B_Pin GPIO_PIN_8
#define ENC2_B_GPIO_Port GPIOE
#define ENC2_E_Pin GPIO_PIN_1
#define ENC2_E_GPIO_Port GPIOD

#define ENC3_A_Pin GPIO_PIN_2
#define ENC3_A_GPIO_Port GPIOE
#define ENC3_B_Pin GPIO_PIN_3
#define ENC3_B_GPIO_Port GPIOE
#define ENC3_E_Pin GPIO_PIN_2
#define ENC3_E_GPIO_Port GPIOD

#define ENC4_A_Pin GPIO_PIN_4
#define ENC4_A_GPIO_Port GPIOE
#define ENC4_B_Pin GPIO_PIN_5
#define ENC4_B_GPIO_Port GPIOE
#define ENC4_E_Pin GPIO_PIN_3
#define ENC4_E_GPIO_Port GPIOD

#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOC
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOC
#define KEY6_Pin GPIO_PIN_5
#define KEY6_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
