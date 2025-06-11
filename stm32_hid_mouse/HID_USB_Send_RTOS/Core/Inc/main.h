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
#include "stm32f1xx_hal.h"

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
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOA
#define Phase_A_Pin GPIO_PIN_6
#define Phase_A_GPIO_Port GPIOA
#define Phase_B_Pin GPIO_PIN_7
#define Phase_B_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_10
#define CSN_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_11
#define CE_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_12
#define IRQ_GPIO_Port GPIOB
#define K_Middle_Pin GPIO_PIN_5
#define K_Middle_GPIO_Port GPIOB
#define K_Middle_EXTI_IRQn EXTI9_5_IRQn
#define K_Left_Pin GPIO_PIN_6
#define K_Left_GPIO_Port GPIOB
#define K_Left_EXTI_IRQn EXTI9_5_IRQn
#define K_Right_Pin GPIO_PIN_7
#define K_Right_GPIO_Port GPIOB
#define K_Right_EXTI_IRQn EXTI9_5_IRQn
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
