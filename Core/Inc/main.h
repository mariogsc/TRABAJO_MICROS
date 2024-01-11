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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POTENCIOMETRO_Pin GPIO_PIN_1
#define POTENCIOMETRO_GPIO_Port GPIOA
#define LDR_Pin GPIO_PIN_3
#define LDR_GPIO_Port GPIOA
#define PULSADOR_Pin GPIO_PIN_5
#define PULSADOR_GPIO_Port GPIOA
#define PULSADOR_EXTI_IRQn EXTI9_5_IRQn
#define INFRARROJO_Pin GPIO_PIN_6
#define INFRARROJO_GPIO_Port GPIOA
#define TRIGGER_Pin GPIO_PIN_4
#define TRIGGER_GPIO_Port GPIOC
#define ULTRASONIDO_Pin GPIO_PIN_1
#define ULTRASONIDO_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_9
#define SERVO_PWM_GPIO_Port GPIOE
#define LED_VERDE_Pin GPIO_PIN_13
#define LED_VERDE_GPIO_Port GPIOE
#define LED_ROJO_Pin GPIO_PIN_15
#define LED_ROJO_GPIO_Port GPIOE
#define LED_CASA_Pin GPIO_PIN_12
#define LED_CASA_GPIO_Port GPIOD
#define LED_3_ULTRA_Pin GPIO_PIN_6
#define LED_3_ULTRA_GPIO_Port GPIOD
#define LED_2_ULTRA_Pin GPIO_PIN_6
#define LED_2_ULTRA_GPIO_Port GPIOB
#define LED_1_ULTRA_Pin GPIO_PIN_7
#define LED_1_ULTRA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
