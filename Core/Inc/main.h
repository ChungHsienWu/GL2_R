/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define led_Pin GPIO_PIN_9
#define led_GPIO_Port GPIOB
#define RESXP_Pin GPIO_PIN_6
#define RESXP_GPIO_Port GPIOB
#define sync_Pin GPIO_PIN_8
#define sync_GPIO_Port GPIOA
#define sync_EXTI_IRQn EXTI9_5_IRQn
#define pic_sw_Pin GPIO_PIN_12
#define pic_sw_GPIO_Port GPIOD
#define pic_sw_EXTI_IRQn EXTI15_10_IRQn
#define flash_cs_Pin GPIO_PIN_12
#define flash_cs_GPIO_Port GPIOB
#define en_Pin GPIO_PIN_12
#define en_GPIO_Port GPIOF
#define adc_en_Pin GPIO_PIN_3
#define adc_en_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
