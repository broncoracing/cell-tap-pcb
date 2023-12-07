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
#define STAT6_Pin GPIO_PIN_10
#define STAT6_GPIO_Port GPIOB
#define STAT7_Pin GPIO_PIN_11
#define STAT7_GPIO_Port GPIOB
#define STAT8_Pin GPIO_PIN_12
#define STAT8_GPIO_Port GPIOB
#define STAT9_Pin GPIO_PIN_13
#define STAT9_GPIO_Port GPIOB
#define STAT10_Pin GPIO_PIN_14
#define STAT10_GPIO_Port GPIOB
#define STAT11_Pin GPIO_PIN_15
#define STAT11_GPIO_Port GPIOB
#define SYS_STAT_Pin GPIO_PIN_15
#define SYS_STAT_GPIO_Port GPIOA
#define STAT0_Pin GPIO_PIN_4
#define STAT0_GPIO_Port GPIOB
#define STAT1_Pin GPIO_PIN_5
#define STAT1_GPIO_Port GPIOB
#define STAT2_Pin GPIO_PIN_6
#define STAT2_GPIO_Port GPIOB
#define STAT3_Pin GPIO_PIN_7
#define STAT3_GPIO_Port GPIOB
#define STAT4_Pin GPIO_PIN_8
#define STAT4_GPIO_Port GPIOB
#define STAT5_Pin GPIO_PIN_9
#define STAT5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Error limits for ADC readings
#define ADC_ERR_HIGH 4000
#define ADC_ERR_LOW 100

#define N_ADC_CHANNELS 15
#define N_STATUS_LEDS 12
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
