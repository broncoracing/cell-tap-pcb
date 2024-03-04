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
void can_irq(CAN_HandleTypeDef *pcan);
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

struct SteinhartHartParameters {
  float A,B,C;
};

struct ThermistorCal_T {
  uint8_t temp_calibrations[3]; // stores whether the calibration temperatures have been read
  uint8_t abc_calibration; // stores whether the thermistor calibration values have been calculated
  // readings to 
  float temps[3];
  uint16_t adc_readings[N_ADC_CHANNELS][3];
  struct SteinhartHartParameters parameters[N_ADC_CHANNELS];
};


#define PAGE_SIZE 0x400
#define CAL_TABLE 0x08010000 - PAGE_SIZE // last 1k block of flash* used to store calibration data
// * except that this chip actually secretly has 128k of flash instead of 64k (but don't tell anyone)


// Bootloader ID things - so that this board can check its own bootloader ID
struct app_vars_t
{
  // Application binary size
  uint32_t page_count;
  // CRC of application to verify before jumping to it
	uint32_t crc;
};

struct board_vars_t
{
  // Timestamp from bootloader build. Used to reset data when flashing new bootloader.
  uint64_t bl_build_version;
  // Board ID for bootloader. Must be different for each board
  uint8_t id;
  uint8_t _padding[3];
};


struct bl_vars_t // size has to be a multiple of 4
{
  struct app_vars_t app;
  struct board_vars_t board;
};

_Static_assert(sizeof(struct bl_vars_t) % 4 == 0);

struct __packed bl_cmd_t 
{
	uint8_t brd;
	uint8_t cmd;
	uint16_t par1;
	uint32_t par2;
};

_Static_assert (sizeof(struct bl_cmd_t) == 8);


// Base address to write app
#define APP_BASE ((uint32_t *)(0x08003000))

// Location where pvars are stored
#define FLASH_VARS ((volatile struct bl_vars_t *)(APP_BASE - 0x100))


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
