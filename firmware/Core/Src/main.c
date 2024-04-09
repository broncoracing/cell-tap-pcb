/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can-ids/CAN.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TODO Read from flash
uint32_t BOARD_ID = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_TypeDef* status_ports[N_STATUS_LEDS] = {
  STAT0_GPIO_Port,
  STAT1_GPIO_Port,
  STAT2_GPIO_Port,
  STAT3_GPIO_Port,
  STAT4_GPIO_Port,
  STAT5_GPIO_Port,
  STAT6_GPIO_Port,
  STAT7_GPIO_Port,
  STAT8_GPIO_Port,
  STAT9_GPIO_Port,
  STAT10_GPIO_Port,
  STAT11_GPIO_Port,
};
uint16_t status_pins[N_STATUS_LEDS] = {
  STAT0_Pin,
  STAT1_Pin,
  STAT2_Pin,
  STAT3_Pin,
  STAT4_Pin,
  STAT5_Pin,
  STAT6_Pin,
  STAT7_Pin,
  STAT8_Pin,
  STAT9_Pin,
  STAT10_Pin,
  STAT11_Pin,
};

uint8_t adc_to_channel_mapping[N_ADC_CHANNELS] = {
  0, 1, 2, 12, 10, 6, 8, 14, 3, 4, 9, 11, 13, 7, 5
};



volatile uint16_t adc_readings[N_ADC_CHANNELS];

int16_t temperatures[N_ADC_CHANNELS];

// Orion BMS CAN data
uint8_t orion_data_ready = 0; // stores whether data is available to send yet
const uint8_t thermistor_module_number = 0;
int8_t pack_min_temperature; // in degrees C
uint8_t pack_min_idx;
int8_t pack_max_temperature; // in degrees C
uint8_t pack_max_idx;
int32_t pack_temp_sum; // for calculating average temperature
uint8_t pack_temp_samples;
const uint8_t thermistor_count = 12 * 7;

const uint8_t orion_bms_address_claim[8] = {
  0xF3, 0x00, 0x80, 0xF3, 0x00, 0x40, 0x1E, 0x90
};


uint32_t write_cal_data(struct ThermistorCal_T* data);


float compute_resistance(uint16_t adc_reading) {
  float adc_float = (float) adc_reading;
  float voltage_fac = adc_float / ((float) 0x1000); // input voltage as a fraction of VDDA
  const float ref_resistance = 10000.0; // precision reference resistor value
  float rtd_resistance = ref_resistance * ((1.0 / voltage_fac) - 1);
  return rtd_resistance;
}

float c2k(float c) {
    return c + 273.15;
}
float k2c(float k) {
    return k - 273.15;
}

struct SteinhartHartParameters compute_parameters(float r1, float t1, float r2, float t2, float r3, float t3) {
    float ln_r1 = logf(r1);
    float ln_r2 = logf(r2);
    float ln_r3 = logf(r3);

    float Y_1 = 1.0 / c2k(t1);
    float Y_2 = 1.0 / c2k(t2);
    float Y_3 = 1.0 / c2k(t3);

    float gamma_2 = (Y_2 - Y_1) / (ln_r2 - ln_r1);
    float gamma_3 = (Y_3 - Y_1) / (ln_r3 - ln_r1);

    float C = ((gamma_3 - gamma_2) / (ln_r3 - ln_r2)) * 1.0 / (ln_r1 + ln_r2 + ln_r3);

    float B = gamma_2 - C * (ln_r1 * ln_r1 + ln_r1 * ln_r2 + ln_r2 * ln_r2);

    float A = Y_1 - (B + C * ln_r1 * ln_r1) * ln_r1;

    struct SteinhartHartParameters params = {A,B,C};
    return params;
}

float compute_temperature(struct SteinhartHartParameters* p, float r) {
  float ln_r = logf(r);
  float t_inv = p->A + p->B * ln_r + p->C * ln_r * ln_r * ln_r;
  float temp_kelvin = 1.0 / t_inv;
  return k2c(temp_kelvin);
}

void can_send_float(uint8_t errno, float f1) {
  // Message header
  CAN_TxHeaderTypeDef m;
  // Use standard ID
  m.IDE = CAN_ID_STD;
  // Set message ID
  m.StdId = CELL_TAP_CAL_RESP_ID + BOARD_ID;
  // Use CAN_RTR_DATA to transmit data
  // We may use CAN_RTR_REMOTE to request things to send data in the future
  m.RTR = CAN_RTR_DATA;
  // Amount of data
  m.DLC = 8;

  // Data to send
  uint8_t data_u8[8];

  uint32_t* int_value = (uint32_t*) &f1;
  data_u8[4] = (*int_value >> 24) & 0xff;
  data_u8[5] = (*int_value >> 16) & 0xff;
  data_u8[6] = (*int_value >> 8) & 0xff;
  data_u8[7] = (*int_value) & 0xff;
  data_u8[0] = errno;
  data_u8[1] = 0;
  data_u8[2] = 0;
  data_u8[3] = 0;

  // Send a message
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &m, data_u8, &mailbox);
  
  // Wait for message to be sent. This is usually not needed.
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
}

uint8_t cal_cmd(uint8_t cmd, float value) {
  uint8_t errno = 0;
  
  // read current calibration from flash
  struct ThermistorCal_T current_cal;
  struct ThermistorCal_T* memcpy_source = CAL_TABLE;
  memcpy(&current_cal, memcpy_source, sizeof(struct ThermistorCal_T));
  switch(cmd) {
    case 0:
      // clear calibration
      struct ThermistorCal_T blank_cal = {0};
      if(write_cal_data(&blank_cal) > 0) errno = 1;
      break;
    case 1:
    case 2:
    case 3:
      current_cal.temps[cmd - 1] = value;
      for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
        // TODO use filtered adc readings
        uint16_t reading = adc_readings[i];
        if(reading < ADC_ERR_LOW || reading > ADC_ERR_HIGH) {
          errno = 3;
          break;
        }
        current_cal.adc_readings[i][cmd - 1] = adc_readings[i];
      }
      if(errno == 0) {
        current_cal.temp_calibrations[cmd - 1] = 1;
        if(write_cal_data(&current_cal) > 0) errno = 1;
      }
      break;
    case 4:
      // Verify that all three calibration points have been read, otherwise the result will be nonsense
      if(current_cal.temp_calibrations[0] && current_cal.temp_calibrations[1] && current_cal.temp_calibrations[2]) {
        float t1 = current_cal.temps[0];
        float t2 = current_cal.temps[1];
        float t3 = current_cal.temps[2];
        // now low/mid/high_cal_idx should be in order
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          float r1 = compute_resistance(current_cal.adc_readings[i][0]);
          float r2 = compute_resistance(current_cal.adc_readings[i][1]);
          float r3 = compute_resistance(current_cal.adc_readings[i][2]);

          // previous debugging code
          // can_send_float(200, r1);
          // can_send_float(201, t1);
          // can_send_float(202, r2);
          // can_send_float(203, t2);
          // can_send_float(204, r3);
          // can_send_float(205, t3);

          struct SteinhartHartParameters params = compute_parameters(r1, t1, r2, t2, r3, t3);
          if(isnan(params.A) || isnan(params.B) || isnan(params.C)) {
            errno = 5;
            break;
          }

          current_cal.parameters[i] = params;
        }
      } else {
        errno = 4;
        break;
      }

      current_cal.abc_calibration = 1;

      if(write_cal_data(&current_cal) > 0) errno = 1;
      break;
    case 5: // dump out calibration values
        // ABC Values
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(20 + i, current_cal.parameters[i].A);
        }
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(40 + i, current_cal.parameters[i].B);
        }
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(60 + i, current_cal.parameters[i].C);
        }

        // Calib. temps
        can_send_float(80, current_cal.temps[0]);
        can_send_float(81, current_cal.temps[1]);
        can_send_float(82, current_cal.temps[2]);

        // ADC Readings
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(100 + i, (float) current_cal.adc_readings[i][0]);
        }
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(120 + i, (float) current_cal.adc_readings[i][1]);
        }
        for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
          can_send_float(140 + i, (float) current_cal.adc_readings[i][2]);
        }
      break;

    default:
      errno = 2;
  }

  // send response
  // Message header
  CAN_TxHeaderTypeDef m;
  // Use standard ID
  m.IDE = CAN_ID_STD;
  // Set message ID
  m.StdId = CELL_TAP_CAL_RESP_ID + BOARD_ID;
  // Use CAN_RTR_DATA to transmit data
  // We may use CAN_RTR_REMOTE to request things to send data in the future
  m.RTR = CAN_RTR_DATA;
  // Amount of data
  m.DLC = CELL_TAP_CAL_RESP_DLC;

  // Data to send
  uint8_t data[1];
  data[0] = errno;

  // Send a message
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &m, data, &mailbox);
  // Wait for message to be sent. This is usually not needed.
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
  return 0;
}

void update_orion_bms_data(int8_t t, uint8_t idx) {
  pack_temp_sum += t;
  pack_temp_samples ++;
  if(t < pack_min_temperature) {
    pack_min_temperature = t;
    pack_min_idx = idx;
  }
  if(t > pack_max_temperature) {
    pack_max_temperature = t;
    pack_max_idx = idx;
  }
}

void reset_orion_bms_data(void) {
  pack_max_temperature = INT8_MIN;
  pack_min_temperature = INT8_MAX;
  pack_temp_sum = 0;
  pack_temp_samples = 0;

  for(uint8_t i = 0; i < 12; ++i) {
    int8_t temp_i8 = (int8_t)(temperatures[i] / 100);
    update_orion_bms_data(temp_i8, i);
  }
}

// CAN Interrupt handler. Called whenever a new CAN frame is received.
void can_irq(CAN_HandleTypeDef *pcan) {
  // Message header
  CAN_RxHeaderTypeDef msg;
  // Message data. Only filled with as much data as the message contained, the rest is garbage.
  uint8_t data[8];

  // Read the next CAN frame
  HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &msg, data);

  // Only accept standard CAN IDs, we don't use extended IDs.
  if(msg.IDE == CAN_ID_STD) { 
    switch (msg.StdId)
      {
      // Add cases for each CAN ID you want to handle

      // ID 0xB0 is a bootloader command, so reset the mcu when received.
      case BOOTLOADER_ID:
        __NVIC_SystemReset(); // Reset to bootloader
        break;
      
      case CELL_TAP_CAL_CMD_ID: // calibration command
        uint8_t command = read_field_u8(&CELL_TAP_CAL_CMD_cmd, data);
        uint32_t int_value = (data[1] << 24) + (data[2] << 16) + (data[3] << 8) + data[4];
        float value = *(float *) &int_value; // reinterpret bytes as float without doing conversion

        cal_cmd(command, value);

      default:
        // Board 0 collects data and sends it to the orion BMS
        if(BOARD_ID == 0){
          // check if the message is from a segment board
          if(msg.StdId >= CELL_TAP_1_1_ID && msg.StdId < CELL_TAP_7_1_ID + 4) {
            if(msg.StdId % 4 != 3) { // don't read the board thermistors, only cells
              uint8_t board_no = (msg.StdId - CELL_TAP_1_1_ID) / 4;
              uint8_t message_no = msg.StdId % 4;
              uint8_t start_thermistor_idx = board_no * 12 + message_no * 4;
              // record max/min temperature
              int16_t* temps = (int16_t*) data;
              for(uint8_t i = 0; i < 4; ++i) {
                int8_t temp_i8 = (int8_t) (temps[i] / 100);
                update_orion_bms_data(temp_i8, start_thermistor_idx + i);
              }
            }
          }

        }
        break;
      }
  }
}

void can_transmit_orion_bms(void) {
  // Message header
  CAN_TxHeaderTypeDef m;
  // Use standard ID
  m.IDE = CAN_ID_EXT;
  // Set message ID
  m.ExtId = 0x1839F380;
  // Use CAN_RTR_DATA to transmit data
  // We may use CAN_RTR_REMOTE to request things to send data in the future
  m.RTR = CAN_RTR_DATA;
  // Amount of data
  m.DLC = 8;

  // Data to send
  uint8_t data[8];

  int32_t average_temp = pack_temp_sum / pack_temp_samples;
  data[0] = thermistor_module_number;
  data[1] = pack_min_temperature;
  data[2] = pack_max_temperature;
  data[3] = (int8_t) average_temp;
  data[4] = thermistor_count;
  data[5] = pack_max_idx;
  data[6] = pack_min_idx;

  // Checksum 8-bit
  // (sum of all bytes + 0x39 + length)
  uint8_t checksum = 0;
  for(int i = 0; i < 7; ++i) {
    checksum += data[i];
  }
  checksum += 0x39;
  checksum += 8;

  data[7] = checksum;

  // Send a message
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &m, data, &mailbox);
  // Wait for message to be sent. This is usually not needed.
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
}

void can_transmit_orion_bms_address_claim(void) {
  // Message header
  CAN_TxHeaderTypeDef m;
  // Use standard ID
  m.IDE = CAN_ID_EXT;
  // Set message ID
  m.ExtId = 0x18EEFF80;
  // Use CAN_RTR_DATA to transmit data
  // We may use CAN_RTR_REMOTE to request things to send data in the future
  m.RTR = CAN_RTR_DATA;
  // Amount of data
  m.DLC = 8;

  // Send a message
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &m, orion_bms_address_claim, &mailbox);
  // Wait for message to be sent. This is usually not needed.
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
}

// We only send one CAN message each time to save CAN bus resources.
static uint8_t can_message_idx = 0;

void can_transmit_temperatures(void) {
  // Message header
  CAN_TxHeaderTypeDef m;
  // Use standard ID
  m.IDE = CAN_ID_STD;
  // Set message ID
  m.StdId = CELL_TAP_1_1_ID + can_message_idx + 4 * BOARD_ID;
  // Use CAN_RTR_DATA to transmit data
  // We may use CAN_RTR_REMOTE to request things to send data in the future
  m.RTR = CAN_RTR_DATA;
  // Amount of data
  m.DLC = CELL_TAP_1_1_DLC;

  // Data to send
  uint8_t data[8];

  switch(can_message_idx) {
    case 0:
      data[0] = (temperatures[0] >> 8) & 0xFF;
      data[1] = temperatures[0] & 0xFF;
      data[2] = (temperatures[1] >> 8) & 0xFF;
      data[3] = temperatures[1] & 0xFF;
      data[4] = (temperatures[2] >> 8) & 0xFF;
      data[5] = temperatures[2] & 0xFF;
      data[6] = (temperatures[3] >> 8) & 0xFF;
      data[7] = temperatures[3] & 0xFF;
      break;
    case 1:
      data[0] = (temperatures[4] >> 8) & 0xFF;
      data[1] = temperatures[4] & 0xFF;
      data[2] = (temperatures[5] >> 8) & 0xFF;
      data[3] = temperatures[5] & 0xFF;
      data[4] = (temperatures[6] >> 8) & 0xFF;
      data[5] = temperatures[6] & 0xFF;
      data[6] = (temperatures[7] >> 8) & 0xFF;
      data[7] = temperatures[7] & 0xFF;
      break;
    case 2:
      data[0] = (temperatures[8] >> 8) & 0xFF;
      data[1] = temperatures[8] & 0xFF;
      data[2] = (temperatures[9] >> 8) & 0xFF;
      data[3] = temperatures[9] & 0xFF;
      data[4] = (temperatures[10] >> 8) & 0xFF;
      data[5] = temperatures[10] & 0xFF;
      data[6] = (temperatures[11] >> 8) & 0xFF;
      data[7] = temperatures[11] & 0xFF;
      break;
    case 3:
      data[0] = (temperatures[12] >> 8) & 0xFF;
      data[1] = temperatures[12] & 0xFF;
      data[2] = (temperatures[13] >> 8) & 0xFF;
      data[3] = temperatures[13] & 0xFF;
      data[4] = (temperatures[14] >> 8) & 0xFF;
      data[5] = temperatures[14] & 0xFF;

      int16_t max_temp = INT16_MIN;
      for(int i = 0; i < 12; ++i) {
        int16_t temp = temperatures[i];
        if(temp > max_temp) {
          max_temp = temp;
        }
      }
      data[6] = (max_temp >> 8) & 0xFF;
      data[7] = max_temp & 0xFF;
      break;
    default:
      Error_Handler();
      break;
  }

  // Send a message
  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &m, data, &mailbox);
  // Wait for message to be sent. This is usually not needed.
  while (HAL_CAN_IsTxMessagePending(&hcan, mailbox));
  
  // Board 0 sends data to the BMS
  if(BOARD_ID == 0) {
    // only send orion data once all data is out
    if(can_message_idx == 3) {
      orion_data_ready = 1;
    }

    if(orion_data_ready) {
      can_transmit_orion_bms();
      reset_orion_bms_data();
    }

    // send address claim
    if(can_message_idx % 2 == 0) {
      can_transmit_orion_bms_address_claim();
    }
  }
  
  can_message_idx = (can_message_idx + 1) % 4;
}


uint8_t __fls_wr(const uint32_t *page, const uint32_t *buf, uint32_t len)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  uint32_t erase_err;
  FLASH_EraseInitTypeDef erase_page = {
      FLASH_TYPEERASE_PAGES,
      FLASH_BANK_1,
      (uint32_t)page, 1};

  if (HAL_OK != HAL_FLASHEx_Erase(&erase_page, &erase_err)) {
    return 1;
  }

  for (uint32_t i = 0; i < len; ++i)
  {
    HAL_StatusTypeDef res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)page, *buf);
    if (HAL_OK != res)
    {
      __NOP();
      return 2;
    }
    ++page;
    ++buf;
  }
  __NOP();

  return 0;
}

uint8_t fls_wr(const uint32_t *page, const uint32_t *buf, uint32_t len)
{
  // does flash equal buffer already?
  if (0 == memcmp(page, buf, 4 * len))
  {
    return 0;
  }

  HAL_FLASH_Unlock();
  uint8_t r = __fls_wr(page, buf, len);
  HAL_FLASH_Lock();
  if (r)
  {
    return r;
  }

  // verify
  if (0 != memcmp(page, buf, 4 * len))
  {
    return 10;
  }

  return 0;
}


uint32_t write_cal_data(struct ThermistorCal_T* data) {
  return fls_wr(CAL_TABLE, (uint32_t*) data, sizeof(struct ThermistorCal_T) / sizeof(uint32_t));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // necessary for bootloader
  SCB->VTOR = (uint32_t)0x08003000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  if(FLASH_VARS->board.id >= 10 || FLASH_VARS->board.id <= 16) {
    // Board id in the proper range :)
      BOARD_ID = FLASH_VARS->board.id - 10;
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_ADCEx_Calibration_Start(&hadc1);


  for(uint8_t i = 0; i < 12; ++i) {
    HAL_GPIO_WritePin(status_ports[i], status_pins[i], GPIO_PIN_SET);
    HAL_Delay(50);
  }
  for(uint8_t i = 0; i < 12; ++i) {
    HAL_GPIO_WritePin(status_ports[i], status_pins[i], GPIO_PIN_RESET);
    HAL_Delay(50);
  }

  struct ThermistorCal_T* current_cal = CAL_TABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_readings, N_ADC_CHANNELS);
    HAL_Delay(100);

    for(uint8_t i = 0; i < N_ADC_CHANNELS; ++i) {
      uint8_t channel = adc_to_channel_mapping[i];
      uint16_t reading = adc_readings[i];

      if(current_cal->abc_calibration) {
        float resistance = compute_resistance(reading);
        float temp = compute_temperature(&(current_cal->parameters[i]), resistance);
        temperatures[channel] = (int16_t) (temp * 100.0);
      } else {
        temperatures[channel] = reading;
      }
      if(channel < N_STATUS_LEDS){
        HAL_GPIO_WritePin(status_ports[channel], status_pins[channel], reading < ADC_ERR_LOW || reading > ADC_ERR_HIGH);
      }
    }
    
    // TODO Filtering sensor values
    can_transmit_temperatures();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 15;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Set up filter to accept all CAN messages
  CAN_FilterTypeDef sf;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
    Error_Handler();
  }
  
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  // Enables interrupts
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STAT6_Pin|STAT7_Pin|STAT8_Pin|STAT9_Pin
                          |STAT10_Pin|STAT11_Pin|STAT0_Pin|STAT1_Pin
                          |STAT2_Pin|STAT3_Pin|STAT4_Pin|STAT5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_STAT_GPIO_Port, SYS_STAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STAT6_Pin STAT7_Pin STAT8_Pin STAT9_Pin
                           STAT10_Pin STAT11_Pin STAT0_Pin STAT1_Pin
                           STAT2_Pin STAT3_Pin STAT4_Pin STAT5_Pin */
  GPIO_InitStruct.Pin = STAT6_Pin|STAT7_Pin|STAT8_Pin|STAT9_Pin
                          |STAT10_Pin|STAT11_Pin|STAT0_Pin|STAT1_Pin
                          |STAT2_Pin|STAT3_Pin|STAT4_Pin|STAT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_STAT_Pin */
  GPIO_InitStruct.Pin = SYS_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_STAT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
