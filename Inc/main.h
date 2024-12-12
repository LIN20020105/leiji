/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#define RxBuffer_MaxSize 512
#define ADC_CHANNELS 3

extern uint32_t ADC_Buffer[ADC_CHANNELS];

typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} RTC_TimeStruct;

// #pragma pack(push, 1)
// typedef struct ThunderDate {
//   uint32_t thunder1;
//   uint32_t thunder2;
//   uint32_t timestamp;
// } ThunderDate;
// typedef struct Package {
//   uint32_t head;
//   uint32_t sendtime;
//   uint16_t battery;
//   uint16_t nSize;
//   uint32_t checksum;
//   ThunderDate payload[100];
// } Package;
//#pragma pack(pop)

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
// void MyRTC_ReadTime(void);
// void UpdateRTC(int year, int month, int day, int hour, int minute, int
// second); void Get_nowtime(void);
uint32_t calculate_timestamp(RTC_TimeStruct *rtc_time);
static int is_leap_year(int year);
static int days_in_month(int year, int month);
// uint32_t get_timestamp(void);
// void serialize_package(const Package *pkg, uint8_t *hex_buffer, size_t
// *size); size_t calculate_package_size(const Package *pkg); void
// send_data(const Package *pkg); void serialize_all_package(const Package *pkg,
// uint8_t *hex_buffer,
//                            size_t *allsize);
// uint8_t Send_All_data(void);
// void Update_RTC_time(int year, int month, int day, int hour, int minute,
//                      int second);
// void HAL_RTCEx_RTCEventErrorCallback(RTC_HandleTypeDef *hrtc);
void sys_enter_stop_mode(void);
void sys_out_stop_mode(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
void SystemClock_Config(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
