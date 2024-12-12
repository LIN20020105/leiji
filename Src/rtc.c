/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include "config.h"
uint32_t sec_prescaler = 1;
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
	sec_prescaler = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_RTC);
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_JUNE;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */
  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// RTC_TimeTypeDef RTC_Time;

void RTC_Update_Second(RTC_TimeTypeDef *rtc_time, int sec) {
  int nsec = 1, nmin = 1, nhour = 1;

  // limit maximun alarm time
  if (sec > 60 * 60 * 6)
    sec = 60 * 60 * 6;

  nsec = (int)(rtc_time->Seconds) + sec;
  nmin = (int)(rtc_time->Minutes) + nsec / 60;
  nhour = (int)(rtc_time->Hours) + nmin / 60;
  rtc_time->Seconds = nsec % 60;
  rtc_time->Minutes = nmin % 60;
  rtc_time->Hours = nhour % 24;
}

uint32_t RTC_Get_Timestamp() {
  // uint32_t sec = 0, min = 0, hour = 0, day = 0, month = 0, year = 0;
  uint32_t ts = 0;
  // RTC_DateTypeDef sDate;
  RTC_TimeTypeDef sTime;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  // HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  ts = sTime.Seconds + sTime.Minutes * 60 + sTime.Hours * 3600;
  ts = ts * 1000;
	
	
  // calc ms from RTC
  uint32_t period = (RTC->DIVH << 16) | RTC->DIVL;
  uint32_t ms = (sec_prescaler - period) * 1000 / sec_prescaler;
  return ts + ms;
}
void RTC_Set_Alarm() {

  RTC_TimeTypeDef sTime;
  // sTime.Hours = 0;
  // sTime.Minutes = 0;
  // sTime.Seconds = 0;
  // if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
  //   Error_Handler();
  // }

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  RTC_Update_Second(&sTime, ALARM_SECDONS);

  RTC_AlarmTypeDef sAlarmTime;
  sAlarmTime.Alarm = RTC_ALARM_A;
  sAlarmTime.AlarmTime.Hours = sTime.Hours;
  sAlarmTime.AlarmTime.Minutes = sTime.Minutes;
  sAlarmTime.AlarmTime.Seconds = sTime.Seconds;

  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarmTime, RTC_FORMAT_BIN);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
