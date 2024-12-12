/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "rtc.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "nbiot.h"
#include "pack.h"
#include "sample.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// uint8_t USART2_aRxBuffer;
// uint8_t UART2_Rx_Cnt = 0;
uint32_t ADC_Buffer[ADC_CHANNELS];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t dev_state = DEV_INIT;
uint8_t RTC_Alarm_flag = 0;
uint32_t rdy_send_time = 0;
uint8_t sample_flag = 0;
uint8_t enter_stop = 0;
uint32_t uuid = 0x0;
extern uint16_t battery;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t rtc_ts = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  uuid = HAL_GetDEVID();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  dev_state = DEV_INIT;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  dev_state = DEV_WORK;
  RTC_Set_Alarm();
  init_package();

  // Note: Init NB-iot If necessary:
  // nbiot_reset();
  // NB_IotConnect();
  // NB_IoT_connect_MQTT();

  // sys_enter_stop_mode();
  // sys_out_stop_mode();
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  while (1) {
    // RTC clk will use LSE (now LSI 40Khz is not accurate)
    rtc_ts = RTC_Get_Timestamp();
    static int RTC_LED = 1;

    if (RTC_Alarm_flag == 1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, RTC_LED);
      RTC_LED = !RTC_LED;

      // Send data if connected to server
      if (IO_NB_NETMODE && IO_NB_LINK) {
        sample_adc(0);
        send_package(battery, RTC_Get_Timestamp());
        RTC_Set_Alarm();
        RTC_Alarm_flag = 0;

      } else {
        // 1. Reset NBIOT_DTU If timeout (60s)
        if (uwTick - rdy_send_time >= 1000 * 60) {
          // rst action (by AT, or ?)

        }
        // 2. Reset Fail, cancel sending task and enter low-power mode
        else if (uwTick - rdy_send_time >= 1000 * 120) {
          RTC_Set_Alarm();
          RTC_Alarm_flag = 0;
        }
      }
    }
    if (sample_flag == 1) {
      sample_adc(1);
      sample_flag = 0;
    }

    // Here feed watchdog (enbale it when everything is finished)
    // Feed();

    // No more task:
    if (!sample_flag && !RTC_Alarm_flag) {
      // 1. MCU and NB-iot Enter low-power mode

      // sys_enter_stop_mode();

      // 2. Recovery MCU and NB-iot state
      // sys_out_stop_mode();
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// HAL lib USART handler not efficient, but it works
// So We use IRQ_handler called by assemble to receive data

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//   if (huart->Instance == USART1) {
//     USART1_RxBuffer[UART2_Rx_Cnt++] = USART2_aRxBuffer;
//     if (USART1_RxBuffer[UART2_Rx_Cnt - 1] == '\n' &&
//         USART1_RxBuffer[UART2_Rx_Cnt - 3] == 'K') {
//       UART2_Rx_Cnt = 0;
//     }
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART2_aRxBuffer, 1);
//   }
// }

void sys_enter_stop_mode(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();

  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  enter_stop = 1;

  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
  HAL_SuspendTick();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
void sys_out_stop_mode(void) {
  SystemClock_Config();
  MX_GPIO_Init();
  // MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  // MX_RTC_Init();
  HAL_ResumeTick();
}

// unuse
static int is_leap_year(int year) {
  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}
// unuse
static int days_in_month(int year, int month) {
  const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 2 && is_leap_year(year)) {
    return 29;
  }
  return days_per_month[month - 1];
}
// unuse
uint32_t calculate_timestamp(RTC_TimeStruct *rtc_time) {

  int year = rtc_time->year;
  int month = rtc_time->month;
  int day = rtc_time->day;
  int hour = rtc_time->hour;
  int minute = rtc_time->minute;
  int second = rtc_time->second;

  int total_days = 0;

  for (int y = 2024; y < year; y++) {
    total_days += is_leap_year(y) ? 366 : 365;
  }

  for (int m = 1; m < month; m++) {
    total_days += days_in_month(year, m);
  }

  total_days += day - 1;

  uint32_t total_seconds =
      total_days * 24 * 60 * 60 + hour * 60 * 60 + minute * 60 + second;

  return total_seconds;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
