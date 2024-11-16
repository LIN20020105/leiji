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
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nbiot.h"
#include "string.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char USART2_RxBuffer[RxBuffer_MaxSize];   //接收数据
uint8_t USART2_aRxBuffer;			//接收中断缓冲，用来一个字节一个字节接收
uint8_t UART2_Rx_Cnt = 0;		//接收缓冲计数

uint32_t ADC_Buffer[ADC_CHANNELS];//用来接收在外部中断时的电压值

uint32_t ADC_ALLBuffer[MAX_ADC_READINGS][ADC_CHANNELS]; // 二维数组用来存放15分钟内接收到的全部AD值
uint8_t adc_index = 0; // 当前ADC值索引

volatile uint32_t interrupt_count = 0; // 中断计数器
volatile uint8_t data_ready_flag = 0; // 数据处理标志位
//unsigned char RxFlag1;
//unsigned char RxFlag2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);//拉高电平，启动NBIot
  HAL_TIM_Base_Start_IT(&htim2); //开启定时器



nbiot_reset();
NB_IotConnect();
NB_IoT_connect_MQTT();
publishMQTTMessage();
// Pub_Msgdata("lin.123345");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  if (data_ready_flag) {
            data_ready_flag = 0; // 重置标志位
  for (uint8_t i = 0; i < adc_index; i++) {
            uint32_t val1 = ADC_ALLBuffer[i][0]; // 第一路ADC值
            uint32_t val2 = ADC_ALLBuffer[i][1]; // 第二路ADC值
            uint32_t val3 = ADC_ALLBuffer[i][2]; // 第三路ADC值
				uint32_t Volt1=(3300*val1)>>12;
				uint32_t Volt2=(3300*val2)>>12;
				uint32_t Volt3=(3300*val3)>>12; 

            send_ADC_Values(Volt1, Volt2, Volt3); // 发送ADC值
	        }
        adc_index = 0; // 重置索引以便下次接收新的ADC值

  }
 

  
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* 串口接收中断回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART2) 
	{
		USART2_RxBuffer[UART2_Rx_Cnt++] =USART2_aRxBuffer;   //接收数据转存
		if(USART2_RxBuffer[UART2_Rx_Cnt-1] == '\n'&& USART2_RxBuffer[UART2_Rx_Cnt-3] == 'K') //接收到OK时将接收标志位置0
			{
			UART2_Rx_Cnt = 0;
			}
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_aRxBuffer, 1);   //再开启接收中断
	}
}


/* 外部中断回调函数 上下降沿触发 在外部中断中进行采集和传输数据 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Buffer,ADC_CHANNELS);//开启ADC DMA采集	
			// 等待DMA传输完成
	while (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC1) == RESET) 
			{
			}
	__HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
		uint32_t val1 = ADC_Buffer[0]; // 第一路ADC值
		uint32_t val2 = ADC_Buffer[1]; // 第二路ADC值
		uint32_t val3 = ADC_Buffer[2]; // 第三路ADC值	
		uint32_t Volt1=(3300*val1)>>12;
		uint32_t Volt2=(3300*val2)>>12;
		uint32_t Volt3=(3300*val3)>>12; 

	    send_ADC_Values(Volt1, Volt2, Volt3);
	/*开启二维数组接收15分钟内所有AD值 */
	 if (adc_index < MAX_ADC_READINGS) 
		 {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_ALLBuffer[adc_index], ADC_CHANNELS); 
			adc_index++; 
		}
}

/* 定时器中断回调函数，定时器1s进入一次中断，计数900次即为15分钟，发送15分钟内接收到的全部数据 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM2)
	{
		    interrupt_count++; // 增加中断计数器
        if (interrupt_count == 600) { // 检查是否达到900次
            data_ready_flag = 1; // 设置数据处理标志位
            interrupt_count = 0; // 重置计数器
        }
	 }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
