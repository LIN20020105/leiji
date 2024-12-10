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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nbiot.h"
#include "string.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char USART1_RxBuffer[RxBuffer_MaxSize];   //��������
uint8_t USART1_aRxBuffer;			//�����жϻ��壬����һ���ֽ�һ���ֽڽ���
uint8_t UART1_Rx_Cnt = 0;		//���ջ������??

uint32_t ADC_Buffer[ADC_CHANNELS];//�����������ⲿ�ж�ʱ�ĵ�ѹֵ


uint8_t readingIndex = 0; // ��ǰ�洢������

uint16_t MyRTC_Time[] = {24, 11, 19, 12, 20, 00};
char Time_RxBuffer[RxBuffer_MaxSize];   //����ʱ������
    int day, month, year, hour, minute, second;//�洢���뵽����������

volatile uint32_t interrupt_count = 0; // �жϼ�����
volatile uint8_t data_ready_flag = 0; // ���ݴ�����־λ
volatile uint32_t RTC_interrupt_count = 0; // �жϼ�����
volatile uint8_t RTC_data_ready_flag = 0; // ���ݴ�����־λ

Package pkg;


uint16_t thunder_index = 0;
uint8_t stop_index = 0;//��������Ƿ�մ�ֹͣ״̬����
uint8_t Exti_index = 0;//�����жϵ�Ϊ�ⲿ�жϻ���ֹͣ����ʱ�����ó�ʼ��ADC��DMA


//uint8_t dev_state = DEV_INIT;

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
//  dev_state = DEV_INIT;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//  dev_state = DEV_WORK;
  
//  sys_enter_stop_mode();
//	sys_out_stop_mode();

//   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  
  
//__HAL_RTC_ALARM_ENABLE_IT(&hrtc,RTC_IT_ALRA);

//nbiot_reset();

//NB_IotConnect();
//NB_IoT_connect_MQTT();//���ӵ�MQTT

	Get_nowtime(); //��ȡ��ǰʱ�䲢��������RTCʱ�Ӻ�RTC�����ж�ʱ��15min��������


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // Send_All_data();//����15min���յ���ȫ�����ݣ���ʱ��15min����RTC����15min
//	if(Send_All_data() == 1)//�ж��Ƿ���ִ���귢��ȫ����������
//		{    
//		sys_enter_stop_mode();//������ȫ�����ݺ����ֹͣģʽ���ȴ���һ���ⲿ�жϻ��ѣ�RTC���ѣ�PA0�����ػ��ѣ�ע�ⶨʱ���жϲ��ܻ���ֹͣģʽ
//		sys_out_stop_mode();//���Ѻ����³�ʼ�����裬����nbiot����ʱ�������ӵ�MQTT�Լ�����RTC����Ϊ15min֮��
//		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* ���ڽ����жϻص����� */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART1) 
	{
		USART1_RxBuffer[UART1_Rx_Cnt++] =USART1_aRxBuffer;   //��������ת��
		if(USART1_RxBuffer[UART1_Rx_Cnt-1] == '\n'&& USART1_RxBuffer[UART1_Rx_Cnt-3] == 'K') //���յ�OKʱ�����ձ�־λ��0
			{
			UART1_Rx_Cnt = 0;
			}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART1_aRxBuffer, 1);   //�ٿ��������ж�
	}
//	    HAL_IWDG_Refresh(&hiwdg); // ˢ�¿��Ź���ʱ��
}

/* �ⲿ�жϻص����� �����ش��� ���׻���ƽʱ���� ���ⲿ�ж��н��вɼ��ʹ������� */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if(stop_index == 1)//���ֹͣ״̬Ϊ�ձ��ⲿ�жϻ��ѣ������ִ���жϻص�������ִ�л��Ѻ�����������������Ǹձ����ѣ�����������MQTT
		{
			HAL_Init();
			SystemClock_Config();
			MX_GPIO_Init();
			MX_DMA_Init();
			MX_ADC1_Init();
			MX_USART1_UART_Init();
			//  MX_IWDG_Init();
			MX_RTC_Init();
			HAL_ResumeTick();
			Exti_index = 1;
		}
		
	static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_interrupt_time) > 50) { // 50ms ȥ����ʱ��
        last_interrupt_time = current_time;
   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Buffer,ADC_CHANNELS);//����ADC DMA�ɼ�	
	while (__HAL_DMA_GET_FLAG(&hdma_adc1, DMA_FLAG_TC1) == RESET) 
			{
			}
	__HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
			
	pkg.nSize++;//��һ���׻� �׻������ͼ�һ
		pkg.payload[pkg.nSize-1].thunder1 = ADC_Buffer[0];
        pkg.payload[pkg.nSize-1].thunder2 = ADC_Buffer[1];
        pkg.payload[pkg.nSize-1].timestamp = get_timestamp();
			
	if(pkg.nSize  > 1000){
	pkg.nSize =1;}
		uint32_t battery_ad = ADC_Buffer[2]; // ���ad3	
		pkg.battery = (float)(battery_ad)/4095*100.0;//��ص���??
		pkg.head = 0x5a5a5a5a;
		pkg.sendtime = get_timestamp();
		pkg.checksum = 0;
		pkg.checksum = pkg.head + pkg.sendtime + pkg.battery + pkg.nSize;
	for(uint16_t i = 0;i <pkg.nSize;i++){
			pkg.checksum += pkg.payload[i].thunder1 + pkg.payload[i].thunder2 + pkg.payload[i].timestamp;
			}
	send_data(&pkg);
		}	
//    HAL_IWDG_Refresh(&hiwdg); // ˢ�¿��Ź���ʱ��
	
}

void send_data(const Package *pkg){
    uint8_t buffer[256];
	size_t size;

    serialize_package(pkg, buffer, &size);
	send_NB_IoT("ATO\r\n");
	HAL_UART_Transmit(&huart1, buffer,size, HAL_MAX_DELAY);
}

// ����ʵ�����ݴ�С
size_t calculate_package_size(const Package *pkg) {
	    return sizeof(pkg->head) + sizeof(pkg->sendtime) + sizeof(pkg->battery) + 
           sizeof(pkg->nSize) + sizeof(pkg->checksum) + sizeof(ThunderDate);
}

void serialize_package(const Package *pkg, uint8_t *hex_buffer, size_t *size) {
    memcpy(hex_buffer, &pkg->head, sizeof(pkg->head));
    memcpy(hex_buffer + sizeof(pkg->head), &pkg->sendtime, sizeof(pkg->sendtime));
    memcpy(hex_buffer + sizeof(pkg->head) + sizeof(pkg->sendtime), &pkg->battery, sizeof(pkg->battery));
    memcpy(hex_buffer + sizeof(pkg->head) + sizeof(pkg->sendtime) + sizeof(pkg->battery), &pkg->nSize, sizeof(pkg->nSize));
    memcpy(hex_buffer + sizeof(pkg->head) + sizeof(pkg->sendtime) + sizeof(pkg->battery) + sizeof(pkg->nSize), &pkg->checksum, sizeof(pkg->checksum));
	size_t offset = sizeof(pkg->head) + sizeof(pkg->sendtime) + sizeof(pkg->battery) + sizeof(pkg->nSize) + sizeof(pkg->checksum);
    memcpy(hex_buffer + offset, &pkg->payload[pkg->nSize - 1].thunder1, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(hex_buffer + offset, &pkg->payload[pkg->nSize - 1].thunder2, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(hex_buffer + offset, &pkg->payload[pkg->nSize - 1].timestamp, sizeof(uint32_t));
	*size = calculate_package_size(pkg);
}

void serialize_all_package(const Package *pkg, uint8_t *hex_buffer, size_t *allsize) {
    size_t offset = 0;
    memcpy(hex_buffer + offset, &pkg->head, sizeof(pkg->head));
    offset += sizeof(pkg->head);
    memcpy(hex_buffer + offset, &pkg->sendtime, sizeof(pkg->sendtime));
    offset += sizeof(pkg->sendtime);
//    memcpy(hex_buffer + offset, &pkg->nSize, sizeof(pkg->nSize));
//    offset += sizeof(pkg->nSize);
    memcpy(hex_buffer + offset, &pkg->checksum, sizeof(pkg->checksum));
    offset += sizeof(pkg->checksum);
    for (uint16_t i =1; i <= pkg->nSize; i++) {
		memcpy(hex_buffer + offset, &i, sizeof(i));
		offset += sizeof(i);
        memcpy(hex_buffer + offset, &pkg->payload[i-1].thunder1, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        memcpy(hex_buffer + offset, &pkg->payload[i-1].thunder2, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        memcpy(hex_buffer + offset, &pkg->payload[i-1].timestamp, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    }
    *allsize = offset;
}
//����15�����ڽ��յ���ȫ������
uint8_t Send_All_data(void){
	 uint8_t buffer[256]; // ������������С����Ӧʵ������
	size_t allsize;
	if(data_ready_flag==1 || RTC_data_ready_flag == 1){
		pkg.head = 0x5a5a5a5a;
        pkg.sendtime = get_timestamp();
//        pkg.nSize = pkg.nSize;
        pkg.checksum = 0;
        pkg.checksum = pkg.head + pkg.sendtime  + pkg.nSize;
		for (uint16_t i = 0; i < pkg.nSize; i++) {
            pkg.checksum += pkg.payload[i].thunder1 + pkg.payload[i].thunder2 + pkg.payload[i].timestamp;
			}
      //auto a = (uint8_t*) (&pkg)
    serialize_all_package(&pkg, buffer, &allsize);
	send_NB_IoT("ATO\r\n");
	HAL_UART_Transmit(&huart1, buffer,allsize, HAL_MAX_DELAY);
	data_ready_flag = 0; // ������ݴ�����־�?
	RTC_data_ready_flag = 0;
	Get_nowtime();	//������������Ϊ15���Ӻ�
	return 1;//�з������ݷ���1
	}
	return 0;//û�з������ݷ���0
}

//��ȡʱ�������??
uint32_t get_timestamp(){
	MyRTC_ReadTime();	
	RTC_TimeStruct rtc_time = {
        .year = MyRTC_Time[0],
        .month = MyRTC_Time[1],
        .day = MyRTC_Time[2],
        .hour = MyRTC_Time[3],
        .minute = MyRTC_Time[4],
        .second = MyRTC_Time[5]
    };
	   uint32_t timestamp = calculate_timestamp(&rtc_time);//ʱ���??
	return timestamp;
}



//RTCʱ���жϻص�����,15���Ӵ���һ�Σ������жϿ��Ի���
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
		RTC_data_ready_flag = 1;//���ñ�־λΪ1	
	
//	    HAL_IWDG_Refresh(&hiwdg); // ˢ�¿��Ź���ʱ��

}

//����RTC���ӵ����ӣ�ÿ15���ӽ���һ������
void Update_RTC_time(int year, int month, int day, int hour, int minute, int second){
		RTC_TimeTypeDef sTime;
		RTC_AlarmTypeDef sAlarm = {0};
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		minute += 1;
		if (minute >= 60)
			{
			minute -= 60;
			hour += 1;
		if (hour >= 24)
			{
			hour -= 24;
			day += 1;
			if (day > days_in_month(year, month))
				{
					day = 1;
					month += 1;
					if (month > 12)
					{
					month = 1;
					year += 1;
					}
				}
			}
			}			
		sAlarm.AlarmTime.Hours = ((hour / 10) << 4) | (hour % 10);
		sAlarm.AlarmTime.Minutes = (((minute) / 10) << 4) | (minute % 10) ;/* �����´���������ʱ���ǵ�ǰʱ���??15min֮�� */
		sAlarm.AlarmTime.Seconds = ((second / 10) << 4) | (second % 10);
		sAlarm.Alarm = RTC_ALARM_A;		
		HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);
}
//����ֹͣģʽ
void sys_enter_stop_mode(void){
    __HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);//��ֹ����wakeup����
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);//���PA0���ѱ�־
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);//ʹ��wakeup����
	//��ͣ�δ�ʱ�ӣ���ֹͨ���δ�ʱ���жϻ���
    HAL_SuspendTick();
	stop_index = 1;//ֹͣ״̬��־λ
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
//�˳�ֹͣģʽ����еĲ���??
void sys_out_stop_mode(void){
	stop_index = 0;
	SystemClock_Config();
//	MX_GPIO_Init();
//	MX_DMA_Init();
//	MX_ADC1_Init();
//	MX_USART1_UART_Init();
//	MX_RTC_Init();
	HAL_ResumeTick();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
//	if(Exti_index != 1){
//		SystemClock_Config();
//		MX_GPIO_Init();
//		MX_DMA_Init();
//		MX_ADC1_Init();
//		//MX_TIM2_Init();
//		MX_USART1_UART_Init();
//		// MX_USART2_UART_Init();
//		// MX_USART3_UART_Init();
//		MX_RTC_Init();
//		HAL_ResumeTick();
//	}
//	Exti_index = 0;
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);//���ߵ�ƽ������NBIot
//	//HAL_TIM_Base_Start_IT(&htim2); //������ʱ��
//	__HAL_RTC_ALARM_ENABLE_IT(&hrtc,RTC_IT_ALRA);
//	nbiot_reset();

//	NB_IotConnect();
//	NB_IoT_connect_MQTT();//���ӵ�MQTT

//	Get_nowtime(); //��ȡ��ǰʱ�䲢��������RTCʱ�Ӻ�RTC�����ж�ʱ��
}
//��ȡ��ǰ����ʱ�亯�����ҵ��ø���RTCʱ�Ӻ���
void Get_nowtime(void)
{
	send_NB_IoT("+++\r\n");
	send_NB_IoT("AT+TIME\r\n");
		while(NB_IoT_ack_check("OK")!=1 || USART1_RxBuffer[3] != 'T')
		{
			Clear_Buffer();
			send_NB_IoT("+++\r\n");
			send_NB_IoT("AT+TIME\r\n");
		}
		memcpy(Time_RxBuffer,&USART1_RxBuffer[10],17);//��USART2_RxBuffer�е�ʱ�����Time_RxBuffer
		if (sscanf(Time_RxBuffer, "%2d/%2d/%2d,%2d:%2d:%2d", 
               &year, &month, &day, &hour, &minute, &second) == 6) {
			UpdateRTC(year, month, day, hour, minute, second);   
			Update_RTC_time(year, month, day, hour, minute, second);
		}
//	    HAL_IWDG_Refresh(&hiwdg); // ˢ�¿��Ź���ʱ��
}

//RTC��ȡʱ�亯��
void MyRTC_ReadTime(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  MyRTC_Time[0] = sDate.Year;
  MyRTC_Time[1] = sDate.Month;
  MyRTC_Time[2] = sDate.Date;
  MyRTC_Time[3] = sTime.Hours;
  MyRTC_Time[4] = sTime.Minutes;
  MyRTC_Time[5] = sTime.Seconds;

}
// �ж��Ƿ�������
static int is_leap_year(int year) {
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

// ��ȡĳ��ĳ�µ�����
static int days_in_month(int year, int month) {
    const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && is_leap_year(year)) {
        return 29;
    }
    return days_per_month[month - 1];
}
// �����??2024��1��1��00:00:00��ָ�����ڵ�������,��ʱ���??
uint32_t calculate_timestamp(RTC_TimeStruct *rtc_time) {
	
    int year = rtc_time->year;
    int month = rtc_time->month;
    int day = rtc_time->day;
    int hour = rtc_time->hour;
    int minute = rtc_time->minute;
    int second = rtc_time->second;

    int total_days = 0;

    // �����??2024��1��1�յ�ָ����ݵ�������??
    for (int y = 2024; y < year; y++) {
        total_days += is_leap_year(y) ? 366 : 365;
    }

    // �����??2024��1��1�յ�ָ���·ݵ�������
    for (int m = 1; m < month; m++) {
        total_days += days_in_month(year, m);
    }

    // ����ָ�����ڵ�����
    total_days += day - 1;

    // ����������
    uint32_t total_seconds = total_days * 24 * 60 * 60 + hour * 60 * 60 + minute * 60 + second;

    return total_seconds;
}

//����RTCʱ�Ӻ���
void UpdateRTC(int year, int month, int day, int hour, int minute, int second)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
	sTime.Hours = ((hour / 10) << 4) | (hour % 10);
	sTime.Minutes = ((minute / 10) << 4) | (minute % 10);  
	sTime.Seconds = ((second / 10) << 4) | (second % 10);
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
  DateToUpdate.Month = RTC_MONTH_NOVEMBER;
  DateToUpdate.Date = ((day / 10) << 4) | (day % 10);
  DateToUpdate.Year = ((year / 10) << 4) | (year % 10);

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
//   HAL_IWDG_Refresh(&hiwdg); // ˢ�¿��Ź���ʱ��
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
