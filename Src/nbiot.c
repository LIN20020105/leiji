#include "nbiot.h"
#include "usart.h"	
#include "string.h"	
#include "tim.h"	
#include "stdio.h"
#include <stdlib.h>

extern char USART2_RxBuffer[RxBuffer_MaxSize];   

char *strx;
char *strx1;
char *strx2;
extern uint8_t readingIndex ; // 当前ADC值索引
char time[5]; 

typedef struct {
    uint32_t adc1; // 第一路ADC值
    uint32_t adc2; // 第二路ADC值
    uint32_t adc3; // 第三路ADC值
    uint16_t rtc_time[6]; // RTC时间
    uint8_t reading_index; // 当前存储的索引
} DataPacket;

extern uint32_t ADC_Buffer[ADC_CHANNELS];
extern uint16_t MyRTC_Time[];
/* 模块发送命令 */
void send_NB_IoT(const char* cmd) {
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)cmd,strlen(cmd));
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

}
/* 模块重启 */
void nbiot_reset(void) {
    send_NB_IoT("AT+QRST=1\r\n");
    HAL_Delay(100);
	Clear_Buffer();
}

uint8_t NB_IoT_ack_check(const char* str) {
    HAL_Delay(100);
        if (strstr((const char*)USART2_RxBuffer, str)) 
			{
            return 1;
			} 
		else 
			{
            return 0;
			}
}
/* 清除接收缓冲区 */
void Clear_Buffer(void)
{
//memset(USART2_RxBuffer, 0, strlen((const char*)USART2_RxBuffer));
memset(USART2_RxBuffer, 0, 256);

}
/* 模块初始化 */
void NB_IotConnect(void) 
{
	send_NB_IoT("AT\r\n");
		HAL_Delay(300); 
		strx=strstr((const char*)USART2_RxBuffer,(const char*)"OK");//返回OK
		Clear_Buffer();
	while(strx==NULL)
    {
		Clear_Buffer();
		send_NB_IoT("AT\r\n");
        HAL_Delay(300);
        strx=strstr((const char*)USART2_RxBuffer,(const char*)"OK");//返回OK
    }

	send_NB_IoT("AT+CPIN?\r\n");
		HAL_Delay(300);
		Clear_Buffer();
	send_NB_IoT("ATE0&W\r\n");
		HAL_Delay(300); 
		Clear_Buffer();
	send_NB_IoT("AT+QMTDISC=0\r\n");
		HAL_Delay(300); 
		Clear_Buffer();
	send_NB_IoT("AT+QMTCLOSE=0\r\n");
		HAL_Delay(300); 
		Clear_Buffer();
	send_NB_IoT("AT+CFUN=1\r\n");
		HAL_Delay(300);
		Clear_Buffer();
	send_NB_IoT("AT+CIMI\r\n");
		HAL_Delay(300);
		strx=strstr((const char*)USART2_RxBuffer,(const char*)"460");//返460，表明识别到卡了
		Clear_Buffer();	
		while(strx==NULL)
    {
        Clear_Buffer();	
		send_NB_IoT("AT+CIMI\r\n");//获取卡号，类似是否存在卡的意思，比较重要。
        HAL_Delay(300);
        strx=strstr((const char*)USART2_RxBuffer,(const char*)"460");//返回OK,说明卡是存在的
    }
	send_NB_IoT("AT+CGATT=1\r\n");//激活网络，PDP
        HAL_Delay(300);
        strx=strstr((const char*)USART2_RxBuffer,(const char*)"OK");//返OK
        Clear_Buffer();	
	send_NB_IoT("AT+CGATT?\r\n");//查询激活状态
        HAL_Delay(300);
		strx=strstr((const char*)USART2_RxBuffer,(const char*)"+CGATT: 1");//返回1,说明激活
        Clear_Buffer();	
		while(strx==NULL)
		{
            Clear_Buffer();	
            send_NB_IoT("AT+CGATT?\r\n");//获取激活状态
            HAL_Delay(300);
            strx=strstr((const char*)USART2_RxBuffer,(const char*)"+CGATT: 1");//返回1,表明注网成功
		}
	send_NB_IoT("AT+CESQ\r\n");//查看获取CSQ值
		HAL_Delay(300);
		strx=strstr((const char*)USART2_RxBuffer,(const char*)"+CESQ");//返回CSQ
		Clear_Buffer();
		while(strx==NULL)
		{
            Clear_Buffer();	
            send_NB_IoT("AT+CESQ\r\n");//获取激活状态
            HAL_Delay(300);
            strx=strstr((const char*)USART2_RxBuffer,(const char*)"+CESQ");//返回1,表明信号可以
			Clear_Buffer();

		}
}
void NB_IoT_connect_MQTT(void) {
	send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
		HAL_Delay(500);
      while(NB_IoT_ack_check("+QMTOPEN: 0,0")!=1)
	  {
			Clear_Buffer();	
			send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
			HAL_Delay(500);
	  }
	  Clear_Buffer();	
	send_NB_IoT("AT+QMTCONN=0,\"861428040040199\"\r\n");
		HAL_Delay(500);
      while(NB_IoT_ack_check("+QMTCONN: 0,0,0")!=1)
	  {
			Clear_Buffer();	
			send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
			HAL_Delay(500);
			send_NB_IoT("AT+QMTCONN=0,\"861428040040199\"\r\n");
			HAL_Delay(500);
	  }
		Clear_Buffer();	
	send_NB_IoT("AT+QMTSUB=0,1,\"bc25\",0\r\n");
		HAL_Delay(500);
      while(NB_IoT_ack_check("+QMTSUB: 0,1,0,0")!=1)
	  {
			Clear_Buffer();	
			send_NB_IoT("AT+QMTSUB=0,1,\"bc25\",0\r\n");
			HAL_Delay(500);
	  }

	  	Clear_Buffer();
}










