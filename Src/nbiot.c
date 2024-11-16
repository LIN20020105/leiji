#include "nbiot.h"
#include "usart.h"	
#include "string.h"	
#include "tim.h"	
#include "stdio.h"

extern char USART2_RxBuffer[RxBuffer_MaxSize];   

char *strx;
char *strx1;
char *strx2;

//#define IPdata      "47.92.146.210"
//#define ClientID     "861428040040199"             //ClientID
//#define Username          "TPW48OE83Ym001;12010126;9IOP8;1700640653"      //
//#define Password    "17332ccf30b28d986d44bdfce1e0e3e94d40bc4b1d923e72906985ef152b2698;hmacsha256"  //
//#define PubTopic         "bc25"
//#define SubTopic        "bc25"

extern uint32_t ADC_Buffer[ADC_CHANNELS];
/* 模块发送命令 */
void send_NB_IoT(const char* cmd) {
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)cmd,strlen(cmd));
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

//	HAL_Delay(300);
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
		send_NB_IoT("AT+CCLK=?\r\n");
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
/* 模块连接到MQTT */
//void NB_IoT_connect_MQTT(void) {

//	send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
//		HAL_Delay(500);
//		strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTOPEN: 0,0");//检查是否打开成功
//      while(strx==NULL)
//		{
////			Clear_Buffer();	
////			send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
////			HAL_Delay(500);
//            strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTOPEN: 0,0");//检查是否打开成功
//            HAL_Delay(500);
//		}  	
//		Clear_Buffer();	
//	send_NB_IoT("AT+QMTCONN=0,\"861428040040199\"\r\n");
//		HAL_Delay(500);
////		strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTCONN: 0,0,0");//检查是否连接成功
//				strx=strstr((const char*)USART2_RxBuffer,(const char*)"OK");//检查是否连接成功

//		 while(strx==NULL)
//		{
////			send_NB_IoT("AT+QMTOPEN=0,\"47.92.146.210\",1883\r\n");
////			HAL_Delay(500);	
////			Clear_Buffer();
////			send_NB_IoT("AT+QMTCONN=0,\"861428040040199\"\r\n");
////			HAL_Delay(500);
//            strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTCONN: 0,0,0");//检查是否连接成功
//            HAL_Delay(300);
//		}  	
//		Clear_Buffer();	
//	send_NB_IoT("AT+QMTSUB=0,1,\"bc25\",0\r\n");
//		HAL_Delay(500);
//		strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTSUB: 0,1,0,0");//检查是否登陆成功
//		 while(strx==NULL)
//		{
////			Clear_Buffer();	
////			send_NB_IoT("AT+QMTSUB=0,1,\"bc25\",0\r\n");
////			HAL_Delay(500);
//            strx=strstr((const char*)USART2_RxBuffer,(const char*)"+QMTSUB: 0,1,0,0");//检查是否登陆成功
//            HAL_Delay(500);
//		}  	
//		Clear_Buffer();	
//}

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
/* 发布测试命令到MQTT */
void publishMQTTMessage(void) {
	uint8_t hexValue = 0x1A;
	send_NB_IoT("AT+QMTPUB=0,0,0,0,\"bc25\"\r\n");
		HAL_Delay(100);  
	send_NB_IoT("lin.1234.");
		HAL_Delay(100);  
	HAL_UART_Transmit_IT(&huart2, &hexValue, 1);
}
/* 发布数组数据 */
void send_ADC_Values(uint32_t adc1, uint32_t adc2, uint32_t adc3) {
    char buffer[100];
    uint8_t hexValue = 0x1A;	
	send_NB_IoT("AT+QMTPUB=0,0,0,0,\"bc25\"\r\n");
//	HAL_Delay(100);     
    // 格式化并发送ADC值
    snprintf(buffer, sizeof(buffer), "ADC1:%u,ADC2:%u,ADC3:%u", adc1, adc2, adc3);
    send_NB_IoT(buffer);
//    HAL_Delay(100);    
    // 发送消息结束字符
    HAL_UART_Transmit_IT(&huart2, &hexValue, 1);
}

//void Pub_Msgdata(const char *message)
//{
//	uint8_t hexValue = 0x1A;
//	send_NB_IoT("AT+QMTPUB=0,0,0,0,\"bc25\"\r\n");
//		HAL_Delay(100);  
//	send_NB_IoT(message);       // 发送消息内容
//		HAL_Delay(100);  
//    HAL_UART_Transmit_IT(&huart2, &hexValue, 1); // 发送十六进制值

//}




