#include "nbiot.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include <stdlib.h>

extern char USART1_RxBuffer[RxBuffer_MaxSize];

char *strx;

void send_NB_IoT(const char *cmd) {
  // HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}
void nbiot_reset(void) {
  send_NB_IoT("AT+CFUN=1,1\r\n");
  HAL_Delay(100);
  Clear_Buffer();
}

uint8_t NB_IoT_ack_check(const char *str) {
  HAL_Delay(100);
  if (strstr((const char *)USART1_RxBuffer, str)) {
    return 1;
  } else {
    return 0;
  }
}
void Clear_Buffer(void) { memset(USART1_RxBuffer, 0, 256); }
void NB_IotConnect(void) {
  send_NB_IoT("+++\r\n");
  HAL_Delay(300);
  strx = strstr((const char *)USART1_RxBuffer, (const char *)"OK");
  Clear_Buffer();
  while (strx == NULL) {
    Clear_Buffer();
    send_NB_IoT("+++\r\n");
    HAL_Delay(300);
    strx = strstr((const char *)USART1_RxBuffer, (const char *)"OK");
  }

  send_NB_IoT("AT\r\n");
  HAL_Delay(300);
  strx = strstr((const char *)USART1_RxBuffer, (const char *)"OK");
  Clear_Buffer();
  while (strx == NULL) {
    Clear_Buffer();
    send_NB_IoT("AT\r\n");
    HAL_Delay(300);
    strx = strstr((const char *)USART1_RxBuffer, (const char *)"OK");
  }
  send_NB_IoT("AT+DTUMODE=2,1\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+AUTOSTATUS=1,1\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+KEEPALIVE=60,0,\"keepalive\",1\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+HEARTDODGE=0,1\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT&W\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+CPIN\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+CSQ\r\n");
  HAL_Delay(300);
  Clear_Buffer();
  send_NB_IoT("AT+ASKCONNECT?\r\n");
  HAL_Delay(300);
  Clear_Buffer();
}
void NB_IoT_connect_MQTT(void) {
  send_NB_IoT("AT+IPPORTX=\"47.99.152.116\",18883,1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+IPPORTX=\"47.99.152.116\",18883,1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
  send_NB_IoT("AT+CLIENTID=\"B47314F356D1\",1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+IPPORTX=\"47.99.152.116\",18883,1\r\n");
    HAL_Delay(300);
    send_NB_IoT("AT+CLIENTID=\"B47314F356D1\",1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
  send_NB_IoT("AT+USERPWD=\"A\",\"B\",1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+USERPWD=\"A\",\"B\",1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
  send_NB_IoT("AT+MQTTSUB=1,\"subTopic\",0,1,1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+MQTTSUB=1,\"subTopic\",0,1,1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
  //	send_NB_IoT("AT+MQTTPUB=1,\"pubTopic\",0,1,1\r\n");
  //	HAL_Delay(300);
  //      while(NB_IoT_ack_check("OK")!=1)
  //	  {
  //			Clear_Buffer();
  //			send_NB_IoT("AT+MQTTPUB=1,\"pubTopic\",0,1,1\r\n");
  //		  	HAL_Delay(300);
  //	  }
  //	  	Clear_Buffer();
  send_NB_IoT("AT+MQTTKEEP=120,1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+MQTTKEEP=120,1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
  send_NB_IoT("AT+BLOCKINFO=1,1\r\n");
  HAL_Delay(300);
  while (NB_IoT_ack_check("OK") != 1) {
    Clear_Buffer();
    send_NB_IoT("AT+BLOCKINFO=1,1\r\n");
    HAL_Delay(300);
  }
  Clear_Buffer();
}
