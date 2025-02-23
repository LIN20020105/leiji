#ifndef __NBIOT_H__
#define __NBIOT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void send_NB_IoT(const char* cmd);
void Clear_Buffer(void);
void NB_IotConnect(void);
void nbiot_reset(void);
void NB_IoT_connect_MQTT(void);
uint8_t NB_IoT_ack_check(const char* str);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __NBIOT_H__ */
