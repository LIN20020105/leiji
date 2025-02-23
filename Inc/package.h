/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PACKAGE_H__
#define __PACKAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
typedef struct ThunderDate{
	uint32_t thunder1;
	uint32_t thunder2;
	uint32_t timestamp;
}ThunderDate;
ThunderDate thunderData;
typedef struct Package{
	uint32_t head;
	uint32_t sendtime;
	uint16_t battery;
	uint16_t nSize;
	uint32_t checksum;
	ThunderDate payload[1000];
//	uint8_t payload[1000];
}Package;
Package pkg;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __PACKAGE_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
