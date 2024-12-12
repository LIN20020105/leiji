#pragma once
#include "gpio.h"

#define DEV_INIT 0
#define DEV_WORK 1

#define ALARM_HOURS 0
#define ALARM_MINUTES 0
#define ALARM_SECDONS 120

#define IO_NB_NETMODE (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0 ? 0 : 1)
#define IO_NB_LINK (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0 ? 0 : 1)
