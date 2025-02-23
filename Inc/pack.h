#pragma once
#include "sample.h"

#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "rtc.h"
#include "usart.h"

typedef struct ThunderData {
  uint32_t thunder1;
  uint32_t timestamp;
} ThunderData;

typedef struct UPackage {
  uint32_t head;
  uint32_t uuid;
  uint32_t bytes;
  uint32_t timestamp;
  uint16_t battery;
  uint16_t nSize;
  uint32_t checksum;
} UPackage;

#define PAYLOAD_MAX_SIZE 512

void init_package(void);
void push_data(uint32_t thunder1, uint32_t ts);
void send_package(uint16_t battery, uint32_t timestamp);
