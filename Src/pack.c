#include "pack.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>

UPackage package;
ThunderData payload[PAYLOAD_MAX_SIZE];
extern uint32_t thunder1;
extern uint16_t battery;
extern uint32_t uuid;
extern uint32_t hashed_value;
void init_package() {
  package.head = 0xAA55AA55;
  package.bytes = 0;
  package.timestamp = 0;
  package.battery = 0;
  package.checksum = 0;
  package.nSize = 0;
  memset(payload, 0, sizeof(payload));
}

uint32_t calc_checksum() {
  uint32_t checksum = 0;
  checksum += package.head + package.nSize + package.timestamp +
              package.battery + package.uuid + package.bytes;
  for (int i = 0; i < package.nSize; i++) {
    checksum +=
        payload[i].thunder1+ payload[i].timestamp;
  }

  return checksum;
}

void clear_package() { init_package(); }
void push_data(uint32_t thunder1, uint32_t ts) {
  if (package.nSize < PAYLOAD_MAX_SIZE) {
    payload[package.nSize].thunder1 = thunder1;
    payload[package.nSize].timestamp = ts;
    package.nSize++;
  }
}

void send_package(uint16_t battery, uint32_t timestamp) {
  package.battery = battery;
  package.timestamp = timestamp;
  package.uuid = hashed_value;
  package.bytes = sizeof(UPackage) + package.nSize * sizeof(ThunderData);
  package.checksum = calc_checksum();

  // Send package head
  HAL_UART_Transmit(&huart1, (uint8_t *)(&package), sizeof(UPackage), 100);
  // Send payload
  if (package.nSize > 0) {
    HAL_UART_Transmit(&huart1, (uint8_t *)payload,
                      sizeof(ThunderData) * package.nSize, 100);
  }

  clear_package();
}
