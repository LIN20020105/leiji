#include "sample.h"

#include "adc.h"
#include "main.h"

// #include "dma.h"
#include "gpio.h"
#include "pack.h"
#include "rtc.h"
#include "usart.h"

#define ADC_BUFFER_SIZE 40
uint16_t adc_data_1[ADC_BUFFER_SIZE];
uint16_t adc_data_2[ADC_BUFFER_SIZE];
uint16_t adc_data_3[ADC_BUFFER_SIZE];
uint32_t adc1_idx = 0;
uint32_t adc2_idx = 0;
uint32_t adc3_idx = 0;

uint16_t thunder1 = 0, thunder2 = 0, battery = 0;

/**
 * @brief
 *
 * @param push_to_pkg
 * push_to_pkg: 0: not push to package, 1: push to package
 */
void sample_adc(int push_to_pkg) {
  //    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Buffer,ADC_CHANNELS);
  //    adc_data_1[adc1_idx] = ADC_Buffer[0];
  //    adc_data_2[adc2_idx] = ADC_Buffer[1];
  //    adc_data_3[adc3_idx] = ADC_Buffer[2];

  for (int i = 0; i < 3; ++i) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
      ADC_Buffer[i] = HAL_ADC_GetValue(&hadc1);
    }
  }
  thunder1 = ADC_Buffer[0];
  thunder2 = ADC_Buffer[0];
  battery = ADC_Buffer[0];

  if (push_to_pkg) {
    push_data(thunder1, thunder2, RTC_Get_Timestamp());
  }

  // For test. Can be deleted later
  adc_data_1[adc1_idx] = ADC_Buffer[0];
  adc_data_2[adc2_idx] = ADC_Buffer[1];
  adc_data_3[adc3_idx] = ADC_Buffer[2];

  adc1_idx = (adc1_idx + 1) % ADC_BUFFER_SIZE;
  adc2_idx = (adc2_idx + 1) % ADC_BUFFER_SIZE;
  adc3_idx = (adc3_idx + 1) % ADC_BUFFER_SIZE;

  // __HAL_DMA_CLEAR_FLAG(&hdma_adc1, DMA_FLAG_TC1);
}
