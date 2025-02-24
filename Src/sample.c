#include "sample.h"

#include "adc.h"
#include "main.h"

#include "gpio.h"
#include "pack.h"
#include "rtc.h"
#include "usart.h"

#define ADC_BUFFER_SIZE 40
uint16_t adc_data_1[ADC_BUFFER_SIZE];
uint16_t adc_data_2[ADC_BUFFER_SIZE];
uint32_t adc1_idx = 0;
uint32_t adc2_idx = 0;

uint32_t thunder1 = 0;
uint16_t battery = 0;

/**
 * @brief
 *
 * @param push_to_pkg
 * push_to_pkg: 0: not push to package, 1: push to package
 */
void sample_adc(int push_to_pkg) {
// adc_data_3[adc3_idx] = ADC_Buffer[2];

  for (int i = 0; i < 2; ++i) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
      ADC_Buffer[i] = HAL_ADC_GetValue(&hadc1);
    }
  }
  thunder1 = ADC_Buffer[1];
    battery = (ADC_Buffer[0]*1.731*100)/4200;
	if(battery > 100){
	battery = 100;}

  if (push_to_pkg) {
    push_data(thunder1,RTC_Get_Timestamp());
  }

  adc_data_1[adc1_idx] = ADC_Buffer[0];//battery
  adc_data_2[adc2_idx] = ADC_Buffer[1];//thunder

  adc1_idx = (adc1_idx + 1) % ADC_BUFFER_SIZE;
  adc2_idx = (adc2_idx + 1) % ADC_BUFFER_SIZE;

}
