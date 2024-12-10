#include "sample.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

#define ADC_BUFFER_SIZE 40
uint16_t adc_data_1[ADC_BUFFER_SIZE];
uint16_t adc_data_2[ADC_BUFFER_SIZE];
uint16_t adc_data_3[ADC_BUFFER_SIZE];
uint32_t adc1_idx = 0;
uint32_t adc2_idx = 0;
uint32_t adc3_idx = 0;
void sample_adc() {
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Buffer,ADC_CHANNELS);
    adc_data_1[adc1_idx] = ADC_Buffer[0];
    adc_data_2[adc1_idx] = ADC_Buffer[1];
    adc_data_3[adc1_idx] = ADC_Buffer[2];
    
    adc1_idx = (adc1_idx + 1) % ADC_BUFFER_SIZE;
    adc2_idx = (adc2_idx + 1) % ADC_BUFFER_SIZE;
    adc3_idx = (adc3_idx + 1) % ADC_BUFFER_SIZE;


}
