#include "acs712.h"

void ACS712_Init(ACS712_HandleTypeDef *dev, ADC_HandleTypeDef *hadc, uint16_t *dma_buf, uint16_t buf_size) {
    dev->hadc= hadc;
    dev->dma_buffer = dma_buf;
    dev->buf_size = buf_size;
    dev->adc_zero_offset = 3102.27f;
    dev->current_raw = 0.0f;
    dev->current_filtered = 0.0f;
    dev->scale_k = ACS712_VREF / (ACS712_ADC_MAX * ACS712_SENSITIVITY);
}

void ACS712_CalibrateZero(ACS712_HandleTypeDef *dev){
    uint32_t total_sum = 0;
    uint16_t iterations = 300;

    for (uint16_t i = 0; i < iterations; i++){
        uint32_t buf_sum = 0;
        for (uint16_t j = 0; j < dev->buf_size; j++){
            buf_sum += dev->dma_buffer[j];
        }
        total_sum +=(buf_sum/dev->buf_size);
        HAL_Delay(1);
    }

    dev->adc_zero_offset = (float)total_sum/ (float)iterations;
}

float ACS712_Process(ACS712_HandleTypeDef *dev){
    uint32_t buf_sum = 0;
    for (uint16_t i=0; i < dev->buf_size; i++){
        buf_sum += dev->dma_buffer[i];
    }
    float raw_adc_avg = (float)buf_sum/(float)dev->buf_size;

    dev->current_raw = (raw_adc_avg - dev->adc_zero_offset)*dev->scale_k;

    dev->current_filtered = (ACS712_EMA_ALPHA * dev->current_raw) +((1.0f - ACS712_EMA_ALPHA)*dev->current_filtered);

    if (dev->current_filtered > -0.001f && dev->current_filtered < 0.001f){
        dev->current_filtered = 0.0f;
    }
    return dev->current_filtered;
}



