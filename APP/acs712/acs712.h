#ifndef __ACS712_H
#define __ACS712_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define ACS712_SENSITIVITY 0.185f
#define ACS712_VREF        3.3f
#define ACS712_ADC_MAX     4095.0f

#define ACS712_EMA_ALPHA 0.05f

typedef struct{
    ADC_HandleTypeDef *hadc;
    uint16_t *dma_buffer;
    uint16_t buf_size;
    float adc_zero_offset;
    float current_raw;
    float current_filtered;
    float scale_k;
} ACS712_HandleTypeDef;

void ACS712_Init(ACS712_HandleTypeDef *dev, ADC_HandleTypeDef *hadc, uint16_t *dma_buf, uint16_t buf_size);
void ACS712_CalibrateZero(ACS712_HandleTypeDef *dev);
float ACS712_Process(ACS712_HandleTypeDef *dev);

#endif