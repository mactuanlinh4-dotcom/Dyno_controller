#include "acs712.h"
#include <math.h>

#define PI_F 3.14159265f

/* Trung bình buffer DMA, bỏ các mẫu lệch xa trung bình (xung nhiễu, chuyển mạch PWM) */
static float ACS712_RobustAverage(ACS712_HandleTypeDef *dev) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < dev->buf_size; i++) {
        sum += dev->dma_buffer[i];
    }
    float mean = (float)sum / (float)dev->buf_size;

    uint32_t sum_ok = 0;
    uint16_t n_ok = 0;
    for (uint16_t i = 0; i < dev->buf_size; i++) {
        float d = (float)dev->dma_buffer[i] - mean;
        if (d < ACS712_SPIKE_REJECT_COUNTS && d > -ACS712_SPIKE_REJECT_COUNTS) {
            sum_ok += dev->dma_buffer[i];
            n_ok++;
        }
    }
    /* Ít hơn một nửa số mẫu hợp lệ => dòng đang thay đổi nhanh, dùng trung bình thường */
    if (n_ok < dev->buf_size / 2) {
        return mean;
    }
    return (float)sum_ok / (float)n_ok;
}

static void ACS712_LPF_Init(ACS712_HandleTypeDef *dev) {
    /* Butterworth bậc 2 (Q = 0.7071), biến đổi song tuyến tính */
    const float Q = 0.70710678f;
    float K = tanf(PI_F * ACS712_LPF_CUTOFF_HZ / ACS712_SAMPLE_HZ);
    float norm = 1.0f / (1.0f + K / Q + K * K);
    dev->b0 = K * K * norm;
    dev->b1 = 2.0f * dev->b0;
    dev->b2 = dev->b0;
    dev->a1 = 2.0f * (K * K - 1.0f) * norm;
    dev->a2 = (1.0f - K / Q + K * K) * norm;
    dev->z1 = 0.0f;
    dev->z2 = 0.0f;
}

static float ACS712_LPF_Step(ACS712_HandleTypeDef *dev, float x) {
    float y = dev->b0 * x + dev->z1;
    dev->z1 = dev->b1 * x - dev->a1 * y + dev->z2;
    dev->z2 = dev->b2 * x - dev->a2 * y;
    return y;
}

void ACS712_Init(ACS712_HandleTypeDef *dev, ADC_HandleTypeDef *hadc, uint16_t *dma_buf, uint16_t buf_size) {
    dev->hadc= hadc;
    dev->dma_buffer = dma_buf;
    dev->buf_size = buf_size;
    dev->adc_zero_offset = 3102.27f;
    dev->adc_avg = dev->adc_zero_offset;
    dev->current_raw = 0.0f;
    dev->current_ctrl = 0.0f;
    dev->current_display = 0.0f;
    dev->current_filtered = 0.0f;
    dev->autozero_count_ms = 0;
    dev->scale_k = ACS712_VREF / (ACS712_ADC_MAX * ACS712_SENSITIVITY);
    ACS712_LPF_Init(dev);
}

void ACS712_CalibrateZero(ACS712_HandleTypeDef *dev){
    float total_sum = 0.0f;
    uint16_t iterations = 500;

    HAL_Delay(50); /* Chờ ADC/DMA và nguồn ổn định */
    for (uint16_t i = 0; i < iterations; i++){
        total_sum += ACS712_RobustAverage(dev);
        HAL_Delay(1);
    }
    dev->adc_zero_offset = total_sum / (float)iterations;
    dev->adc_avg = dev->adc_zero_offset;

    dev->current_raw = 0.0f;
    dev->current_ctrl = 0.0f;
    dev->current_display = 0.0f;
    dev->current_filtered = 0.0f;
    dev->z1 = 0.0f;
    dev->z2 = 0.0f;
}

float ACS712_Process(ACS712_HandleTypeDef *dev){
    dev->adc_avg = ACS712_RobustAverage(dev);
    dev->current_raw = (dev->adc_avg - dev->adc_zero_offset) * dev->scale_k;

    /* Dòng cho PID: Butterworth 100 Hz, trễ nhỏ */
    dev->current_ctrl = ACS712_LPF_Step(dev, dev->current_raw);
    dev->current_filtered = dev->current_ctrl;

    /* Dòng cho hiển thị: EMA mạnh + vùng chết */
    dev->current_display = (ACS712_DISPLAY_EMA_ALPHA * dev->current_ctrl) +
                           ((1.0f - ACS712_DISPLAY_EMA_ALPHA) * dev->current_display);

    return dev->current_ctrl;
}

void ACS712_AutoZero(ACS712_HandleTypeDef *dev, uint8_t brake_off) {
    if (!brake_off) {
        dev->autozero_count_ms = 0;
        return;
    }
    if (dev->autozero_count_ms < ACS712_AUTOZERO_DELAY_MS) {
        dev->autozero_count_ms++;
        return;
    }
    /* Phanh đã nhả đủ lâu => dòng thực bằng 0, mọi sai lệch là trôi zero */
    float err = dev->adc_avg - dev->adc_zero_offset;
    if (err < ACS712_AUTOZERO_MAX_COUNTS && err > -ACS712_AUTOZERO_MAX_COUNTS) {
        dev->adc_zero_offset += ACS712_AUTOZERO_ALPHA * err;
    }
}

float ACS712_GetDisplayCurrent(ACS712_HandleTypeDef *dev) {
    float i = dev->current_display;
    if (i < ACS712_DISPLAY_DEADBAND_A && i > -ACS712_DISPLAY_DEADBAND_A) {
        i = 0.0f;
    }
    return i;
}
