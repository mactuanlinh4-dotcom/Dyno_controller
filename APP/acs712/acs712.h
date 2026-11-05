#ifndef __ACS712_H
#define __ACS712_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define ACS712_SENSITIVITY 0.185f
#define ACS712_VREF        3.3f
#define ACS712_ADC_MAX     4095.0f

/* ==================== XỬ LÝ TÍN HIỆU ==================== */
/* Tần số gọi ACS712_Process (vòng điều khiển 1 kHz) */
#define ACS712_SAMPLE_HZ            1000.0f

/* (1) Loại xung nhiễu: mẫu DMA lệch khỏi trung bình quá ngưỡng này bị bỏ.
 * 1 A ~ 229.6 count => 60 count ~ 0.26 A */
#define ACS712_SPIKE_REJECT_COUNTS  60.0f

/* (2) Lọc thông thấp Butterworth bậc 2 cho vòng PID: đủ nhanh cho điều khiển,
 * cắt nhiễu cao tần (gợn PWM 20 kHz, nhiễu nội ACS712) */
#define ACS712_LPF_CUTOFF_HZ        100.0f

/* (3) Lọc EMA mạnh cho hiển thị/telemetry (tau ~ 1/(alpha*fs) = 50 ms) */
#define ACS712_DISPLAY_EMA_ALPHA    0.02f
#define ACS712_DISPLAY_DEADBAND_A   0.010f   /* |I| < 10 mA hiển thị 0 */

/* (4) Tự hiệu chỉnh zero khi phanh nhả (bù trôi nhiệt/nguồn của ACS712) */
#define ACS712_AUTOZERO_DELAY_MS    300      /* Chờ dòng cuộn phanh tắt hẳn (L/R) */
#define ACS712_AUTOZERO_ALPHA       0.002f   /* Bám zero chậm (tau ~ 0.5 s) */
#define ACS712_AUTOZERO_MAX_COUNTS  115.0f   /* Chỉ bám khi lệch < ~0.5 A */

typedef struct{
    ADC_HandleTypeDef *hadc;
    uint16_t *dma_buffer;
    uint16_t buf_size;

    float adc_zero_offset;
    float adc_avg;              // Trung bình ADC sau loại xung nhiễu (count)
    float scale_k;              // A / count

    float current_raw;          // Dòng chưa lọc (A)
    float current_ctrl;         // Dòng sau lọc Butterworth - dùng cho PID (A)
    float current_display;      // Dòng lọc mượt - dùng hiển thị (A)
    float current_filtered;     // = current_ctrl (giữ tương thích)

    /* Trạng thái bộ lọc biquad (Direct Form II Transposed) */
    float b0, b1, b2, a1, a2;
    float z1, z2;

    uint16_t autozero_count_ms;
} ACS712_HandleTypeDef;

void ACS712_Init(ACS712_HandleTypeDef *dev, ADC_HandleTypeDef *hadc, uint16_t *dma_buf, uint16_t buf_size);
void ACS712_CalibrateZero(ACS712_HandleTypeDef *dev);

/**
 * @brief Xử lý 1 chu kỳ (gọi đúng ACS712_SAMPLE_HZ)
 * @retval Dòng đã lọc cho vòng điều khiển (A)
 */
float ACS712_Process(ACS712_HandleTypeDef *dev);

/**
 * @brief Tự hiệu chỉnh zero, gọi mỗi chu kỳ sau ACS712_Process
 * @param brake_off: 1 nếu phanh đang nhả (duty = 0)
 */
void ACS712_AutoZero(ACS712_HandleTypeDef *dev, uint8_t brake_off);

/**
 * @brief Dòng đã lọc mượt cho hiển thị/telemetry (A), có vùng chết
 */
float ACS712_GetDisplayCurrent(ACS712_HandleTypeDef *dev);

#endif
