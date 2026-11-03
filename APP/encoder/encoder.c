#include "encoder.h"
#include <math.h>

#define PI_CONST 3.14159265358979323846f

void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim, float ppr) {
    enc->htim = htim;
    enc->ppr = (ppr > 0.0f) ? ppr : ENCODER_DEFAULT_PPR;
    enc->counts_per_rev = enc->ppr * ENCODER_QUADRATURE_FACTOR;
    enc->last_counter = 0;
    enc->last_time_ms = 0;
    enc->delta_counts = 0;
    enc->raw_rpm = 0.0f;
    enc->filtered_rpm = 0.0f;
    enc->omega_rad_s = 0.0f;
    enc->power_watts = 0.0f;
    enc->is_running = 0;
}

void Encoder_Start(Encoder_HandleTypeDef *enc) {
    if (enc->htim != NULL) {
        __HAL_TIM_SET_COUNTER(enc->htim, 0);
        enc->last_counter = 0;
        HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
        enc->is_running = 1;
    }
}

void Encoder_Stop(Encoder_HandleTypeDef *enc) {
    if (enc->htim != NULL) {
        HAL_TIM_Encoder_Stop(enc->htim, TIM_CHANNEL_ALL);
        enc->is_running = 0;
        enc->filtered_rpm = 0.0f;
        enc->omega_rad_s = 0.0f;
        enc->power_watts = 0.0f;
    }
}

void Encoder_Reset(Encoder_HandleTypeDef *enc) {
    if (enc->htim != NULL) {
        __HAL_TIM_SET_COUNTER(enc->htim, 0);
    }
    enc->last_counter = 0;
    enc->delta_counts = 0;
    enc->raw_rpm = 0.0f;
    enc->filtered_rpm = 0.0f;
    enc->omega_rad_s = 0.0f;
    enc->power_watts = 0.0f;
}

void Encoder_Process(Encoder_HandleTypeDef *enc, float dt_ms, float torque_nm) {
    if (enc->htim == NULL || dt_ms <= 0.0f) {
        return;
    }

    /* Đọc giá trị bộ đếm Timer 16-bit hiện tại */
    uint16_t current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);

    /* Ép kiểu sang int16_t tự động xử lý tràn số (overflow/underflow) của Timer 16-bit */
    enc->delta_counts = (int16_t)(current_counter - enc->last_counter);
    enc->last_counter = current_counter;

    /* Chuyển đổi dt từ ms sang giây */
    float dt_sec = dt_ms / 1000.0f;

    /* Tính tốc độ RPM tức thời: RPM = (delta_counts / counts_per_rev) * (60 / dt_sec) */
    enc->raw_rpm = ((float)enc->delta_counts / enc->counts_per_rev) * (60.0f / dt_sec);

    /* Lọc EMA (Exponential Moving Average) làm mượt số đo */
    enc->filtered_rpm = (ENCODER_SPEED_EMA_ALPHA * enc->raw_rpm) + 
                        ((1.0f - ENCODER_SPEED_EMA_ALPHA) * enc->filtered_rpm);

    /* Bộ lọc vùng chết (Deadband): Nếu tốc độ quá nhỏ (< 0.5 RPM) thì coi như đứng yên */
    if (fabsf(enc->filtered_rpm) < 0.5f) {
        enc->filtered_rpm = 0.0f;
    }

    /* Tính vận tốc góc omega (rad/s) = RPM * (2 * PI / 60) */
    enc->omega_rad_s = enc->filtered_rpm * (PI_CONST / 30.0f);

    /* Tính công suất cơ học Dyno: P = Torque (N.m) * |omega| (rad/s) */
    float abs_omega = fabsf(enc->omega_rad_s);
    float active_torque = (torque_nm > 0.0f) ? torque_nm : 0.0f;
    enc->power_watts = active_torque * abs_omega;
}

float Encoder_GetRPM(Encoder_HandleTypeDef *enc) {
    return enc->filtered_rpm;
}

float Encoder_GetOmega(Encoder_HandleTypeDef *enc) {
    return enc->omega_rad_s;
}

float Encoder_GetPower(Encoder_HandleTypeDef *enc) {
    return enc->power_watts;
}
