/**
  ******************************************************************************
  * @file           : encoder.h
  * @brief          : High-precision Incremental Rotary Encoder Driver for STM32
  *                   Designed for Dyno Shaft Speed (RPM) and Mechanical Power (Watts)
  ******************************************************************************
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* Hằng số mặc định cho Encoder BKS3806B-2000-F2 */
#define ENCODER_DEFAULT_PPR         2000.0f
/* Chế độ đếm 4X (TI1 and TI2) cho 8000 counts / vòng */
#define ENCODER_QUADRATURE_FACTOR   4.0f
/* Hệ số lọc EMA cho tốc độ (alpha = 0.20f giúp triệt tiêu rung động cơ khí mà vẫn đáp ứng nhanh) */
#define ENCODER_SPEED_EMA_ALPHA     0.20f

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t          last_counter;
    uint32_t          last_time_ms;
    float             ppr;              // Xung mỗi vòng (PPR = 2000)
    float             counts_per_rev;   // Số count mỗi vòng (4X = 8000)
    
    int16_t           delta_counts;     // Số xung chênh lệch trong chu kỳ lấy mẫu
    float             raw_rpm;          // Vận tốc quay tức thời chưa lọc (RPM)
    float             filtered_rpm;     // Vận tốc quay đã lọc EMA (RPM)
    float             omega_rad_s;      // Vận tốc góc omega (rad/s)
    float             power_watts;      // Công suất cơ học P = Tau * omega (Watt)
    
    uint8_t           is_running;
} Encoder_HandleTypeDef;

/**
 * @brief Khởi tạo đối tượng Encoder
 * @param enc: Con trỏ tới cấu trúc Encoder
 * @param htim: Con trỏ tới Timer phần cứng (ví dụ: &htim3)
 * @param ppr: Số xung mỗi vòng (2000.0f)
 */
void Encoder_Init(Encoder_HandleTypeDef *enc, TIM_HandleTypeDef *htim, float ppr);

/**
 * @brief Bắt đầu đếm xung phần cứng Timer
 */
void Encoder_Start(Encoder_HandleTypeDef *enc);

/**
 * @brief Dừng đếm xung phần cứng Timer
 */
void Encoder_Stop(Encoder_HandleTypeDef *enc);

/**
 * @brief Reset bộ đếm xung và tốc độ về 0
 */
void Encoder_Reset(Encoder_HandleTypeDef *enc);

/**
 * @brief Xử lý tính toán tốc độ vòng quay và công suất cơ học
 * @note Hàm này được gọi định kỳ trong vòng lặp chính (ví dụ chu kỳ dt_ms = 10ms hoặc 20ms)
 * @param enc: Con trỏ tới cấu trúc Encoder
 * @param dt_ms: Khoảng thời gian chu kỳ tính toán (ms)
 * @param torque_nm: Mô-men xoắn thực tế đo được từ Dyno (N.m)
 */
void Encoder_Process(Encoder_HandleTypeDef *enc, float dt_ms, float torque_nm);

/**
 * @brief Lấy tốc độ quay đã lọc (RPM)
 */
float Encoder_GetRPM(Encoder_HandleTypeDef *enc);

/**
 * @brief Lấy vận tốc góc (rad/s)
 */
float Encoder_GetOmega(Encoder_HandleTypeDef *enc);

/**
 * @brief Lấy công suất cơ học (Watt)
 */
float Encoder_GetPower(Encoder_HandleTypeDef *enc);

#endif /* __ENCODER_H */
