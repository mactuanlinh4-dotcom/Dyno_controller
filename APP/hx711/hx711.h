/**
  ******************************************************************************
  * @file           : hx711.h
  * @brief          : Industrial-grade HX711 24-bit ADC Driver for STM32
  *                   Designed for Dyno Loadcell Force & Torque Measurement
  ******************************************************************************
  */

#ifndef __HX711_H
#define __HX711_H

#include "main.h"

/* ==================================================================== */
/* CẤU HÌNH & HẰNG SỐ CỐ ĐỊNH                                          */
/* ==================================================================== */

/**
 * @brief Chế độ Gain và Kênh ngõ vào của HX711
 */
typedef enum {
    HX711_GAIN_128 = 25,  // Kênh A, Gain 128 (Phổ biến nhất cho Loadcell)
    HX711_GAIN_32  = 26,  // Kênh B, Gain 32
    HX711_GAIN_64  = 27   // Kênh A, Gain 64
} HX711_Gain;

/**
 * @brief Hệ số lọc EMA (Exponential Moving Average) triệt tiêu rung động Dyno
 * Alpha = 0.10f cho kết quả phẳng mịn, phản ứng đủ nhanh ở tần số 80Hz/10Hz
 */
#define HX711_EMA_ALPHA         0.10f

/**
 * @brief Giá trị Scale Factor mặc định (ADC counts / kg) trước khi Calib
 */
#define HX711_1_SCALE_FACTOR    216560.0f  // Loadcell 1 (Bên Nén M+)
#define HX711_2_SCALE_FACTOR    216560.0f  // Loadcell 2 (Bên Kéo M-)
#define HX711_SCALE_FACTOR      216560.0f  // Giá trị mặc định chung

/* ==================================================================== */
/* CẤU TRÚC ĐỐI TƯỢNG (HANDLE STRUCT)                                   */
/* ==================================================================== */

typedef struct {
    // Cấu hình phần cứng GPIO
    GPIO_TypeDef *dout_port;
    uint16_t     dout_pin;
    GPIO_TypeDef *sck_port;
    uint16_t     sck_pin;

    // Cấu hình hoạt động
    HX711_Gain   gain;
    float        arm_length_m;      // Chiều dài cánh tay đòn Dyno (mét)

    // Thông số Hiệu chỉnh (Calibration)
    int32_t      offset;            // Giá trị ADC thô ở mốc 0 kg (Tare)
    float        scale_factor;      // Hệ số quy đổi (ADC_counts / kg)

    // Dữ liệu đầu ra thời gian thực
    int32_t      raw_adc;           // Giá trị ADC thô mới nhất đọc về
    float        raw_weight_kg;     // Khối lượng tức thời chưa lọc (kg)
    float        filtered_weight_kg;// Khối lượng đã lọc EMA (kg)
    float        measured_force_n;  // Lực cản thực tế (N)
    float        measured_torque_nm;// Mô-men xoắn thực tế trên tay đòn (N.m)

    // Trạng thái hệ thống
    uint8_t      is_ready;          // 1: Dữ liệu đã sẵn sàng, 0: Đang bận
    uint8_t      is_connected;      // 1: Cảm biến hoạt động bình thường, 0: Đứt dây/Lỗi
} HX711_HandleTypeDef;

/* ==================================================================== */
/* NGUYÊN MẪU HÀM (API PROTOTYPES)                                      */
/* ==================================================================== */

/**
 * @brief Khởi tạo đối tượng HX711
 * @param hx: Con trỏ tới cấu trúc HX711
 * @param dout_port, dout_pin: GPIO nhận dữ liệu (DOUT)
 * @param sck_port, sck_pin: GPIO xuất xung clock (PD_SCK)
 * @param arm_length_m: Chiều dài cánh tay đòn đo lực Dyno (mét)
 */
void HX711_Init(HX711_HandleTypeDef *hx, 
                GPIO_TypeDef *dout_port, uint16_t dout_pin, 
                GPIO_TypeDef *sck_port, uint16_t sck_pin, 
                float arm_length_m);

/**
 * @brief Kiểm tra xem HX711 đã hoàn thành chuyển đổi ADC chưa (DOUT == LOW)
 * @retval 1: Sẵn sàng đọc, 0: Bận
 */
uint8_t HX711_IsReady(HX711_HandleTypeDef *hx);

/**
 * @brief Đọc giá trị ADC thô 24-bit từ HX711 (Interrupt-Safe)
 * @note Hàm này thực hiện dịch 24 bit và phát xung thiết lập Gain
 * @retval Số nguyên 32-bit có dấu (2's complement extended)
 */
int32_t HX711_ReadRaw(HX711_HandleTypeDef *hx);

/**
 * @brief Đặt mốc 0 (Tare / Zero Offset) khi hệ thống chưa có tải
 * @param samples: Số mẫu lấy trung bình để lưu offset (Ví dụ: 20 mẫu)
 */
void HX711_Tare(HX711_HandleTypeDef *hx, uint8_t samples);

/**
 * @brief Hàm cập nhật dữ liệu Non-blocking (Gọi định kỳ trong Vòng lặp chính 1ms hoặc Timer)
 * @note Hàm tự động kiểm tra IsReady, đọc raw, mở rộng bit dấu, lọc EMA và tính Mô-men
 */
void HX711_Process(HX711_HandleTypeDef *hx);

/**
 * @brief Đưa HX711 vào chế độ tiết kiệm điện (Power Down)
 */
void HX711_PowerDown(HX711_HandleTypeDef *hx);

/**
 * @brief Đánh thức HX711 thoát khỏi chế độ Power Down
 */
void HX711_WakeUp(HX711_HandleTypeDef *hx);

#endif /* __HX711_H */