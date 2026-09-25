#include "usb_comm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile float g_target_torque_Nm = 0.0f;
volatile uint8_t g_cmd_received_flag = 0;

static char rx_line_buffer[64];
static uint8_t rx_index = 0;

void USB_Comm_Init(void) {
    g_target_torque_Nm = 0.0f;
    rx_index = 0;
}

// Hàm nhận dữ liệu từ callback USB CDC (Được gọi từ CDC_Receive_FS)
void USB_Comm_ProcessRxData(uint8_t* Buf, uint32_t Len) {
    for (uint32_t i = 0; i < Len; i++) {
        char c = (char)Buf[i];
        
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_line_buffer[rx_index] = '\0';
                
                // Xử lý loại bỏ ký tự 'T' hoặc 't' nếu có
                char *ptr = rx_line_buffer;
                if (*ptr == 'T' || *ptr == 't') {
                    ptr++;
                }
                
                // Chuyển đổi an toàn sang số thực
                g_target_torque_Nm = atof(ptr);
                
                // Giới hạn an toàn tuyệt đối cho phanh TJ-POD-20 (0 - 200 Nm)[cite: 10]
                if (g_target_torque_Nm > 200.0f) g_target_torque_Nm = 200.0f;
                if (g_target_torque_Nm < 0.0f) g_target_torque_Nm = 0.0f;
                
                g_cmd_received_flag = 1;
                rx_index = 0;
            }
        } else if (rx_index < sizeof(rx_line_buffer) - 1) {
            rx_line_buffer[rx_index++] = c;
        }
    }
}

extern USBD_HandleTypeDef hUsbDeviceFS;

void USB_Comm_SendTelemetry(float current_mA, float torque_Nm, float pwm_duty) {
    // 1. Kiểm tra trạng thái bận của USB CDC (Tránh nghẽn USBD_BUSY)
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return; // USB đang bận truyền gói trước, bỏ qua chu kỳ này
    }

    // 2. Tách phần nguyên và phần thập phân (Triệt tiêu lỗi %f bị GCC Newlib-nano chặn)
    int curr_int = (int)current_mA;
    int curr_dec = abs((int)(current_mA * 100.0f)) % 100;

    int torq_int = (int)torque_Nm;
    int torq_dec = abs((int)(torque_Nm * 100.0f)) % 100;

    int duty_int = (int)pwm_duty;
    int duty_dec = abs((int)(pwm_duty * 10.0f)) % 10;

    // 3. Đóng gói chuỗi CSV an toàn dạng số nguyên
    char tx_buffer[96];
    int len = snprintf(tx_buffer, sizeof(tx_buffer), "%d.%02d,%d.%02d,%d.%d\r\n", 
                       curr_int, curr_dec, torq_int, torq_dec, duty_int, duty_dec);

    // 4. Đẩy dữ liệu về PC
    CDC_Transmit_FS((uint8_t*)tx_buffer, (uint16_t)len);
}