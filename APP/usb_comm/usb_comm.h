#ifndef __USB_COMM_H
#define __USB_COMM_H

#include "main.h"
#include "usbd_cdc_if.h"

// Biến lưu thông số mục tiêu và phản hồi
extern volatile float g_target_torque_Nm;  // Mô-men xoắn mục tiêu đặt từ PC
extern volatile uint8_t g_cmd_received_flag;

/* API Giao tiếp USB */
void USB_Comm_Init(void);
void USB_Comm_ProcessRxData(uint8_t* Buf, uint32_t Len);
void USB_Comm_SendTelemetry(float current_mA, float torque_Nm, float pwm_duty);

#endif /* __USB_COMM_H */