#ifndef __USB_COMM_H
#define __USB_COMM_H

#include "main.h"
#include "usbd_cdc_if.h"

extern volatile float g_target_torque_Nm;
extern volatile uint8_t g_cmd_received_flag;

void USB_Comm_Init(void);
void USB_Comm_ProcessRxData(uint8_t* Buf, uint32_t Len);
void USB_Comm_SendTelemetry(float current_mA, 
                            float torque_current_nm, 
                            float torque_loadcell_nm, 
                            float pwm_duty_pct,
                            float speed_rpm,
                            float power_w);

#endif /* __USB_COMM_H */