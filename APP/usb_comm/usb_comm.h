#ifndef __USB_COMM_H
#define __USB_COMM_H

#include "main.h"
#include "usbd_cdc_if.h"
#include "road_load.h"

extern volatile float g_target_torque_Nm;
extern volatile uint8_t g_cmd_received_flag;
extern volatile float g_vehicle_mass_kg;
extern volatile float g_wheel_radius_m;
extern volatile uint8_t g_road_sim_enabled;

void USB_Comm_Init(void);
uint8_t USB_Comm_IsConnected(void);
/* In số thực không cần printf float (newlib-nano). decimals = 1 hoặc 2 */
void Format_Float(char *buf, size_t size, float val, int decimals);
void USB_Comm_ProcessRxData(uint8_t* Buf, uint32_t Len);
void USB_Comm_SendTelemetry(float current_mA, 
                            float torque_current_nm, 
                            float torque_loadcell_nm, 
                            float pwm_duty_pct,
                            float speed_rpm,
                            float power_w,
                            float speed_kmh,
                            float road_force_n);

#endif /* __USB_COMM_H */