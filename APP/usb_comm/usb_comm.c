#include "usb_comm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

volatile float g_target_torque_Nm = 0.0f;
volatile uint8_t g_cmd_received_flag = 0;
volatile float g_vehicle_mass_kg = ROAD_LOAD_DEFAULT_MASS_KG;
volatile float g_wheel_radius_m = ROAD_LOAD_DEFAULT_RADIUS_M;
volatile uint8_t g_road_sim_enabled = 0;

static char rx_line_buffer[64];
static uint8_t rx_index = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;

void USB_Comm_Init(void) {
    g_target_torque_Nm = 0.0f;
    g_cmd_received_flag = 0;
    rx_index = 0;
}

void USB_Comm_ProcessRxData(uint8_t* Buf, uint32_t Len) {
    for (uint32_t i = 0; i < Len; i++) {
        char c = (char)Buf[i];
        
        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_line_buffer[rx_index] = '\0';
                
                char *ptr = rx_line_buffer;
                char cmd = *ptr;
                if (cmd >= 'a' && cmd <= 'z') cmd -= 32;

                if (cmd == 'M') {
                    /* M<kg>: khối lượng xe */
                    float m = (float)atof(ptr + 1);
                    if (m > 0.0f) g_vehicle_mass_kg = m;
                } else if (cmd == 'R') {
                    /* R<m>: bán kính bánh xe */
                    float r = (float)atof(ptr + 1);
                    if (r > 0.0f) g_wheel_radius_m = r;
                } else if (cmd == 'S') {
                    /* S1: bật mô phỏng tải đường, S0: tắt và nhả phanh */
                    g_road_sim_enabled = (atoi(ptr + 1) != 0) ? 1 : 0;
                    if (!g_road_sim_enabled) g_target_torque_Nm = 0.0f;
                } else {
                    /* T<Nm> hoặc chỉ số: đặt mô-men thủ công (tắt mô phỏng) */
                    if (cmd == 'T') ptr++;
                    g_road_sim_enabled = 0;
                    g_target_torque_Nm = (float)atof(ptr);
                    if (g_target_torque_Nm > 200.0f) g_target_torque_Nm = 200.0f;
                    if (g_target_torque_Nm < 0.0f)   g_target_torque_Nm = 0.0f;
                }

                g_cmd_received_flag = 1;
                rx_index = 0;
            }
        } else if (rx_index < sizeof(rx_line_buffer) - 1) {
            rx_line_buffer[rx_index++] = c;
        }
    }
}

uint8_t USB_Comm_IsConnected(void) {
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1 : 0;
}

void Format_Float(char *buf, size_t size, float val, int decimals) {
    char sign[2] = "";
    if (val < 0.0f) {
        sign[0] = '-';
        sign[1] = '\0';
        val = -val;
    }
    int int_part = (int)val;
    float frac = val - (float)int_part;
    int mult = 1;
    for (int i = 0; i < decimals; i++) mult *= 10;
    int dec_part = (int)(frac * mult + 0.5f);
    if (dec_part >= mult) {
        int_part += 1;
        dec_part = 0;
    }
    if (decimals == 1) {
        snprintf(buf, size, "%s%d.%d", sign, int_part, dec_part);
    } else {
        snprintf(buf, size, "%s%d.%02d", sign, int_part, dec_part);
    }
}

void USB_Comm_SendTelemetry(float current_mA, 
                            float torque_current_nm, 
                            float torque_loadcell_nm, 
                            float pwm_duty_pct,
                            float speed_rpm,
                            float power_w,
                            float speed_kmh,
                            float road_force_n) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }

    char str_curr[16], str_tc[16], str_tlc[16], str_duty[16], str_rpm[16], str_pwr[16];
    char str_kmh[16], str_fw[16];
    Format_Float(str_curr, sizeof(str_curr), current_mA, 2);
    Format_Float(str_tc, sizeof(str_tc), torque_current_nm, 2);
    Format_Float(str_tlc, sizeof(str_tlc), torque_loadcell_nm, 2);
    Format_Float(str_duty, sizeof(str_duty), pwm_duty_pct, 1);
    Format_Float(str_rpm, sizeof(str_rpm), speed_rpm, 1);
    Format_Float(str_pwr, sizeof(str_pwr), power_w, 2);
    Format_Float(str_kmh, sizeof(str_kmh), speed_kmh, 1);
    Format_Float(str_fw, sizeof(str_fw), road_force_n, 1);

    static char tx_buffer[160];
    int len = snprintf(tx_buffer, sizeof(tx_buffer), 
                       "%s,%s,%s,%s,%s,%s,%s,%s\r\n",
                       str_curr, str_tc, str_tlc, str_duty, str_rpm, str_pwr,
                       str_kmh, str_fw);

    if (len > 0) {
        CDC_Transmit_FS((uint8_t*)tx_buffer, (uint16_t)len);
    }
}