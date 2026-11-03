#include "usb_comm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

volatile float g_target_torque_Nm = 0.0f;
volatile uint8_t g_cmd_received_flag = 0;

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
                if (*ptr == 'T' || *ptr == 't') {
                    ptr++;
                }
                
                g_target_torque_Nm = (float)atof(ptr);
                
                if (g_target_torque_Nm > 200.0f) g_target_torque_Nm = 200.0f;
                if (g_target_torque_Nm < 0.0f)   g_target_torque_Nm = 0.0f;
                
                g_cmd_received_flag = 1;
                rx_index = 0;
            }
        } else if (rx_index < sizeof(rx_line_buffer) - 1) {
            rx_line_buffer[rx_index++] = c;
        }
    }
}

static void Format_Float(char *buf, size_t size, float val, int decimals) {
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
                            float power_w) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0) {
        return;
    }

    char str_curr[16], str_tc[16], str_tlc[16], str_duty[16], str_rpm[16], str_pwr[16];
    Format_Float(str_curr, sizeof(str_curr), current_mA, 2);
    Format_Float(str_tc, sizeof(str_tc), torque_current_nm, 2);
    Format_Float(str_tlc, sizeof(str_tlc), torque_loadcell_nm, 2);
    Format_Float(str_duty, sizeof(str_duty), pwm_duty_pct, 1);
    Format_Float(str_rpm, sizeof(str_rpm), speed_rpm, 1);
    Format_Float(str_pwr, sizeof(str_pwr), power_w, 2);

    static char tx_buffer[160];
    int len = snprintf(tx_buffer, sizeof(tx_buffer), 
                       "%s,%s,%s,%s,%s,%s\r\n", 
                       str_curr, str_tc, str_tlc, str_duty, str_rpm, str_pwr);

    if (len > 0) {
        CDC_Transmit_FS((uint8_t*)tx_buffer, (uint16_t)len);
    }
}