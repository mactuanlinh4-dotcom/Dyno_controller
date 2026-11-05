#include "hx711.h"

static inline void HX711_MicroDelay(void) {
    for (volatile int i = 0; i < 25; i++) {
        __NOP();
    }
}

void HX711_Init(HX711_HandleTypeDef *hx, 
                GPIO_TypeDef *dout_port, uint16_t dout_pin, 
                GPIO_TypeDef *sck_port, uint16_t sck_pin, 
                float arm_length_m) 
{
    hx->dout_port          = dout_port;
    hx->dout_pin           = dout_pin;
    hx->sck_port           = sck_port;
    hx->sck_pin            = sck_pin;
    hx->gain               = HX711_GAIN_128;
    hx->arm_length_m       = arm_length_m;
    hx->offset             = 0;
    hx->scale_factor       = HX711_SCALE_FACTOR;
    hx->raw_adc            = 0;
    hx->raw_weight_kg      = 0.0f;
    hx->filtered_weight_kg = 0.0f;
    hx->measured_force_n   = 0.0f;
    hx->measured_torque_nm = 0.0f;
    hx->is_ready           = 0;
    hx->is_connected       = 0;
    hx->last_ready_tick    = HAL_GetTick();

    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
}

uint8_t HX711_IsReady(HX711_HandleTypeDef *hx) {
    if (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin) == GPIO_PIN_RESET) {
        hx->is_ready = 1;
        hx->is_connected = 1;
        return 1;
    }
    hx->is_ready = 0;
    return 0;
}

int32_t HX711_ReadRaw(HX711_HandleTypeDef *hx) {
    uint32_t count = 0;

    __disable_irq();

    for (uint8_t i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
        HX711_MicroDelay();
        
        count = count << 1;
        
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
        HX711_MicroDelay();

        if (HAL_GPIO_ReadPin(hx->dout_port, hx->dout_pin) == GPIO_PIN_SET) {
            count++;
        }
    }

    for (uint8_t i = 0; i < (hx->gain - 24); i++) {
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
        HX711_MicroDelay();
        HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
        HX711_MicroDelay();
    }

    __enable_irq();

    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    hx->raw_adc = (int32_t)count;
    return hx->raw_adc;
}

void HX711_Tare(HX711_HandleTypeDef *hx, uint8_t samples) {
    if (samples == 0) samples = 1;
    int64_t sum = 0;
    uint8_t read_count = 0;
    uint32_t timeout = HAL_GetTick();

    while (read_count < samples) {
        if (HX711_IsReady(hx)) {
            sum += HX711_ReadRaw(hx);
            read_count++;
        }
        if (HAL_GetTick() - timeout > 6000) {
            hx->is_connected = 0;
            return;
        }
    }

    hx->offset = (int32_t)(sum / samples);
    hx->filtered_weight_kg = 0.0f;
    hx->measured_torque_nm = 0.0f;
}

void HX711_Process(HX711_HandleTypeDef *hx) {
    if (!HX711_IsReady(hx)) {
        // Đứt dây DOUT (pull-up giữ mức cao) => không bao giờ Ready
        if (HAL_GetTick() - hx->last_ready_tick > HX711_TIMEOUT_MS) {
            hx->is_connected       = 0;
            hx->filtered_weight_kg = 0.0f;
            hx->measured_force_n   = 0.0f;
            hx->measured_torque_nm = 0.0f;
        }
        return;
    }
    hx->last_ready_tick = HAL_GetTick();

    // Bảo vệ chống lỗi chia cho 0
    if (hx->scale_factor > -1.0f && hx->scale_factor < 1.0f) {
        hx->scale_factor = HX711_SCALE_FACTOR;
    }

    int32_t raw_val = HX711_ReadRaw(hx);

    hx->raw_weight_kg = (float)(raw_val - hx->offset) / hx->scale_factor;

    hx->filtered_weight_kg = (HX711_EMA_ALPHA * hx->raw_weight_kg) + 
                               ((1.0f - HX711_EMA_ALPHA) * hx->filtered_weight_kg);

    if (hx->filtered_weight_kg > -0.02f && hx->filtered_weight_kg < 0.02f) {
        hx->filtered_weight_kg = 0.0f;
    }

    hx->measured_force_n   = hx->filtered_weight_kg * 9.80665f;
    hx->measured_torque_nm = hx->measured_force_n * hx->arm_length_m;
}

void HX711_PowerDown(HX711_HandleTypeDef *hx) {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

void HX711_WakeUp(HX711_HandleTypeDef *hx) {
    HAL_GPIO_WritePin(hx->sck_port, hx->sck_pin, GPIO_PIN_RESET);
    hx->is_ready = 0;
}