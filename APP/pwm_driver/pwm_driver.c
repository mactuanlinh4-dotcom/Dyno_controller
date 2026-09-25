#include "pwm_driver.h"
#include "stm32g4xx_hal_tim.h"
#include <math.h>
#define TJ_POD_MAX_TORQUE_NM    200.0f

void PWM_Driver_Init(PWM_Driver_HandleTypeDef *driver, TIM_HandleTypeDef *htim, uint32_t channel){
    driver->htim = htim;
    driver->channel = channel;
    driver->is_running = 0;
    driver->current_duty = 0.0f;
}

void PWM_Driver_Start(PWM_Driver_HandleTypeDef *driver){
    if(driver->is_running) return;
    HAL_TIM_PWM_Start(driver->htim, driver->channel);
    driver->is_running = 1;
}

void PWM_Driver_Stop(PWM_Driver_HandleTypeDef *driver){
    HAL_TIM_PWM_Stop(driver->htim, driver->channel);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->channel, 0);

    driver->current_duty = 0.0f;
    driver->is_running = 0;
}

void PWM_Driver_SetDutyCycle(PWM_Driver_HandleTypeDef *driver, float duty_percent){
    if (duty_percent > 100.0f){
        duty_percent =100.0f;
    } else if (duty_percent < 0.0f){
        duty_percent = 0.0f;
    }
    
    driver->current_duty = duty_percent;
    uint32_t arr_val = __HAL_TIM_GET_AUTORELOAD(driver->htim)+1;
    uint32_t ccr_val = (uint32_t)((duty_percent/100.0f)*(float)arr_val);
    if (ccr_val >= arr_val){
        ccr_val = arr_val;
    }

    __HAL_TIM_SET_COMPARE(driver->htim, driver->channel, ccr_val);

}

float Calculate_Boost_Torque(float T_start, float T_target, float t_fast, float tau) {
    float delta_T = T_target - T_start;
    
    // Tính hệ số đáp ứng k = 1 - e^(-t/tau)
    float k = 1.0f - expf(-t_fast / tau);
    if (k < 0.01f) k = 0.01f; // Tránh chia cho 0
    
    float T_boost = T_start + (delta_T / k);
    
    // Khống chế không vượt quá giới hạn phần cứng phanh (0 - 200 Nm)
    if (T_boost > TJ_POD_MAX_TORQUE_NM) T_boost = TJ_POD_MAX_TORQUE_NM;
    if (T_boost < 0.0f) T_boost = 0.0f;
    
    return T_boost;
}