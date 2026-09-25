#ifndef __PWM_DRIVER_H
#define __PWM_DRIVER_H

#include "main.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_tim.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_duty;
    uint8_t is_running;
} PWM_Driver_HandleTypeDef;

void PWM_Driver_Init(PWM_Driver_HandleTypeDef *driver, TIM_HandleTypeDef *htim, uint32_t channel);
void PWM_Driver_Start(PWM_Driver_HandleTypeDef * driver);
void PWM_Driver_Stop(PWM_Driver_HandleTypeDef *driver);
void PWM_Driver_SetDutyCycle(PWM_Driver_HandleTypeDef *driver, float duty_percent);
float Calculate_Boost_Torque(float T_start, float T_target, float t_fast, float tau);
#endif