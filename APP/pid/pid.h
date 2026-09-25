#ifndef __PID_H
#define __PID_H

#include "stm32g4xx_hal.h"

typedef struct {
    float Kp;           
    float Ki;           
    float Kd;           
    float dt;           
    float out_min;      
    float out_max;      
    float integrator;   
    float prev_error;   
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void PID_Reset(PID_HandleTypeDef *pid);
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float measurement);

#endif 