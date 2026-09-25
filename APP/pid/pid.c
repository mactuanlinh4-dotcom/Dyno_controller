#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max){
    pid->Kp=Kp;
    pid->Ki=Ki;
    pid->Kd=Kd;
    pid->dt=dt;
    pid->out_max=out_max;
    pid->out_min=out_min;
    PID_Reset(pid);
}

void PID_Reset(PID_HandleTypeDef *pid){
    pid->integrator=0.0f;
    pid->prev_error= 0.0f;
}

float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float measurement){
    float error = setpoint-measurement;

    float p_term = pid->Kp*error;
    pid->integrator +=pid->Ki*pid->dt*error;
    if(pid->integrator > pid->out_max){
        pid->integrator=pid->out_max;
    } else if (pid->integrator < pid->out_min){
        pid->integrator = pid->out_min;
    }

    float d_term = 0.0f;
    if(pid->dt > 0.0f){
        d_term=pid->Kd*(error-pid->prev_error)/pid->dt;
    }
    pid->prev_error=error;

    float output = p_term + pid->integrator + d_term;
    if(output > pid->out_max){
        output = pid->out_max;
    }else if(output < pid->out_min){
        output = pid->out_min;
    }

    return output;
}

