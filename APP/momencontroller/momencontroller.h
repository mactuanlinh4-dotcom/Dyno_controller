#ifndef __BREAKMC_
#define __BREAKMC_

#define MOMEN_MAX     200
#define CURRENT_MAX   2.7
#define VOLTAGE_BRAKE 24

typedef struct{
    float k_m;
    float k_current;
    float current;
    uint16_t currrent_bit;
} MOMEN_HandleTypeDef;

void MomenControllerInit(MOMEN_HandleTypeDef *dev);
void MomenToCurrent(MOMEN_HandleTypeDef *dev, uint16_t momen);
void CurrentToARR(MOMEN_HandleTypeDef *dev);
void MomenControl(MOMEN_HandleTypeDef *dev);

#endif