#include "momencontroller.h"

void MomenControllerInit(MOMEN_HandleTypeDef *dev){
    dev ->k_m = MOMEN_MAX/CURRENT_MAX;
    dev ->k_current = VOLTAGE_BRAKE/CURRENT_MAX;
}
