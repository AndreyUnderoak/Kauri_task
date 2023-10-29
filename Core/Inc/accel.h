#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "stm32f1xx_hal.h"
#include "lis331dlh_reg.h"


void accel_init(I2C_HandleTypeDef*);
int accel_get_data(float*);


#endif /* INC_ACCEL_H_ */
