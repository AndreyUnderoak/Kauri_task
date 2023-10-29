#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "lis331dlh_reg.h"


void accel_init(I2C_HandleTypeDef*);
void accel_get_data(void);


#endif /* INC_ACCEL_H_ */
