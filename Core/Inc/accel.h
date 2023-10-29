#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "lis331dlh_reg.h"

void lis331dlh_read_data_polling(I2C_HandleTypeDef*);


#endif /* INC_ACCEL_H_ */
