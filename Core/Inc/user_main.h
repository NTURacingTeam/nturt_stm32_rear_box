


#ifndef USER_MAIN_H_
#define USER_MAIN_H_
#include "main.h"
// remove the extra frontbox sensors transfer functions
#define REAR_BOX_SENSOR_ONLY

void user_main(void);
void CAN_error_handler(void);
HAL_StatusTypeDef I2C_start_error_handler();

#endif
