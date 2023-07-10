#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdbool.h>
#include <stdint.h>

#include "semphr.h"
#include "timers.h"

/*macro: freertos object parameter*/
#define SENSOR_DATA_TASK_STACK_SIZE 256

// freertos timer info
#define SENSOR_TIMER_PERIOD 5 //in ms
#define TIRE_TEMP_PERIOD 250 //in ms
#define IMPLAUSIBILTITY_PERIOD 100 // in ms

typedef struct {
  float left;
  float right;
  SemaphoreHandle_t mutex;
} travel_data_t;

extern travel_data_t travel_sensor;

typedef struct {
  float left[8];
  float right[8];
  SemaphoreHandle_t mutex;
} tire_temp_data_t;

extern tire_temp_data_t tire_temp_sensor;

typedef struct {
    float left;
    float right;
    SemaphoreHandle_t mutex;
} wheel_speed_data_t;

extern wheel_speed_data_t wheel_speed_sensor;

/*htim7 ISR*/
//external linkage because we need to call this in HAL_TIMPeriodElapsedCallback in main.c
void __hall_timer_elapsed(TIM_HandleTypeDef *htim);
/*sensor timer ISR*/
void sensor_timer_callback(void *argument);

//init the freertos objects
void sensor_init(void);

#endif //_SENSORS_H