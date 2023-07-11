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

#define FLAG_ADC1_FINISH 0x10
#define FLAG_ADC2_FINISH 0x20
#define FLAG_I2C1_FINISH 0x40
#define FLAG_I2C3_FINISH 0x80
#define FLAG_SPI4_FINISH 0x100
#define FLAG_3US_FINISH 0x200 
#define FLAG_READ_SUS_PEDAL 0x1000
#define FLAG_READ_TIRE_TEMP 0x2000
#define FLAG_READ_STEER 0x4000
#define FLAG_D6T_STARTUP 0x100000
#define FLAG_HALL_EDGE_RIGHT 0x20000 //TODO: makes sure the names are either hall or wheel speed but not both
#define FLAG_HALL_EDGE_LEFT 0x40000

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

BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten);
void bindHallMutex(SemaphoreHandle_t left, SemaphoreHandle_t right);

#endif //_SENSORS_H