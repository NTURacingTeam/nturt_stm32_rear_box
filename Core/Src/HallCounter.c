#include "main.h"
#include "cmsis_os2.h"

extern TIM_HandleTypeDef* const p_htim_hallSensorBase;

extern osEventFlagsId_t sensorEventGroupHandle;
extern osMessageQueueId_t hallLHandle;
extern osMessageQueueId_t hallRHandle;

uint32_t counterL = 0;
uint32_t counterR = 0;

void StartHallCounter(void *argument) {
    
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch(GPIO_Pin) {
        case R_HALL_Pin:
            counterR++;
            break;
        case L_HALL_Pin:
            counterL++;
            break;
        default:
            ; //TODO: handle exception
    }
}