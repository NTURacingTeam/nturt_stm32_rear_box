#include "main.h"
#include "cmsis_os2.h"

extern osEventFlagsId_t sensorEventGroupHandle;
extern osMessageQueueId_t hallLHandle;
extern osMessageQueueId_t hallRHandle;

extern osSemaphoreId_t hallCounterBinSemHandle;
static volatile uint32_t counterL = 0;
static volatile uint32_t counterR = 0;

extern osMutexId_t hallStoreMutexHandle;
static volatile uint32_t countL = 0;
static volatile uint32_t countR = 0;

static inline void counterTimeoutHandler(void);
static inline void countTimeoutHandler(void);
static inline uint16_t wheel_speed_transfer_function(uint32_t);

static const uint32_t mutexTimeout = 2U;

void StartHallConverter(void *argument) {
    uint32_t bufL = 0;
    uint32_t bufR = 0;
    uint16_t speedL = 0;
    uint16_t speedR = 0;
    for(;;) {

        //wait for another thread to send signal here
        osEventFlagsWait(sensorEventGroupHandle, sensorStartEvent, osFlagsWaitAll, osWaitForever);

        //enter critical section
        if(osMutexAcquire(hallStoreMutexHandle, mutexTimeout) == osOK) {

            //buffer the counted values
            bufL = countL;
            bufR = countR;

            //leave critical section
            osMutexRelease(hallStoreMutexHandle);

            //throw the value into the transfer functions
            speedL = wheel_speed_transfer_function(bufL);
            speedR = wheel_speed_transfer_function(bufR);

            //send the values into queues
            osMessageQueuePut(hallLHandle, &speedL, 0, 0); //TODO
            osMessageQueuePut(hallRHandle, &speedR, 0, 0); //need timeout exception handling 
        }
        else {//timeout on Mutex

            //use old value instead
            osMessageQueuePut(hallLHandle, &speedL, 0, 0); 
            osMessageQueuePut(hallRHandle, &speedR, 0, 0); 
            //probably needs error reporting
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    //enter critical sections
    //ignore current signal if the semaphore is taken
    if(osSemaphoreAcquire(hallCounterBinSemHandle, 0) == osOK) {

        //increase the counter value by 1 based on the GPIO that got the edge signal
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

        //leave critical section
        osSemaphoreRelease(hallCounterBinSemHandle);
    }
}

void StarthallCounterStorer(void *argument) {
    for(;;) {

        /*wait for ISR signalling*/
        osThreadFlagsWait(timerLapEvent, osFlagsWaitAll, osWaitForever);
        
        /*enter critical section*/
        if(osMutexAcquire(hallStoreMutexHandle, mutexTimeout) == osOK) {
            if(osSemaphoreAcquire(hallCounterBinSemHandle, mutexTimeout) == osOK) {

                //if successively get both resources
                countL = counterL;
                countR = counterR;

                /*leaves count critical section*/
                osMutexRelease(hallStoreMutexHandle);

                /*resets the counter*/
                counterL = 0;
                counterR = 0;

                /*leaves counter critical section*/
                osSemaphoreRelease(hallCounterBinSemHandle);

            }
            else {
                //timeout on the counter variable
                osMutexRelease(hallStoreMutexHandle);
                counterTimeoutHandler();
            }
        }
        else { //timeout on the timed value, probably should throw a error notification on failed attempt to update
            if(osSemaphoreAcquire(hallCounterBinSemHandle, mutexTimeout) == osOK) {
                //tries to reset the counter
                countL = 0;
                countR = 0;
                osSemaphoreRelease(hallCounterBinSemHandle);
                countTimeoutHandler();
            }
            else {
                countTimeoutHandler();
                counterTimeoutHandler();
            }
        }
    }
}

static inline void countTimeoutHandler(void) {
    //TODO
    //timeout on the counter variable that should be written by the EXTI ISR
    //probably should throw a error notification
}

static inline void counterTimeoutHandler(void) {
    //TODO
    //timeout on the count variable that should be written by the timer ISR
    //probably should throw a error notification
}

/**
  * @brief  transfer function for the hall tachometer on ep4
  * @param  reading: the number of hall trigger per 10ms
  * @retval the wheel speed in rad/s, times 256
  * @note the result is multiplied by 256 so that the MSB represents the integer part of the number while LSB represents the part less than 1
  */
static inline uint16_t wheel_speed_transfer_function(uint32_t reading){
	const float pi = 3.1415927;
	float input = reading;
	const float tooth_per_rev = 14.0;
	float value = 0.0;
	value = input *HALL_FREQ /tooth_per_rev *pi *256; //TODO update 100 with real timer values
	return (uint16_t)value;

}
