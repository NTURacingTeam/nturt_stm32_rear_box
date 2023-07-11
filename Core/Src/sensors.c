/**
 * @file sensors.c
 * @author MinLun Tsou (astatine1184@gmail.com)
 * @brief the controller function for the 2 pedals, APPS and BSE and every other sensors
 * @version 0.2
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 * TODO: update the explanation for other sensors
 * There are 3 foot pedal sensors: 2 APPS and 1 BSE
 * The relative rules for APPS and BSE are in T.4
 * 
 * analog sensors are used for the APPS, and the numbers are read by an internal ADC.
 * The numbers are linearly transformed into 0~1 with a positive slope and offset.
 * The normalized numbers are then passed to the other tasks.
 * 
 * There are a few safety checks done by this tasks before normalizing the readings
 * If the sensor reading go out of bounds, error is reported
 * If the 2 APPS deviate for more than 10 %, error is reported
 * 
 * This function is executed periodically, and the process is:
 * 1. calls the ADC and DMA to read the numbers automatically
 * 2. check if the numbers are out of bounds
 * 3. (optional) DSP filter the readings
 * 4. normalize the reading to 0 and 1
 * 5. push the new numbers onto a shared resource
 */

// glibc include
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
// #include "task.h"
#include "semphr.h"
#include "timers.h"

// project include
#include "transfer_functions.h"

//own include
#include "sensors.h"

#define USE_HALL_SENSOR
#define USE_D6T

#define MUTEX_TIMEOUT 0x02
#define ADC_TIMEOUT 0x02
#define ADC_RETRY_TIMEOUT 0x02
#define I2C_TIMEOUT 0xFF
#define SPI_TIMEOUT 0x02

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

/**
 * @brief structure to hold the set of data that is present in the transmission of i2c data with D6T sensors
 * @note see https://omronfs.omron.com/en_US/ecb/products/pdf/en_D6T_users_manual.pdf
 * @note the position of the members is important in memory. pls don't move them around.
 * 
 */
typedef struct {
    uint8_t addr_write;     //< address of D6T sensor that is left shifted once. Also the starting address of CRC calculation.
    uint8_t command;        //< command that is senst to the D6T sensor
    uint8_t addr_read;      //< address of D6T sensor that is left shited once then added 1. sent after Restarted signal.
    struct {
        uint8_t low;        //< the low byte of the sensor comes first before the high byte in the 16bit data.
        uint8_t high;
    }PTAT;                  //< the temperature of the sensor itself. Also the starting address of the Rx data buffer.        
    struct {
        uint8_t low;
        uint8_t high;    
    } temp[8];              //< the temperature of the 8 different measurement channels. low byte is sent first.
    uint8_t PEC;            //< packet error check code. Is to be compared with the CRC-8 result of previous 21 bytes.
} i2c_d6t_dma_buffer_t;

typedef struct {
    uint32_t elapsed_count;
    uint32_t timer_count;
    SemaphoreHandle_t mutex;
} timer_time_t;

static volatile timer_time_t hall_time_L = {0};
static volatile timer_time_t hall_time_R = {0};
static volatile uint32_t hall_timer_elapsed = 0;

travel_data_t travel_sensor = {
    .left = 0.0,
    .right = 0.0,
    //mutex is initialized in user_main.c along with everything freertos
};

tire_temp_data_t tire_temp_sensor = {
    .left = {0.0},
    .right = {0.0}
    //mutex is initialized in user_main.c along with everything freertos
};

wheel_speed_data_t wheel_speed_sensor = {
    .left = 0.0,
    .right = 0.0
    //mutex is initialized in user_main.c along with everything freertos 
};

static BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten);
static void update_time_stamp(timer_time_t* last, volatile const timer_time_t* now, timer_time_t* diff);
#ifdef USE_D6T
static uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags);
#endif


void sensor_timer_callback(void *argument) {
    static uint32_t expire_count = 0;

    xTaskNotify(sensorHandle, FLAG_READ_SUS_PEDAL, eSetBits);
    expire_count++;

    if(expire_count >= TIRE_TEMP_PERIOD/SENSOR_TIMER_PERIOD) {
        xTaskNotify(sensorHandle, FLAG_READ_TIRE_TEMP, eSetBits);
        expire_count = 0;
    }
}

/**
 * @brief handler function for the data acquisition task of the pedal sensors
 * 
 */
void StartSensorTask(void* argument) {
    //dma buffer zone
    static volatile i2c_d6t_dma_buffer_t d6t_dma_buffer_R = {0};
    static volatile i2c_d6t_dma_buffer_t d6t_dma_buffer_L = {0};

    timer_time_t hall_time_L_last = {0};
    timer_time_t hall_time_R_last = {0};

    //TODO: handle every return status of FreeRTOS and HAL API
    (void)argument;
    //variable to store the pending flags that is sent from xTaskNotify
    uint32_t pending_notifications = 0U;

    /*initialize D6T sensors*/
    //first wait for 20ms for the D6T sensors to boot up
#ifdef USE_D6T
    vTaskDelay(pdMS_TO_TICKS(20));

   if(init_D6T(&hi2c3, &d6t_dma_buffer_R, FLAG_D6T_STARTUP, &pending_notifications)) {
       Error_Handler();
   }
   if(init_D6T(&hi2c1, &d6t_dma_buffer_L, FLAG_D6T_STARTUP, &pending_notifications)) {
       Error_Handler();
   }
    
    //wait for 500ms after initialization of D6T
    vTaskDelay(pdMS_TO_TICKS(500));
#endif

    /*initialize the pedal sensors' compensation*/
    //calib the adc
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    //start the hall timer
#ifdef USE_HALL_SENSOR
    HAL_TIM_Base_Start(&htim6);
#endif

    osTimerStart(sensorTimerHandle, pdMS_TO_TICKS(5));

    while(1) {
        if(!pending_notifications) {
            //wait for notifications from timer if there are no pending ones
            (void)xTaskNotifyWait(0, 0xFFFFFFFFUL, &pending_notifications, portMAX_DELAY);
        }

        if(pending_notifications & FLAG_READ_SUS_PEDAL) {
            pending_notifications &= ~FLAG_READ_SUS_PEDAL; //clear bit

            HAL_ADC_Start_IT(&hadc1);
            HAL_ADC_Start_IT(&hadc2);

            wait_for_notif_flags(FLAG_ADC2_FINISH | FLAG_ADC1_FINISH, ADC_TIMEOUT, &pending_notifications);

            //update the travel sensor's value
            xSemaphoreTake(travel_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                travel_sensor.left = travel_transfer_function(HAL_ADC_GetValue(&hadc1));
                travel_sensor.right = travel_transfer_function(HAL_ADC_GetValue(&hadc2));
            xSemaphoreGive(travel_sensor.mutex);
        
        }
        if(pending_notifications & FLAG_HALL_EDGE_LEFT) {
            xSemaphoreTake(hall_time_L.mutex, MUTEX_TIMEOUT);
                pending_notifications &= ~FLAG_HALL_EDGE_LEFT; //clear flags

                timer_time_t diff = {0};
                update_time_stamp(&hall_time_L_last, &hall_time_L, &diff);

                xSemaphoreTake(wheel_speed_sensor.mutex, MUTEX_TIMEOUT); 
                    wheel_speed_sensor.left = wheel_speed_tranfser_function(diff.elapsed_count, diff.timer_count);
                xSemaphoreGive(wheel_speed_sensor.mutex);
            xSemaphoreGive(hall_time_L.mutex);
        }
        if(pending_notifications & FLAG_HALL_EDGE_RIGHT) {
            xSemaphoreTake(hall_time_R.mutex, MUTEX_TIMEOUT);
                pending_notifications &= ~FLAG_HALL_EDGE_RIGHT; //clear flags

                timer_time_t diff = {0};
                update_time_stamp(&hall_time_R_last, &hall_time_R, &diff);

                xSemaphoreTake(wheel_speed_sensor.mutex, MUTEX_TIMEOUT);
                    wheel_speed_sensor.right = wheel_speed_tranfser_function(diff.elapsed_count, diff.timer_count);
                xSemaphoreGive(wheel_speed_sensor.mutex);
            xSemaphoreGive(hall_time_R.mutex);
        }
        if(pending_notifications & FLAG_READ_TIRE_TEMP) {
            pending_notifications &= ~FLAG_READ_TIRE_TEMP;
#ifdef USE_D6T
            //read the values from both sensors
            HAL_I2C_Mem_Read_DMA(
                &hi2c3,
                d6t_dma_buffer_R.addr_write, 
                d6t_dma_buffer_R.command, 
                1, 
                &(d6t_dma_buffer_R.PTAT.low), 
                sizeof(d6t_dma_buffer_R.PTAT)+sizeof(d6t_dma_buffer_R.temp)+sizeof(d6t_dma_buffer_R.PEC));
            HAL_I2C_Mem_Read_DMA(
                &hi2c1, 
                d6t_dma_buffer_L.addr_write, 
                d6t_dma_buffer_L.command, 
                1, 
                &(d6t_dma_buffer_L.PTAT.low), 
                sizeof(d6t_dma_buffer_L.PTAT)+sizeof(d6t_dma_buffer_L.temp)+sizeof(d6t_dma_buffer_L.PEC));
            // wait for the DMA to finish, while we can do other stuff in the mean time
            //TODO: setup timeout exception and deal with error case where the stuff did not finish
#endif
        }
        if(pending_notifications & FLAG_I2C1_FINISH) {
            pending_notifications &= ~FLAG_I2C1_FINISH; //clear flags

            //TODO: CRC the data

            xSemaphoreTake(tire_temp_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                for(int i=0; i<sizeof(d6t_dma_buffer_L.temp)/sizeof(d6t_dma_buffer_L.temp[0]); i++) {
                    tire_temp_sensor.left[i] = tire_temp_transfer_function(d6t_dma_buffer_L.temp[i].high, d6t_dma_buffer_L.temp[i].low);
                }
            xSemaphoreGive(tire_temp_sensor.mutex);
        }
        if(pending_notifications & FLAG_I2C3_FINISH) {
            pending_notifications &= ~FLAG_I2C3_FINISH; //clear flags

            //TODO: CRC the data

            xSemaphoreTake(tire_temp_sensor.mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT));
                for(int i=0; i<sizeof(d6t_dma_buffer_R.temp)/sizeof(d6t_dma_buffer_R.temp[0]); i++) {
                    tire_temp_sensor.right[i] = tire_temp_transfer_function(d6t_dma_buffer_R.temp[i].high, d6t_dma_buffer_R.temp[i].low);
                }
            xSemaphoreGive(tire_temp_sensor.mutex);   
        }
    }
}

/**
 * @brief wrapper function to wait for specific bits in the task notification
 * @param target the target that needs to be waited 
 * @param timeout the timeout in ticks
 * @param gotten all the flags that are set. target bits would be autocleared if they are gotten
 * @return the status of the function
 */
BaseType_t wait_for_notif_flags(uint32_t target, uint32_t timeout, uint32_t* const gotten) {    
    uint32_t flag_buf = 0U;
    uint32_t flag_gotten = *gotten;
    const TickType_t t0 = xTaskGetTickCount();
    TickType_t tlast = t0;
    
    //if either of which is not set
    while(~flag_gotten & target) {
        BaseType_t Wait_result = xTaskNotifyWait(0, 0xFFFFFFFFUL, &flag_buf, timeout-(tlast-t0));
        if(Wait_result == pdFALSE) {
            return pdFALSE; 
        }

        flag_gotten |= flag_buf;
        *gotten = flag_gotten;
        tlast = xTaskGetTickCount();
    } 

    //remove the gotten flags if the flags are correctly received
    *gotten &= ~(target);
    return pdTRUE;
}

/**
 * @brief this function initializes the payload data of the i2c addresses and the D6T sensors themselves
 * 
 * @param hi2c the i2c handle that handles the i2c communications
 * @param rawData and array that stores the data to be transmitted and that to be received
 * @param txThreadFlag the task notification flag used to indicate completion of transfer
 * @param otherflags the other flags that might be caught when are waiting for the the DMA to complete through FreeRTOS notifications
 */
#ifdef USE_D6T
static uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags) {
    //TODO: use the return value to report error
    //fixed parameters of D6T sensors: address, I2C command to get data, and the startup transmissions
    const uint8_t D6Taddr = 0b0001010;
    const uint8_t getCommand = 0x4C;
    const uint8_t startupCommand[5][4] = {
        {0x02, 0x00, 0x01, 0xEE},
        {0x05, 0x90, 0x3A, 0xB8},
        {0x03, 0x00, 0x03, 0x8B},
        {0x03, 0x00, 0x07, 0x97},
        {0x02, 0x00, 0x00, 0xE9}
    };

    //fill the first 3 bytes of rawData with these data since they will be used in CRC
    rawData->addr_write = D6Taddr<<1;
    rawData->command = getCommand;
    rawData->addr_read = (D6Taddr << 1) + 1;

    //init sensor 
    if(HAL_I2C_IsDeviceReady(hi2c, D6Taddr << 1, 5, 0xF) == HAL_OK) {
        for(int i = 0; i<5; i++) {
            HAL_I2C_Master_Transmit_DMA(hi2c, D6Taddr << 1, startupCommand[i], 4);
            if(wait_for_notif_flags(txThreadFlag, I2C_TIMEOUT, otherflags) != pdTRUE) {
                return 1;
            }
        }
    } else {
        return 1;
    }
    return 0;
}
#endif

/**
 * @brief this function calculates the difference between last and now, and updates the "now" time to "last"
 * 
 * @param last previous time this function is called
 * @param now the new time stamp at which this time is called
 * @param diff calculates the time difference between last and now
 */
void update_time_stamp(timer_time_t* last, volatile const timer_time_t* now, timer_time_t* diff) {
    diff->elapsed_count = now->elapsed_count - last->elapsed_count;
    diff->timer_count = now->timer_count - last->timer_count;

    if(now->timer_count < last->timer_count) {
        diff->elapsed_count -= 1;
    }
    last->elapsed_count = now->elapsed_count;
    last->timer_count = now->timer_count;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    xTaskNotifyFromISR(sensorHandle, FLAG_D6T_STARTUP, eSetBits, NULL);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == &hi2c1) {
        xTaskNotifyFromISR(sensorHandle, FLAG_I2C1_FINISH, eSetBits, NULL);
    }
    else if (hi2c == &hi2c3) {
        xTaskNotifyFromISR(sensorHandle, FLAG_I2C3_FINISH, eSetBits, NULL);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc == &hadc1) {
        xTaskNotifyFromISR(sensorHandle, FLAG_ADC1_FINISH, eSetBits, NULL);
    } else if(hadc == &hadc2) {
        xTaskNotifyFromISR(sensorHandle, FLAG_ADC2_FINISH, eSetBits, NULL);
    }
}

#ifdef USE_HALL_SENSOR
void __hall_timer_elapsed(TIM_HandleTypeDef *htim) {
    (void)htim;
    hall_timer_elapsed++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {    
    if(GPIO_Pin == L_HALL_Pin) {
        if(xSemaphoreTakeFromISR(hall_time_L.mutex, NULL) == pdTRUE) {
            hall_time_L.timer_count = __HAL_TIM_GET_COUNTER(&htim6);
            hall_time_L.elapsed_count = hall_timer_elapsed;
            xTaskNotifyFromISR(sensorHandle, FLAG_HALL_EDGE_LEFT, eSetBits, NULL);
            xSemaphoreGiveFromISR(hall_time_L.mutex, NULL);
        }
    }   
    if(GPIO_Pin == R_HALL_Pin) {
        if(xSemaphoreTakeFromISR(hall_time_R.mutex, NULL) == pdTRUE) {
            hall_time_R.timer_count = __HAL_TIM_GET_COUNTER(&htim6);
            hall_time_R.elapsed_count = hall_timer_elapsed;
            xTaskNotifyFromISR(sensorHandle, FLAG_HALL_EDGE_RIGHT, eSetBits, NULL);
            xSemaphoreGiveFromISR(hall_time_R.mutex, NULL);
        }
    }
}
#endif
