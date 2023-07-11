#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"
#include "sensors.h"
#include "D6T.h"
#include "transfer_functions.h"

/**
 * @brief this function initializes the payload data of the i2c addresses and the D6T sensors themselves
 * 
 * @param hi2c the i2c handle that handles the i2c communications
 * @param rawData and array that stores the data to be transmitted and that to be received
 * @param txThreadFlag the task notification flag used to indicate completion of transfer
 * @param otherflags the other flags that might be caught when are waiting for the the DMA to complete through FreeRTOS notifications
 */
uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags, uint32_t timeout) {
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
            if(wait_for_notif_flags(txThreadFlag, timeout, otherflags) != pdTRUE) {
                return 1;
            }
        }
    } else {
        return 1;
    }
    return 0;
}

void update_D6T(i2c_d6t_dma_buffer_t* rawData, float* payload) {
    for(int i=0; i<sizeof(rawData->temp)/sizeof(rawData->temp[0]); i++) {
        payload[i] = tire_temp_transfer_function(rawData->temp[i].high, rawData->temp[i].low);
    }
}
