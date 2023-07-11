#ifndef __D6T_H
#define __D6T_H

#include "sensors.h"
#include "stm32g4xx_hal.h"

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

uint8_t init_D6T(I2C_HandleTypeDef* const hi2c, volatile i2c_d6t_dma_buffer_t* rawData, uint32_t txThreadFlag, uint32_t* otherflags, uint32_t timeout);

static inline HAL_StatusTypeDef start_D6T_read_DMA(I2C_HandleTypeDef* const hi2c, i2c_d6t_dma_buffer_t* rawData) {
    return HAL_I2C_Mem_Read_DMA(
                hi2c,
                rawData->addr_write, 
                rawData->command, 
                1, 
                &(rawData->PTAT.low), 
                sizeof(rawData->PTAT)+sizeof(rawData->temp)+sizeof(rawData->PEC));
}

void update_D6T(i2c_d6t_dma_buffer_t* rawData, float* payload);
#endif //_D6T_H