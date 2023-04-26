/**
 * @file i2cReader.c
 * @author Tsou
 * @brief 
 * @version 0.1
 * @date 2023-02-28
 * 
 * @copyright Copyright (c) 2023
 * 
 * Note
 * https://omronfs.omron.com/en_US/ecb/products/pdf/en_D6T_users_manual.pdf
 * p17: sample sudo code for 09H
 * 
 * p13: sample startup sequence
 * 
 * p9: i2c flow and spec
 * 
 * jp ver.
 * https://components.omron.com/jp-ja/ds_related_pdf/CDSC-019.pdf
 * p31 8L-09H
 * p31: i2c connection procedure
 * 
 * Here the data is huge. might want to use dynamic memory with pointers on queues.
 */

#include <main.h>
#include <cmsis_os2.h>

/*define the sensor type as D6T-8L-09H. otherwise MLX90614 from Melexis is used*/
#define USE_D6T

extern I2C_HandleTypeDef* const p_hi2c_tireTempR;
extern CRC_HandleTypeDef* const p_hcrc_i2c;

#ifdef USE_D6T
static const tempSensorAddr = 0b0001010;
static const uint8_t getCommand = 0x4C;
static const uint8_t startupCommand[5][4] = {
    {0x02, 0x00, 0x01, 0xEE},
    {0x05, 0x90, 0x3A, 0xB8},
    {0x03, 0x00, 0x03, 0x8B},
    {0x03, 0x00, 0x07, 0x97},
    {0x92, 0x00, 0x00, 0xE9}
};
volatile uint8_t rawData[22] = {0};
#else
//TODO: MLX90614 variables
#endif


extern osThreadId_t i2cReaderHandle;
static const uint32_t i2cRTxCpltFlag = 1U;
static const uint32_t i2cRRxCpltFlag = 1U << 1;
static const uint32_t i2cTxTimeout = 5U;
// time for all the bits be read is 19*9*0.01 = 1.98 ms for 100k baud
static const uint32_t i2cRxTimeout = 4U;

static uint8_t CRC8_Calc(uint8_t* rawData, uint8_t size);

void StartI2cReader(void *argument) {
    rawData[0] = tempSensorAddr<<1;
    rawData[1] = getCommand;
    rawData[2] = (tempSensorAddr << 1) + 1;
    volatile uint8_t* payload = &(rawData[3]);

    //wait for 20ms
    osDelay(20);

    //init sensor
    if(HAL_I2C_IsDeviceReady(p_hi2c_tireTempR, tempSensorAddr << 1, 5, 0xF) == HAL_OK) {
        for(int i = 0; i<5; i++) {
            HAL_I2C_Master_Transmit_DMA(p_hi2c_tireTempR, tempSensorAddr << 1, startupCommand[i], 4);
            osThreadFlagsWait(i2cRTxCpltFlag, osFlagsWaitAny, i2cTxTimeout);
        }
    } else {
        //TODO: report error - cannot connect to sensor
    }
    
    //wait for 500ms
    osDelay(500);

    for(;;) {
        //read data
        //TODO: here HAL writes commands in Poll mode, then reads in DMA mode. can use 2 DMA with "seq" APIs if we want 
        //max performance
        HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempR, tempSensorAddr << 1, getCommand, 1, payload, 19);
        osThreadFlagsWait(i2cRRxCpltFlag, osFlagsWaitAny, i2cRxTimeout);

        //check CRC
        if(HAL_CRC_Calculate(p_hcrc_i2c, rawData, 21) != rawData[21]) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            //TODO: PEC error handling
        }

        //push data on queue or mutex global

        //wait for at least 250ms
        osDelay(250);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    osThreadFlagsSet(i2cReaderHandle, i2cRTxCpltFlag);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    osThreadFlagsSet(i2cReaderHandle, i2cRRxCpltFlag);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    //TODO: error handling
}

static uint8_t CRC8_Check(uint8_t* rawData, uint8_t size) {
    //TODO: check CRC
    HAL_CRC_Calculate(p_hcrc_i2c, rawData, size);
    return 0;
}
