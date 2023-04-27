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
//#define USE_D6T

extern I2C_HandleTypeDef* const p_hi2c_tireTempR;
extern I2C_HandleTypeDef* const p_hi2c_tireTempL;
extern CRC_HandleTypeDef* const p_hcrc_i2c;

#ifdef USE_D6T //D6T-8L-09H
volatile uint8_t i2cStreamR[22] = {0};
volatile uint8_t i2cStreamL[22] = {0};
#else //MLX90614
volatile uint8_t i2cStreamR1[6] = {0};
volatile uint8_t i2cStreamR2[6] = {0};
volatile uint8_t i2cStreamL1[6] = {0};
volatile uint8_t i2cStreamL2[6] = {0};
#endif


extern osThreadId_t i2cReaderHandle;

static const uint32_t i2cRTxCpltFlag = 1U;
static const uint32_t i2cRRxCpltFlag = 1U << 1;
static const uint32_t i2cLTxCpltFlag = 1U << 2;
static const uint32_t i2cLRxCpltFlag = 1U << 3;

static const uint32_t i2cTxTimeout = 5U;
//For D6T, time for all the bits be read is 19*9*0.01 = 1.98 ms in 100k baud
static const uint32_t i2cRxTimeout = 4U;

#ifdef USE_D6T
static void init_D6T(I2C_HandleTypeDef* const, volatile uint8_t*, uint32_t);
#else
static void init_MLX(I2C_HandleTypeDef* const, volatile uint8_t*, uint32_t);
#endif

void StartI2cReader(void *argument) {
#ifdef USE_D6T
    //wait for 20ms
    osDelay(20);

    //initialize D6T sensors and the rawData array
    init_D6T(p_hi2c_tireTempR, i2cStreamR, i2cRTxCpltFlag);
    init_D6T(p_hi2c_tireTempL, i2cStreamL, i2cLTxCpltFlag);
    
    //wait for 500ms
    osDelay(500);
#else
    //initialize data buffers and check if the sensors are connected
    init_MLX(p_hi2c_tireTempR, i2cStreamR1, i2cRTxCpltFlag);
    init_MLX(p_hi2c_tireTempR, i2cStreamR2, i2cRTxCpltFlag);
    init_MLX(p_hi2c_tireTempR, i2cStreamL1, i2cLTxCpltFlag);
    init_MLX(p_hi2c_tireTempR, i2cStreamL2, i2cLTxCpltFlag);
#endif

    for(;;) {
        //read data
        //TODO: here HAL writes commands in Poll mode, then reads in DMA mode. can use 2 DMA with "seq" APIs if we want 
        //max performance
#ifdef USE_D6T
        HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempR, i2cStreamR[0], i2cStreamR[1], 1, &(i2cStreamR[3]), 19);
        HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempL, i2cStreamL[0], i2cStreamL[1], 1, &(i2cStreamL[3]), 19);
        osThreadFlagsWait(i2cRRxCpltFlag | i2cLRxCpltFlag, osFlagsWaitAll, i2cRxTimeout);
#else
        HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempR, i2cStreamR1[0], i2cStreamR1[1], 1, &(i2cStreamR1[3]),3);
        HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempL, i2cStreamL1[0], i2cStreamL1[1], 1, &(i2cStreamL1[3]),3);
        uint32_t firstCleared = osThreadFlagsWait(i2cLRxCpltFlag | i2cRRxCpltFlag, osFlagsWaitAny, i2cRxTimeout);
        
        switch(firstCleared) {
            case i2cRRxCpltFlag:
                HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempR, i2cStreamR2[0], i2cStreamR2[1], 1, &(i2cStreamR2[3]),3);
                osThreadFlagsWait(i2cLRxCpltFlag, osFlagsWaitAll, i2cRxTimeout);
                HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempL, i2cStreamL2[0], i2cStreamL2[1], 1, &(i2cStreamL2[3]),3);
                break;
            case i2cLRxCpltFlag:
                HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempL, i2cStreamL2[0], i2cStreamL2[1], 1, &(i2cStreamL2[3]),3);
                osThreadFlagsWait(i2cRRxCpltFlag, osFlagsWaitAll, i2cRxTimeout);
                HAL_I2C_Mem_Read_DMA(p_hi2c_tireTempR, i2cStreamR2[0], i2cStreamR2[1], 1, &(i2cStreamR2[3]),3);
                break;
            default:
        }
        osThreadFlagsWait(i2cLRxCpltFlag | i2cRRxCpltFlag, osFlagsWaitAny, i2cRxTimeout);
#endif
#ifdef USE_D6T
        //check CRC TODO: maybe function the whole thing?
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamR, 21) != i2cStreamR[21]) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            //TODO: PEC error handling
        }
        //check CRC
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamL, 21) != i2cStreamL[21]) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            //TODO: PEC error handling
        }
#else
        //error handling for MLX sensors
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamL1, 5) != i2cStreamL1[5]) {

        }
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamL2, 5) != i2cStreamL2[5]) {
            
        }
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamR1, 5) != i2cStreamR1[5]) {
            
        }
        if(HAL_CRC_Calculate(p_hcrc_i2c, i2cStreamR2, 5) != i2cStreamR2[5]) {
            
        }
#endif
        //push data on queue or mutex global

        //wait for at least 250ms
        osDelay(250);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == p_hi2c_tireTempL) {
        osThreadFlagsSet(i2cReaderHandle, i2cLTxCpltFlag);
    }
    else if (hi2c == p_hi2c_tireTempR) {
        osThreadFlagsSet(i2cReaderHandle, i2cRTxCpltFlag);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c == p_hi2c_tireTempR) {
        osThreadFlagsSet(i2cReaderHandle, i2cRRxCpltFlag);
    }
    else if (hi2c == p_hi2c_tireTempL) {
        osThreadFlagsSet(i2cReaderHandle, i2cLRxCpltFlag);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    //TODO: error handling
}

#ifdef USE_D6T
static void init_D6T(I2C_HandleTypeDef* const hi2c, volatile uint8_t* rawData, uint32_t txThreadFlag) {
    //fixed parameters of D6T sensors: address, I2C command to get data, and the startup transmissions
    const uint8_t D6Taddr = 0b0001010;
    const uint8_t getCommand = 0x4C;
    const uint8_t startupCommand[5][4] = {
        {0x02, 0x00, 0x01, 0xEE},
        {0x05, 0x90, 0x3A, 0xB8},
        {0x03, 0x00, 0x03, 0x8B},
        {0x03, 0x00, 0x07, 0x97},
        {0x92, 0x00, 0x00, 0xE9}
    };

    //fill the first 3 bytes of rawData with these data since they will be used in CRC
    rawData[0] = D6Taddr<<1;
    rawData[1] = getCommand;
    rawData[2] = (D6Taddr << 1) + 1;

    //init sensor 
    if(HAL_I2C_IsDeviceReady(hi2c, D6Taddr << 1, 5, 0xF) == HAL_OK) {
        for(int i = 0; i<5; i++) {
            HAL_I2C_Master_Transmit_DMA(hi2c, D6Taddr << 1, startupCommand[i], 4);
            osThreadFlagsWait(txThreadFlag, osFlagsWaitAny, i2cTxTimeout);
        }
    } else {
        //TODO: report error - cannot connect to sensor
    }
}
#else
static void init_MLX(I2C_HandleTypeDef* const hi2c, volatile uint8_t* stream, uint32_t txThreadFlag) {
    const uint8_t addrR1 = 0x2A;
    const uint8_t addrR2 = 0x3A;
    const uint8_t addrL1 = 0x4A;
    const uint8_t addrL2 = 0x5A;
    const uint8_t memAddr = 0x07;

    if(stream == &i2cStreamL1) {
        stream[0] = addrL1<<1;
    }
    else if (stream == &i2cStreamL2) {
        stream[0] = addrL2<<1;
    }
    else if (stream == &i2cStreamR1) {
        stream[0] = addrR1<<1;
    }
    else if (stream == &i2cStreamR2) {
        stream[0] = addrR2<<1;
    }

    stream[1] = memAddr;
    stream[2] = stream[0]+1;
    if(HAL_I2C_IsDeviceReady(hi2c, stream[0], 5, 0xF == HAL_OK)) {
        return;
    }
    else {
        //TODO: report error
    }

}
#endif