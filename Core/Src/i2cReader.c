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
 */

#include <main.h>
#include <cmsis_os2.h>

extern I2C_HandleTypeDef* const p_hi2c_tireTempR;

static const tempSensorAddr = 0x14;

static const uint8_t startupCommand[5][4] = {
    {0x02, 0x00, 0x01, 0xEE},
    {0x05, 0x90, 0x3A, 0xB8},
    {0x03, 0x00, 0x03, 0x8B},
    {0x03, 0x00, 0x07, 0x97},
    {0x92, 0x00, 0x00, 0xE9}
};

void StartI2cReader(void *argument) {
    HAL_I2C_Master_Transmit_DMA(p_hi2c_tireTempR, tempSensorAddr, startupCommand[0], 4);
    for(;;) {
        ;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {

}