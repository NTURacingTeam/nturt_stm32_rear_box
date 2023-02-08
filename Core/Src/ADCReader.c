#include "main.h"
#include "cmsis_os.h"

/*peripheral handler structs*/
extern ADC_HandleTypeDef* p_hadc_susR;
extern ADC_HandleTypeDef* p_hadc_susL;

/*thread struct and queue structs*/
extern osThreadId_t adcReaderHandle;
extern osMessageQueueId_t adcLHandle;
extern osMessageQueueId_t adcRHandle;
extern osEventFlagsId_t sensorEventGroupHandle;

/*private constants for ADC*/
static const uint32_t ADC_timeout = 5U;
static const uint32_t queue_timeout = 5U;

/*flag for adc thread flag*/
static const uint32_t LConvCpltFlag = 1U << 1;
static const uint32_t RConvCpltFlag = 1U << 2;

void StartAdcReader(void *argument) {
    for(;;) {
        /*wait for others to call its execution*/
        osEventFlagsWait(sensorEventGroupHandle, sensorStartEvent, osFlagsWaitAll, osWaitForever);

        /*start the adc*/
        HAL_ADC_Start_IT(p_hadc_susL);
        HAL_ADC_Start_IT(p_hadc_susR);

        /*wait for conversion to complete*/
        osThreadFlagsWait(LConvCpltFlag | RConvCpltFlag, osFlagsWaitAll, ADC_timeout);

        /*read the data*/
        uint32_t lvalue = HAL_ADC_GetValue(p_hadc_susL);
        uint32_t rvalue = HAL_ADC_GetValue(p_hadc_susR);

        /*push data onto queue*/
        osMessageQueuePut(adcLHandle, &lvalue, 0, queue_timeout);
        osMessageQueuePut(adcRHandle, &rvalue, 0, queue_timeout);

        /*signal completion*/
        osEventFlagsSet(sensorEventGroupHandle, adcTaskCplt);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if(hadc == p_hadc_susL) {
        /*send notification to adc reader*/
        osThreadFlagsSet(adcReaderHandle, LConvCpltFlag);
    }
    else if(hadc == p_hadc_susR) {
        /*send notification to adc reader*/
        osThreadFlagsSet(adcReaderHandle, RConvCpltFlag);
    }
    
}