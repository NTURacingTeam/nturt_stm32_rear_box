#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

 #define PRINTF_TEST

extern FDCAN_HandleTypeDef* p_hcan;
extern UART_HandleTypeDef* p_huart_testCOM;

extern osMessageQueueId_t adcLHandle;
extern osMessageQueueId_t adcRHandle;
extern osEventFlagsId_t sensorEventGroupHandle;

static const FDCAN_TxHeaderTypeDef CanHeader1 = {
  .Identifier = 0x080AD093,
  .IdType = FDCAN_EXTENDED_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_8,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_OFF,
  .FDFormat = FDCAN_CLASSIC_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0x00
};

static const FDCAN_TxHeaderTypeDef CanHeader2 = {
  .Identifier = 0x080AD094,
  .IdType = FDCAN_EXTENDED_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_8,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_OFF,
  .FDFormat = FDCAN_CLASSIC_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0x01
};

static uint32_t queueTimeout = 5U;

void StartCanProvider(void *argument) {
  /*data variable*/
  uint8_t payload1[8] = {0};
  uint8_t payload2[8] = {0};

  /*sensor reading variable*/
  uint32_t adcLreading = 0;
  uint32_t adcRreading = 0;

#ifdef PRINTF_TEST
  uint8_t buf[20] = {0};
#endif

  /*set up the reception filter*/
  {
    FDCAN_FilterTypeDef filterConfig = {
      .IdType = FDCAN_EXTENDED_ID,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterID1 = 0x080AD092,
      .FilterID2 = 0x1FFFFFFF,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterIndex = 0U
    };
    HAL_FDCAN_ConfigFilter(p_hcan, &filterConfig);
  }
  
  /*enable reception interrupt*/
  HAL_FDCAN_ActivateNotification(p_hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, NULL);

  /*enable CAN*/
  HAL_FDCAN_Start(p_hcan);

  for(;;)
  {
    // /*read data from queue*/
    // osMessageQueueGet(adcLHandle, &adcLreading, NULL, queueTimeout);
    // /*format the CAN frame*/
    // /*send the message*/
    // HAL_FDCAN_AddMessageToTxFifoQ(p_hcan, &CanHeader1, payload1);
    // HAL_FDCAN_AddMessageToTxFifoQ(p_hcan, &CanHeader2, payload2);
    
    /*start the sensor*/
    osEventFlagsSet(sensorEventGroupHandle, sensorStartEvent);

    /*wait for completion*/
    if(osEventFlagsWait(sensorEventGroupHandle, adcTaskCplt, osFlagsWaitAll, 5U) == osFlagsErrorTimeout) {
      ;
    }

    /*get finished values*/
    osMessageQueueGet(adcLHandle, &adcLreading, NULL, queueTimeout);
    osMessageQueueGet(adcRHandle, &adcRreading, NULL, queueTimeout);

#ifdef PRINTF_TEST
    /*test print*/
    sprintf(buf, "%ld, %ld\r\n", adcLreading, adcRreading);

    /*print it out*/
    HAL_UART_Transmit(p_huart_testCOM, buf, strlen(buf), HAL_MAX_DELAY);
#endif
    /*output to CAN*/

    osDelay(100);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  /*determine the origin of the message*/
  /*read data from RxMessage*/
  /*Toggle BrakeLight based on input*/
  HAL_GPIO_TogglePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin); //example
}