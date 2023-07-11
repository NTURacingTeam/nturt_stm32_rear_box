#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "sensors.h"

// #define PRINTF_TEST
#ifdef PRINTF_TEST
#define BUFSIZE 20

extern UART_HandleTypeDef* p_huart_testCOM;
#endif

#define MUTEX_TIMEOUT 0x02

static const FDCAN_TxHeaderTypeDef CanHeader1 = {
  .Identifier = 0x080AD094,
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
  .Identifier = 0x080AD095,
  .IdType = FDCAN_EXTENDED_ID,
  .TxFrameType = FDCAN_DATA_FRAME,
  .DataLength = FDCAN_DLC_BYTES_8,
  .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
  .BitRateSwitch = FDCAN_BRS_OFF,
  .FDFormat = FDCAN_CLASSIC_CAN,
  .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
  .MessageMarker = 0x01
};

void StartCanProvider(void *argument) {
  /*data variable*/
  uint8_t payload1[8] = {0};
  uint8_t payload2[8] = {0};

#ifdef PRINTF_TEST
  char buf[BUFSIZE] = {0};
#endif

  /*set up the CAN*/
  {
    //decides how does can react to unmatched frames
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    //decides how does CAN react to full FIFO
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    //configs the single filter we need for brake light
    FDCAN_FilterTypeDef filterConfig = {
      .IdType = FDCAN_EXTENDED_ID,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterID1 = 0x080AD091,
      .FilterID2 = 0x1FFFFFFF,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterIndex = 0U
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig);
  
    /*enable reception interrupt*/
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  }

  /*enable CAN*/
  HAL_FDCAN_Start(&hfdcan1);

  for(;;)
  {


#ifdef PRINTF_TEST
    /*test print*/
    int length = sprintf(buf, "%ld, %ld, ", adcLreading, adcRreading);

    /*print it out*/
    if(length != -1) {
      HAL_UART_Transmit(p_huart_testCOM, buf, length + 1, HAL_MAX_DELAY);
    }

    length = sprintf(buf, "%d, %d\r\n", hallLreading, hallRreading);
    if(length != -1) {
	  HAL_UART_Transmit(p_huart_testCOM, buf, length + 1, HAL_MAX_DELAY);
	}
#endif
    

    /*output to CAN*/
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanHeader1, payload1);
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CanHeader2, payload2);

    osDelay(10);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  /*read data from RxMessage*/
  if(RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    uint8_t payload[8];
    FDCAN_RxHeaderTypeDef RxHeader;
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, payload);
    /*Toggle BrakeLight based on input*/
    if(payload[5] & 0b1) {
      HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_RESET);
		}
  }
}
