/**
  ******************************************************************************
  * @file    user_main.c
  * @author  Tsou, Min-Lun from 2022 NTUR
  * @brief   custom user code that is separated from the automatically generated main.c
  *          This file should manage all the code that was initially put in main.c,
  *          including
  *           + user code in main() such as the super loop
  *           + interrupt callback functions
  *           + CAN filter initialization
  *
  @verbatim
  ==============================================================================
                    ##### Operation of the Code #####
  ==============================================================================
  [..]
  The mission of the rear box STM32 is to
  (#) Gather all the data from all the sensors
  (#) Send all the sensor data into the CAN bus
  (#) Control the brake light based on the signal from CAN

  [..]
  The sensors that connects to the front box include:
  (+) 2 suspension travel sensors, each connected to an ADC channel
  (+) 2 tire temperature sensors, connected to an I^2C master peripheral
  (+) 2 wheel speed sensors, each connected to a GPIO EXTI line
  The sensors occupies the ADC, I^2C, and EXTI peripherals. Each of the peripherals operate
  independently, so this part of the code written can be generally divided into 3 parts, each
  dedicated to one of them. The controlling code here is simply a subset of that in the front box.

  [..]
  The ADC operates as such:
  (#) ADC1 is initiated in CubeMX with an active DMA and 2 channels each connected to a sensor.
  (#) an array of 2 uint32_t is allocated for the DMA
  (#) HAL_ADC_Start_DMA() is called once in user_main(), then the DMA automatically puts the data
  	  into the array.
  (#) various transfer functions from analog_transfer_function.c are called in user_main() to
  	  convert the numbers back into the original values.
  (#) the converted values can simply be put in the CAN array.


  [..](CODE_UNFINISHED)
  The software for the hall sensors operate as such:
  (#) EXTI for falling edge detection is initiated in CubeMX for each sensor, along with timer3 in
  	  regular up-counting mode.
  (#) int hall_counter is allocated for each sensor, which increase by 1 by the software every
  	  time an EXTI ISR is called.
  (#) HAL_TIM_Base_Start_IT() is called once in user_main() to start the timer. Then, timer3
  	  overflow ISR collects the counted value in a fixed interval, puts the number in
  	  hall_counter_result, then resets the hall_counter.
  (#) we can work out the wheel speed by the timer interval before putting the value in the
  	  CAN array.

  [..]
  The rest of the sensors that operates through I^2C and are called in the super-loop. The MCU can
  simply grab the data by calling the library API.

  [..]
  Some calculation must be done in order to obtain the original data for the different sensors in
  order for it to fit the format in the CAN frame. After that is done, the MCU would put everything
  into 2 8-byte arrays, then sends it all through the CAN peripheral.
  The CAN HAL API is called as follows:
  (#) CAN is initiated in CubeMX in normal mode with 1Mb/s baud.
  (#) TxHeader struct is allocated, along with TxMailbox(unused but required for HAL API) and
  	  an 8 byte CAN_TxData array.
  (#) HAL_CAN_Start() is called once in user_main().
  (#) set up the information for arbitration fields with the TxHeader struct, then call
  	  HAL_CAN_AddTxMessage once CAN_TxData is ready.

  [..]
  The rear box listens to the CAN bus, and controls the brake light based on the BSE micro switch
  bit coming from the "Front Box_2" frame.
  (#) CAN is initiated in CubeMX in normal mode with 1Mb/s baud as shown above
  (#) HAL_CAN_RxFifo0MsgPendingCallback() is defined to interrupt and execute once a message is
  	  received
  (#) in user_main(),first a CAN filter is set up to only accept "Front Box_2" frame on the CAN bus
  (#) After the filter, HAL_CAN_ActivateNotification() is called once in to activate
  	  FIFO0_msg_pending interrupt callback, then the usual HAL_CAN_Start() is called
  (#) When the desired frame arrives, the callback would be called and the message is obtained by
  	  some HAL API. Then, the MCU can control the designated GPIO that switches the MOSFET for the
  	  brake light after reading the required bit in the frame.


  @endverbatim
  */

/*private include*/
#include "user_main.h"
#include "main.h"
#include <stdio.h>
#include "MLX.h"
#include "analog_transfer_function.h"

/* auto-generated peripheral handler structure by MX.
 * First defined in main.c
 * Must be updated to match the defined structures in main.c everytime peripheral
 * usage is changed.
 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#define ADC_DMA_ARRAY_RANK_LTRAVEL 0	/*IN3,PA3*/
#define ADC_DMA_ARRAY_RANK_RTRAVEL 1	/*IN9,PB1*/
/*ADC1 DMA destination array, the corresponding rank is defined above.*/
uint32_t ADC_value[2]={0};

/*CAN required custom variables*/
static CAN_TxHeaderTypeDef CAN_Tx_header_1 = {
	.IDE = CAN_ID_EXT,
	.ExtId = 0x080AD093,
	.RTR = CAN_RTR_DATA,
	.DLC = 8
};
static uint8_t CAN_Tx_data_1[8]={0};
static uint32_t Tx_mailbox_1;

static CAN_TxHeaderTypeDef CAN_Tx_header_2 = {
	.IDE = CAN_ID_EXT,
	.ExtId = 0x080AD093,
	.RTR = CAN_RTR_DATA,
	.DLC = 8
};
static uint8_t CAN_Tx_data_2[8]={0};
static uint32_t Tx_mailbox_2;

CAN_RxHeaderTypeDef CAN_Rx_header;
uint8_t CAN_RxData[8]={0};
/**
  * @brief  contains the part of the application instructions that is originally put in main(),
  * 		including the super loop.
  * 		Should be called once after all auto-gererated init functions in main()
  * @param  None
  * @retval None
  */
void user_main(void){
	/*Starting the ADC with DMA*/
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,2);

	/*setting up the CAN receive filter*/
	CAN_FilterTypeDef canfilterconfig = {
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 0,  // how many filters to assign to the CAN1 (master can)
			.FilterBank = 7,  // which filter bank to use from the assigned ones
			.FilterFIFOAssignment = CAN_FILTER_FIFO0,
			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_32BIT,
			/*only letting 0x080AD092(ExtID for FrontBox2 ) pass through: FilterHIGH contains first 16bit(right aligned);*/
			/*FilterLOW contains last 13 bits(positioned at bit 15:3); last 3 bit of filterLOW is neglected (IDE RTR 0)*/
			.FilterIdHigh = 0x080AD092>>13,
			.FilterIdLow = (0x080AD092<<3) & 0x0000FFFF,
			.FilterMaskIdHigh = 0xFFFF,
			.FilterMaskIdLow = 0xFFFF&(~0b111)
	};
	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig)!=HAL_OK){
		Error_Handler();
	}
	/*turning on receive interrupt mask, then start the CAN peripheral*/
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
		Error_Handler();
	}
	HAL_CAN_Start(&hcan);

	/*superloop*/
	for(;/*ever*/;){
		
		float temp=MLX90614_ReadReg(0x5A,0x08,0)*0.02-273.15;
		printf("%.2f\r\n",temp);
		//  printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x06,0)*0.02-273.15);
		// printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x07,0)*0.02-273.15);
		// printf("%.2f C \r\n",MLX90614_ReadReg(0x5A,0x08,0)*0.02-273.15);
		// printf("%x\r\n",MLX90614_ReadReg(0x5A,0x08,0));
		printf("%ld,%ld\r\n",ADC_value[0],ADC_value[1]);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
		//uint8_t A[3] ="aa";
		//HAL_UART_Transmit(&huart1,(uint8_t*)&A,2,0xFFFF);
		// HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);


		/*CAN sending message*/
		if(HAL_CAN_AddTxMessage(&hcan,&CAN_Tx_header_1,CAN_Tx_data_1,&Tx_mailbox_1)!=HAL_OK){
		  Error_Handler();
		}
		if(HAL_CAN_AddTxMessage(&hcan,&CAN_Tx_header_2,CAN_Tx_data_2,&Tx_mailbox_2)!=HAL_OK){
		  Error_Handler();
		}
		/*rough transmit interval*/
		HAL_Delay(20);
		
	}/*for(;;)*/
}

/**
  * @brief  User defined EXTI interrupt callback function, namely EXTI ISR.
  * 		Shall only be called by HAL interrupt handlers
  * @param  GPIO_PIN: the GPIO pin that generated the interrupt.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	if(GPIO_PIN==GPIO_PIN_5){
		printf("right wheelspeed\n");
	}
	else if(GPIO_PIN==GPIO_PIN_4){
		printf("left wheelspeed\n");
	}
	return;
}

/**
  * @brief  User defined CAN receiving interrupt callback function, namely CAN receiving ISR.
  * 		Shall only be called by HAL interrupt handlers
  * @param  hcan: the can handle structure that received the message.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	/*for rearbox, only 0x080AD092(front box 2) should pass through*/
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_header, CAN_RxData) != HAL_OK){
    Error_Handler();
  }

  /*checks bit1 on byte8 for brake pedal state, turns on the brake light if it is set*/
  if(CAN_RxData[7]&(1u<<1)){
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
  }
  else{
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
  }
  return;
}
