#include "rear_box_can.h"

// glibc
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// can_config include
#include "canmonitorutil.h"
#include "nturt_can_config.h"
#include "nturt_can_config_rear_sensor-binutil.h"

// project include
#include "project_def.h"
#include "sensors.h"
#include "user_main.h"

/* Private macro -------------------------------------------------------------*/
/* can tx checking macro -----------------------------------------------------*/
#define CHECK_TRANSMISSION(RET)                                               \
  do {                                                                        \
    if ((RET) != HAL_OK) {                                                    \
      ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_TX, ERROR_SET); \
      rear_box_can.tx_error_ = true;                                          \
    }                                                                         \
  } while (0)

/* can frame index for rx receive timeout error ------------------------------*/
#define NUM_NOT_DEFINED_FRAME 1
#define NUM_CRITICAL_FRAME 1
#define NUM_OPTIONAL_FRAME 1
#define NUM_FRAME (NUM_CRITICAL_FRAME + NUM_OPTIONAL_FRAME)

// not defined frame
#define FRAME_NOT_DEFINED 0UL

// critical frame
#define FRAME_CRITICAL_BASE (FRAME_NOT_DEFINED + NUM_NOT_DEFINED_FRAME)
#define FRAME_CRITICAL(X) (1UL << (FRAME_CRITICAL_BASE + X - 1UL))

#define FRONT_SENSOR_1_INDEX FRAME_CRITICAL(0)

#define FRAME_CRITICAL_MASK \
  ((1UL << (FRAME_CRITICAL_BASE + NUM_CRITICAL_FRAME - 1UL)) - 1UL)

// optional frame
#define FRAME_OPTIONAL_BASE (FRAME_CRITICAL_BASE + NUM_CRITICAL_FRAME)
#define FRAME_OPTIONAL(X) (1UL << (FRAME_OPTIONAL_BASE + X - 1UL))

// no optional frame

#define FRAME_OPTIONAL_MASK                                            \
  (((1UL << (FRAME_OPTIONAL_BASE + NUM_OPTIONAL_FRAME - 1UL)) - 1UL) - \
   FRAME_CRITICAL_MASK)

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
nturt_can_config_rear_sensor_tx_t can_rear_sensor_tx;
nturt_can_config_rear_sensor_rx_t can_rear_sensor_rx;

// mutex
SemaphoreHandle_t can_rear_sensor_tx_mutex;
SemaphoreHandle_t can_rear_sensor_rx_mutex;

/* Static variable -----------------------------------------------------------*/
// mutex control block
static StaticSemaphore_t can_rear_sensor_tx_mutex_cb;
static StaticSemaphore_t can_rear_sensor_rx_mutex_cb;

/* Static function prototype -------------------------------------------------*/
/**
 * @brief Get the index of a frame from its can id.
 *
 * @param id The can id of the frame.
 * @return int The index of the frame. 0 if the frame is not defined.
 */
uint32_t frame_id_to_index(uint32_t id);

/* virtual function redirection ----------------------------------------------*/
ModuleRet RearBoxCan_start(RearBoxCan* const self) {
  return self->super_.super_.vptr_->start(&self->super_.super_);
}

void RearBoxCan_configure(RearBoxCan* const self) {
  self->super_.vptr_->configure(&self->super_);
}

void RearBoxCan_receive(RearBoxCan* const self, const bool is_extended,
                        const uint32_t id, const uint8_t dlc,
                        const uint8_t* const data) {
  self->super_.vptr_->receive(&self->super_, is_extended, id, dlc, data);
}

void RearBoxCan_receive_hp(RearBoxCan* const self, const bool is_extended,
                           const uint32_t id, const uint8_t dlc,
                           const uint8_t* const data) {
  self->super_.vptr_->receive_hp(&self->super_, is_extended, id, dlc, data);
}

void RearBoxCan_periodic_update(RearBoxCan* const self,
                                const TickType_t current_tick) {
  self->super_.vptr_->periodic_update(&self->super_, current_tick);
}

/* virtual function definition -----------------------------------------------*/
// from CanTransceiver base class
void __RearBoxCan_configure(CanTransceiver* const self) {
  (void)self;

  /* config can filter -------------------------------------------------------*/
  FDCAN_FilterTypeDef can_filter0 = {
      .IdType = FDCAN_EXTENDED_ID,
      .FilterIndex = 1,
      .FilterType = FDCAN_FILTER_DUAL,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = FRONT_SENSOR_1_CANID,
      .FilterID2 = FRONT_SENSOR_1_CANID,
  };
  CHECK_INIT(HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter0));

  // configure filter for non-matched id and remote frame
  CHECK_INIT(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                          FDCAN_REJECT_REMOTE,
                                          FDCAN_REJECT_REMOTE));

  // start can
  CHECK_INIT(HAL_FDCAN_Start(&hfdcan1));

  // activate interrupt for high priority can signal to fifo1
  CHECK_INIT(HAL_FDCAN_ActivateNotification(&hfdcan1,
                                            FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0));

  // can singal struct
  nturt_can_config_rear_sensor_Check_Receive_Timeout_Init(&can_rear_sensor_rx);

  // mutex for can signal struct
  can_rear_sensor_rx_mutex =
      xSemaphoreCreateMutexStatic(&can_rear_sensor_rx_mutex_cb);
  can_rear_sensor_tx_mutex =
      xSemaphoreCreateMutexStatic(&can_rear_sensor_tx_mutex_cb);
}

// from CanTransceiver base class
void __RearBoxCan_receive(CanTransceiver* const self, const bool is_extended,
                          const uint32_t id, const uint8_t dlc,
                          const uint8_t* const data) {
  (void)self;
  (void)is_extended;

  xSemaphoreTake(can_rear_sensor_rx_mutex, portMAX_DELAY);
  nturt_can_config_rear_sensor_Receive(&can_rear_sensor_rx, data, id, dlc);
  xSemaphoreGive(can_rear_sensor_rx_mutex);
}

// from CanTransceiver base class
void __RearBoxCan_receive_hp(CanTransceiver* const self, const bool is_extended,
                             const uint32_t id, const uint8_t dlc,
                             const uint8_t* const data) {
  (void)self;
  (void)is_extended;
  (void)id;
  (void)dlc;
  (void)data;
}

// from CanTransceiver base class
void __RearBoxCan_periodic_update(CanTransceiver* const _self,
                                  const TickType_t current_tick) {
  (void)current_tick;

  RearBoxCan* self = (RearBoxCan*)_self;

  xSemaphoreTake(can_rear_sensor_rx_mutex, portMAX_DELAY);
  nturt_can_config_rear_sensor_Check_Receive_Timeout(&can_rear_sensor_rx);
  xSemaphoreGive(can_rear_sensor_rx_mutex);

  if (self->tx_error_) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(self->super_.can_handle_) > 16) {
      ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_TX, ERROR_CLEAR);
      self->tx_error_ = false;
    }
  } else {
    xSemaphoreTake(can_rear_sensor_tx_mutex, portMAX_DELAY);
    CHECK_TRANSMISSION(
        nturt_can_config_rear_sensor_Transmit(&can_rear_sensor_tx));
    xSemaphoreGive(can_rear_sensor_tx_mutex);
  }
}

/* constructor ---------------------------------------------------------------*/
void RearBoxCan_ctor(RearBoxCan* self, CanHandle* const can_handle) {
  // construct inherited class and redirect virtual function
  CanTransceiver_ctor(&self->super_, can_handle);
  static struct CanTransceiverVtbl vtbl = {
      .configure = __RearBoxCan_configure,
      .receive = __RearBoxCan_receive,
      .receive_hp = __RearBoxCan_receive_hp,
      .periodic_update = __RearBoxCan_periodic_update,
  };
  self->super_.vptr_ = &vtbl;

  // initialize member variable
  self->tx_error_ = false;
  self->rx_error_ = 0;
}

/* member function -----------------------------------------------------------*/
inline ModuleRet RearBoxCan_transmit(RearBoxCan* const self,
                                     const bool is_extended, const uint32_t id,
                                     const uint8_t dlc, uint8_t* const data) {
  return CanTransceiver_transmit(&self->super_, is_extended, id, dlc, data);
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/
uint32_t frame_id_to_index(uint32_t id) {
  switch (id) {
    case FRONT_SENSOR_1_CANID:
      return FRONT_SENSOR_1_INDEX;

    default:
      return 0;
  }
}

/* Callback function ---------------------------------------------------------*/
// coderdbc callback function for getting current time in ms
uint32_t __get__tick__() { return get_10us() / 100; }

// coderdbc callback function for sending can message
inline int __send_can_message__(uint32_t msgid, uint8_t ide, uint8_t* d,
                                uint8_t len) {
  return RearBoxCan_transmit(&rear_box_can, ide, msgid, len, d);
}

// coderdbc callback function called when receiving a new frame
void _FMon_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid) {
  if (mon->cycle_error) {
    printf("timeout lift: %lx\n", msgid);
    int index = frame_id_to_index(msgid);
    if (index != FRAME_NOT_DEFINED) {
      if (index & FRAME_CRITICAL_MASK) {
        if (rear_box_can.rx_error_ & FRAME_CRITICAL_MASK) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                   ERROR_CLEAR);
        }
      } else if (index & FRAME_OPTIONAL_MASK) {
        if (rear_box_can.rx_error_ & FRAME_OPTIONAL_MASK) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                   ERROR_CLEAR);
        }
      }
      rear_box_can.rx_error_ &= ~index;
    }

    mon->cycle_error = false;
  }
}

// coderdbc callback function called when reception timeout
void _TOut_MONO_nturt_can_config(FrameMonitor_t* mon, uint32_t msgid,
                                 uint32_t lastcyc) {
  (void)lastcyc;

  if (!mon->cycle_error) {
    int index = frame_id_to_index(msgid);
    if (index != FRAME_NOT_DEFINED) {
      if (index & FRAME_CRITICAL_MASK) {
        if (!(rear_box_can.rx_error_ & FRAME_CRITICAL_MASK)) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_CRITICAL,
                                   ERROR_SET);
        }
      } else if (index & FRAME_OPTIONAL_MASK) {
        if (!(rear_box_can.rx_error_ & FRAME_OPTIONAL_MASK)) {
          ErrorHandler_write_error(&error_handler, ERROR_CODE_CAN_RX_OPTIONAL,
                                   ERROR_SET);
        }
      }
      rear_box_can.rx_error_ |= index;
    }

    mon->cycle_error = true;
  }
}

/* coderdbc callback function called befor transmission ----------------------*/
// don't need to do anything since status and error code are update in
// error_handler and status_controller respectively
void FTrn_REAR_SENSOR_Status_nturt_can_config(REAR_SENSOR_Status_t* m) {
  (void)m;
}

// note: the mutex for can frame is already taken in the transmit function
void FTrn_REAR_SENSOR_1_nturt_can_config(REAR_SENSOR_1_t* m) {
  xSemaphoreTake(travel_sensor.mutex, portMAX_DELAY);
  m->REAR_SENSOR_Rear_Left_Suspension_phys = travel_sensor.left;
  m->REAR_SENSOR_Rear_Right_Suspension_phys = travel_sensor.right;
  xSemaphoreGive(travel_sensor.mutex);

  xSemaphoreTake(wheel_speed_sensor.mutex, portMAX_DELAY);
  m->REAR_SENSOR_Rear_Left_Wheel_Speed_phys = wheel_speed_sensor.left;
  m->REAR_SENSOR_Rear_Right_Wheel_Speed_phys = wheel_speed_sensor.right;
  xSemaphoreGive(wheel_speed_sensor.mutex);
}

void FTrn_REAR_SENSOR_2_nturt_can_config(REAR_SENSOR_2_t* m) {
  xSemaphoreTake(tire_temp_sensor.mutex, portMAX_DELAY);
  m->REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys =
      (tire_temp_sensor.left[0] + tire_temp_sensor.left[1]) / 2;
  m->REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys =
      (tire_temp_sensor.left[2] + tire_temp_sensor.left[3]) / 2;
  m->REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys =
      (tire_temp_sensor.left[4] + tire_temp_sensor.left[5]) / 2;
  m->REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys =
      (tire_temp_sensor.left[6] + tire_temp_sensor.left[7]) / 2;
  m->REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys =
      (tire_temp_sensor.right[0] + tire_temp_sensor.right[1]) / 2;
  m->REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys =
      (tire_temp_sensor.right[2] + tire_temp_sensor.right[3]) / 2;
  m->REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys =
      (tire_temp_sensor.right[4] + tire_temp_sensor.right[5]) / 2;
  m->REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys =
      (tire_temp_sensor.right[6] + tire_temp_sensor.right[7]) / 2;
  xSemaphoreGive(tire_temp_sensor.mutex);
}
