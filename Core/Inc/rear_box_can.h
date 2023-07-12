/**
 * @file rear_box_can.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef REAR_BOX_CAN_H
#define REAR_BOX_CAN_H

// glibc include
#include <stdbool.h>
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"

// can_config include
#include "nturt_can_config.h"
#include "nturt_can_config_rear_sensor-binutil.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* Exported variable ---------------------------------------------------------*/
// c-coderdbc can singal struct
extern nturt_can_config_rear_sensor_tx_t can_rear_sensor_tx;
extern nturt_can_config_rear_sensor_rx_t can_rear_sensor_rx;

// mutex
extern SemaphoreHandle_t can_rear_sensor_tx_mutex;
extern SemaphoreHandle_t can_rear_sensor_rx_mutex;

/* class inherited from CanTransceiver ---------------------------------------*/
typedef struct rear_box_can {
  // inherited class
  CanTransceiver super_;

  /// @brief Transmit error flag.
  bool tx_error_;

  /// @brief Frame receive timeout error flag, where every bit represent a frame
  /// error.
  uint32_t rx_error_;
} RearBoxCan;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for RearBoxCan.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void RearBoxCan_ctor(RearBoxCan* self, CanHandle* const can_handle);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add front box can to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet RearBoxCan_start(RearBoxCan* const self);

/**
 * @brief Function to configure can peripherial settings when starting.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void RearBoxCan_configure(RearBoxCan* const self);

/**
 * @brief Function for receiving can frame.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] is_extended If the frame is extended.
 * @param[in] id ID of the frame.
 * @param[in] dlc Data length code.
 * @param[in] data Data of the frame.
 * @return None.
 * @note This function is virtual.
 */
void RearBoxCan_receive(RearBoxCan* const self, const bool is_extended,
                        const uint32_t id, const uint8_t dlc,
                        const uint8_t* const data);

/**
 * @brief Function for receiving high priority can frame.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] is_extended If the frame is extended.
 * @param[in] id ID of the frame.
 * @param[in] dlc Data length code.
 * @param[in] data Data of the frame.
 * @return None.
 * @note This function is virtual.
 */
void RearBoxCan_receive_hp(RearBoxCan* const self, const bool is_extended,
                           const uint32_t id, const uint8_t dlc,
                           const uint8_t* const data);

/**
 * @brief Function for doing periodic chores, e.g. checking for timeout, sending
 * periodic message.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] current_tick Current tick time.
 * @return None.
 * @note This function is virtual.
 */
void RearBoxCan_periodic_update(RearBoxCan* const self,
                                const TickType_t current_tick);

/**
 * @brief Function for transmitting can frame.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] is_extended If the frame is extended.
 * @param[in] id ID of the frame.
 * @param[in] dlc Data length code.
 * @param[in] data Data of the frame.
 * @return ModuleRet Error code.
 */
ModuleRet RearBoxCan_transmit(RearBoxCan* const self, const bool is_extended,
                              const uint32_t id, const uint8_t dlc,
                              uint8_t* const data);

/* Exported function ---------------------------------------------------------*/

#endif  // REAR_BOX_CAN_H
