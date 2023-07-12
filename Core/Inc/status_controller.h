/**
 * @file status_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef STATUS_CONTROLLER_H
#define STATUS_CONTROLLER_H

// glibc include
#include <stdbool.h>
#include <stdint.h>

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define STATUS_CONTROLLER_TASK_PRIORITY TaskPriorityHigh
#define STATUS_CONTROLLER_TASK_STACK_SIZE 256
#define STATUS_CONTROLLER_TASK_PERIOD 50

// rtd condition
#define RTD_CON_CAN_TX 0x00000001UL
#define RTD_CON_CAN_RX_CRITICAL 0x00000002UL

#define RTD_CON_ALL (RTD_CON_CAN_TX | RTD_CON_CAN_RX_CRITICAL)

/* type ----------------------------------------------------------------------*/
typedef enum {
  /// @brief Initial state, checking RTD condition.
  StatusInit = 0,

  /// @brief Ready state, waiting RTD command to run.
  StatusReady,

  /// @brief Transition state, playing RTD sound.
  StatusRTD,

  /// @brief Running state, motor torque enabled.
  StatusRunning,

  /// @brief Error state, motor torque disabled, error handling.
  StatusError,
} StatusControllerState;

/* Exported variable ---------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling vehicle status.
 *
 */
typedef struct status_controller {
  // inherited class
  Task super_;

  /// @brief Current status.
  StatusControllerState status_;

  /// @brief RTD condition;
  uint32_t rtd_condition_;

  /// @brief Task stack buffer.
  StackType_t task_stack_[STATUS_CONTROLLER_TASK_STACK_SIZE];

  /// @brief Error callback control block.
  struct error_callback_cb error_callback_cb_;

  /// @brief Task handle for blink_builtin_led_task.
  TaskHandle_t blink_builtin_led_task_handle_;

  /// @brief Task handle for play_rtd_sound_task.
  TaskHandle_t play_rtd_sound_task_handle_;

  /// @brief Task control block for blink_builtin_led_task.
  StaticTask_t blink_builtin_led_task_cb_;

  /// @brief Task stack buffer for blink_builtin_led_task.
  StackType_t blink_builtin_led_task_stack_[configMINIMAL_STACK_SIZE];
} StatusController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for StatusController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void StatusController_ctor(StatusController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add status controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 * @note This function is virtual.
 */
ModuleRet StatusController_start(StatusController* const self);

/**
 * @brief Function to get currnet status.
 *
 * @param[in,out] self The instance of the class.
 * @param[out] status Current status.
 * @return ModuleRet Error code.
 */
ModuleRet StatusController_get_status(StatusController* const self,
                                      StatusControllerState* const status);

/**
 * @brief Function to reset status to StatusInit.
 *
 * @param self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet StatusController_reset_status(StatusController* const self);

/**
 * @brief Function to handle APPS, BSE, critical can rx error.
 *
 * @param[in,out] self The instance of the class.
 * @param[in] error_code Error code.
 * @return None
 * @warning For internal use only.
 */
void StatusController_error_handler(void* const _self, uint32_t error_code);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return None
 * @warning For internal use only.
 */
void StatusController_task_code(void* const _self);

/* Exported function ---------------------------------------------------------*/

#endif  // STATUS_CONTROLLER_H
