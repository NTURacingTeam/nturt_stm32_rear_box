/**
 * @file brake_light_controller.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef BRAKE_LIGHT_CONTROLLER_H
#define BRAKE_LIGHT_CONTROLLER_H

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"

/* macro ---------------------------------------------------------------------*/
// parameter
#define BRAKE_LIGHT_CONTROLLER_TASK_PRIORITY TaskPriorityLow
#define BRAKE_LIGHT_CONTROLLER_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define BRAKE_LIGHT_CONTROLLER_TASK_PERIOD 10

/* Exported variable ---------------------------------------------------------*/

/* class ---------------------------------------------------------------------*/
/**
 * @brief Class for controlling brake_light.
 *
 */
typedef struct brake_light_controller {
  // inherited class
  Task super_;

  StackType_t task_stack_[BRAKE_LIGHT_CONTROLLER_TASK_STACK_SIZE];
} BrakeLightController;

/* constructor ---------------------------------------------------------------*/
/**
 * @brief Constructor for BrakeLightController.
 *
 * @param[in,out] self The instance of the class.
 * @return None.
 */
void BrakeLightController_ctor(BrakeLightController* const self);

/* member function -----------------------------------------------------------*/
/**
 * @brief Function to add dashboard controller to freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @return ModuleRet Error code.
 */
ModuleRet BrakeLightController_start(BrakeLightController* const self);

/**
 * @brief Function to run in freertos task.
 *
 * @param[in,out] self The instance of the class.
 * @retval None
 * @warning For internal use only.
 */
void BrakeLightController_task_code(void* const _self);

/* Exported function ---------------------------------------------------------*/

#endif  // BRAKE_LIGHT_CONTROLLER_H
