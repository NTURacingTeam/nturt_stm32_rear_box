#include "brake_light_controller.h"

// glibc include
#include <stdbool.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module incude
#include "stm32_module/stm32_module.h"

// project include
#include "project_def.h"
#include "rear_box_can.h"
#include "status_controller.h"
#include "user_main.h"

/* Exported variable ---------------------------------------------------------*/

/* Static variable -----------------------------------------------------------*/

/* Static function prototype -------------------------------------------------*/

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet BrakeLightController_start(BrakeLightController* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __BrakeLightController_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  BrakeLightController* const self = (BrakeLightController*)_self;
  return Task_create_freertos_task(
      (Task*)self, "status_controller", BRAKE_LIGHT_CONTROLLER_TASK_PRIORITY,
      self->task_stack_, BRAKE_LIGHT_CONTROLLER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void BrakeLightController_ctor(BrakeLightController* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, BrakeLightController_task_code);
  static struct TaskVtbl vtbl = {
      .start = __BrakeLightController_start,
  };
  self->super_.vptr_ = &vtbl;
}

/* member function -----------------------------------------------------------*/
void BrakeLightController_task_code(void* const _self) {
  BrakeLightController* const self = (BrakeLightController*)_self;
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    StatusControllerState status;
    StatusController_get_status(&status_controller, &status);
    if (status == StatusRunning) {
      xSemaphoreTake(can_rear_sensor_rx_mutex, portMAX_DELAY);
      bool brake_macro =
          can_rear_sensor_rx.FRONT_SENSOR_1.FRONT_SENSOR_Brake_Micro;
      xSemaphoreGive(can_rear_sensor_rx_mutex);

      if (brake_macro) {
        LedController_turn_on(&led_controller, LED_BRAKE_LIGHT);
      } else {
        LedController_turn_off(&led_controller, LED_BRAKE_LIGHT);
      }
    } else {
      LedController_turn_on(&led_controller, LED_BRAKE_LIGHT);
    }

    vTaskDelayUntil(&last_wake, BRAKE_LIGHT_CONTROLLER_TASK_PERIOD);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/

/* Callback function ---------------------------------------------------------*/
