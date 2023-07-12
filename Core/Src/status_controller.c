#include "status_controller.h"

// glibc include
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// stm32 include
#include "main.h"

// freertos include
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// project include
#include "project_def.h"
#include "rear_box_can.h"
#include "user_main.h"

/* Exported variable ---------------------------------------------------------*/

/* Static function prototype -------------------------------------------------*/
static void blink_builtin_led_task_code(void* argument);

/* virtual function redirection ----------------------------------------------*/
inline ModuleRet StatusController_start(StatusController* const self) {
  return self->super_.vptr_->start((Task*)self);
}

/* virtual function definition -----------------------------------------------*/
// from Task base class
ModuleRet __StatusController_start(Task* const _self) {
  module_assert(IS_NOT_NULL(_self));

  StatusController* const self = (StatusController*)_self;

  // register error callback function
  ErrorHandler_add_error_callback(&error_handler, &self->error_callback_cb_,
                                  StatusController_error_handler, (void*)self,
                                  ERROR_CODE_ALL);

  return Task_create_freertos_task(
      (Task*)self, "status_controller", STATUS_CONTROLLER_TASK_PRIORITY,
      self->task_stack_, STATUS_CONTROLLER_TASK_STACK_SIZE);
}

/* constructor ---------------------------------------------------------------*/
void StatusController_ctor(StatusController* const self) {
  module_assert(IS_NOT_NULL(self));

  // construct inherited class and redirect virtual function
  Task_ctor((Task*)self, StatusController_task_code);
  static struct TaskVtbl vtbl = {
      .start = __StatusController_start,
  };
  self->super_.vptr_ = &vtbl;

  // initialize member variable
  self->status_ = StatusInit;

  // set rtd condition controlled by error handler to true
  self->rtd_condition_ = RTD_CON_CAN_TX | RTD_CON_CAN_RX_CRITICAL;

  self->blink_builtin_led_task_handle_ = NULL;
}

/* member function -----------------------------------------------------------*/
ModuleRet StatusController_get_status(StatusController* const self,
                                      StatusControllerState* const status) {
  *status = self->status_;

  return ModuleOK;
}

ModuleRet StatusController_reset_status(StatusController* const self) {
  switch (self->status_) {
    case StatusRunning:
      self->status_ = StatusInit;
      break;

    case StatusInit:
    case StatusReady:
    case StatusRTD:
    case StatusError:
      break;
  }

  return ModuleOK;
}

void StatusController_error_handler(void* const _self, uint32_t error_code) {
  StatusController* const self = (StatusController*)_self;

  if (error_code & ERROR_CODE_CAN_TX) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_CAN_TX;
    } else {
      self->rtd_condition_ |= RTD_CON_CAN_TX;
    }
  }

  if (error_code & ERROR_CODE_CAN_RX_CRITICAL) {
    if (error_code & ERROR_SET) {
      self->rtd_condition_ &= ~RTD_CON_CAN_RX_CRITICAL;
    } else {
      self->rtd_condition_ |= RTD_CON_CAN_RX_CRITICAL;
    }
  }
}

void StatusController_task_code(void* const _self) {
  StatusController* const self = (StatusController*)_self;
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    /* update status ---------------------------------------------------------*/
    switch (self->status_) {
      case StatusInit:
        if (self->rtd_condition_ == RTD_CON_ALL) {
          if (self->blink_builtin_led_task_handle_ == NULL ||
              eTaskGetState(self->blink_builtin_led_task_handle_) == eDeleted) {
            self->blink_builtin_led_task_handle_ = xTaskCreateStatic(
                blink_builtin_led_task_code, "blink_builtin_led",
                configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                self->blink_builtin_led_task_stack_,
                &self->blink_builtin_led_task_cb_);
          }
          self->status_ = StatusRunning;
        }
        break;

      case StatusRunning:
        if (self->rtd_condition_ != RTD_CON_ALL) {
          if (self->blink_builtin_led_task_handle_ != NULL &&
              eTaskGetState(self->blink_builtin_led_task_handle_) != eDeleted) {
            vTaskDelete(self->blink_builtin_led_task_handle_);
          }
          self->status_ = StatusError;
        }
        break;

      case StatusError:
        if (self->rtd_condition_ == RTD_CON_ALL) {
          if (self->blink_builtin_led_task_handle_ == NULL ||
              eTaskGetState(self->blink_builtin_led_task_handle_) == eDeleted) {
            self->blink_builtin_led_task_handle_ = xTaskCreateStatic(
                blink_builtin_led_task_code, "blink_builtin_led",
                configMINIMAL_STACK_SIZE, NULL, TaskPriorityLowest,
                self->blink_builtin_led_task_stack_,
                &self->blink_builtin_led_task_cb_);
          }
          self->status_ = StatusRunning;
        }
        break;

      case StatusReady:
      case StatusRTD:
        break;
    }

    /* write status to can frame ---------------------------------------------*/
    xSemaphoreTake(can_rear_sensor_tx_mutex, portMAX_DELAY);
    can_rear_sensor_tx.REAR_SENSOR_Status.REAR_SENSOR_Status = self->status_;
    xSemaphoreGive(can_rear_sensor_tx_mutex);

    vTaskDelayUntil(&last_wake, STATUS_CONTROLLER_TASK_PERIOD);
  }
}

/* Exported function ---------------------------------------------------------*/

/* Static function -----------------------------------------------------------*/
static void blink_builtin_led_task_code(void* argument) {
  (void)argument;

  while (1) {
    LedController_blink(&led_controller, LED_BUILTIN_GREEN, 500);
    vTaskDelay(1000);
  }
}

/* Callback function ---------------------------------------------------------*/
