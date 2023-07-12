/**
 * @file user_main.h
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief
 */

#ifndef USER_MAIN_H
#define USER_MAIN_H

// glibc include
#include <stdint.h>

// freertos include
#include "FreeRTOS.h"
#include "task.h"

// stm32_module include
#include "stm32_module/stm32_module.h"

// project include
#include "brake_light_controller.h"
#include "project_def.h"
#include "rear_box_can.h"
#include "status_controller.h"

/* macro ---------------------------------------------------------------------*/
// initialization checking macro
#define CHECK_INIT(RET)    \
  do {                     \
    if ((RET) != HAL_OK) { \
      Error_Handler();     \
    }                      \
  } while (0)

/* Exported variable ---------------------------------------------------------*/
// stm32_module
extern ErrorHandler error_handler;
extern LedController led_controller;

// project
extern BrakeLightController brake_light_controller;
extern RearBoxCan rear_box_can;
extern StatusController status_controller;

extern TaskHandle_t freertos_stats_task_handle;

/* Entry point ---------------------------------------------------------------*/
/**
 * @brief Project initialize entry point.
 * @retval None
 */
void user_init();

/* Task implementation -------------------------------------------------------*/
/**
 * @brief Function implementing the freertos_stats_task thread.
 * @param argument: Not used
 * @retval None
 */
void freertos_stats_task(void *argument);

/* Exported function ---------------------------------------------------------*/
/**
 * @brief Function to get 10 microsecond since the microcontroller booted.
 *
 * The time source comes from the counter of 32-bit timer TIM3 that tick up
 * every 10 microseconds.
 * @retval Current 10 microsecond.
 * @warning The value will wraparound roughly 11 hours.
 */
uint32_t get_10us();

#endif  // USER_MAIN_H
