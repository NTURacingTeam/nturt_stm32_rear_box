#include "user_main.h"

// glibc include
#include <stdio.h>

// stm32 include
#include "main.h"

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

/* Exported variable ---------------------------------------------------------*/
// stm32_module
ErrorHandler error_handler;
LedController led_controller;

// project
BrakeLightController brake_light_controller;
RearBoxCan rear_box_can;
StatusController status_controller;

TaskHandle_t freertos_stats_task_handle;

/* Static variable -----------------------------------------------------------*/
// stm32_module
static struct error_callback_cb auxiliary_error_callback_cb;
static struct led_cb led_cb[NUM_LED];

#ifndef PRODUCTION
// project
static StaticTask_t freertos_stats_task_cb;
static uint32_t freertos_stats_task_buffer[FREERTOS_STATS_TASK_STACK_SIZE];
#endif  // PRODUCTION

/* Static function prototype -------------------------------------------------*/
/**
 * @brief Function to initialize led module.
 *
 * @return None.
 */
static void led_module_init();

/**
 * @brief Function to augment the function of error handler when error occurred.
 *
 * @param[in] argument Not used.
 * @param[in] error_code The error code.
 * @return None.
 */
static void auxiliary_error_handler(void *const argument, uint32_t error_code);

/* Entry point ---------------------------------------------------------------*/
void user_init() {
  // light up brake light when initializing
  HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_SET);

  // stm32_module
  ErrorHandler_ctor(&error_handler);
  ErrorHandler_start(&error_handler);
  led_module_init();

// project
#ifndef TESTING
  BrakeLightController_ctor(&brake_light_controller);
  BrakeLightController_start(&brake_light_controller);
  RearBoxCan_ctor(&rear_box_can, &hfdcan1);
  RearBoxCan_start(&rear_box_can);
  StatusController_ctor(&status_controller);
  StatusController_start(&status_controller);

  // register error callback function
  ErrorHandler_add_error_callback(&error_handler, &auxiliary_error_callback_cb,
                                  auxiliary_error_handler, NULL,
                                  ERROR_CODE_ALL);
#endif  // TESTING

#ifndef PRODUCTION
  // start freertos stats task for monitoring freertos states
  freertos_stats_task_handle = xTaskCreateStatic(
      freertos_stats_task, "freertos_stats_task",
      FREERTOS_STATS_TASK_STACK_SIZE, NULL, TaskPriorityLowest,
      freertos_stats_task_buffer, &freertos_stats_task_cb);
#endif  // PRODUCTION
}

/* Task implementation -------------------------------------------------------*/
void freertos_stats_task(void *argument) {
  (void)argument;

  while (1) {
    int task_size = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_status = pvPortMalloc(task_size * sizeof(TaskStatus_t));
    uint32_t total_run_time;
    uxTaskGetSystemState(task_status, task_size, &total_run_time);

    printf("Task count = %d\n", task_size);
    printf("No Name             Status      Usage   HW\n");

    for (int i = 0; i < task_size; i++) {
      int runtime_percent =
          100 * task_status[i].ulRunTimeCounter / total_run_time;

      char *task_state;
      switch (task_status[i].eCurrentState) {
        case eRunning:
          task_state = "RUNNING";
          break;
        case eReady:
          task_state = "READY";
          break;
        case eBlocked:
          task_state = "BLOCKED";
          break;
        case eSuspended:
          task_state = "SUSPENED";
          break;
        case eDeleted:
          task_state = "DELETED";
          break;
        default:
          task_state = "UNKNOWN";
      }

      printf("%2d %-16s %-8s %7d%% %4u\n", i, task_status[i].pcTaskName,
             task_state, runtime_percent, task_status[i].usStackHighWaterMark);
    }
    vPortFree(task_status);
    vTaskDelay(1000);
  }
}

/* Exported function ---------------------------------------------------------*/
uint32_t get_10us() { return htim2.Instance->CNT; }

/* Static function -----------------------------------------------------------*/
static void led_module_init() {
  LedController_ctor(&led_controller);

  // should be in the same order as taht in project_def.h
  LedController_add_led(&led_controller, &led_cb[LED_BUILTIN_GREEN],
                        LD2_GPIO_Port, LD2_Pin);
  LedController_add_led(&led_controller, &led_cb[LED_BRAKE_LIGHT],
                        BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin);

  LedController_start(&led_controller);

  // turn on brake light
  LedController_turn_on(&led_controller, LED_BRAKE_LIGHT);
}

static void auxiliary_error_handler(void *const argument, uint32_t error_code) {
  (void)argument;

  // write error code to can frame
  uint32_t full_error_code;
  ErrorHandler_get_error(&error_handler, &full_error_code);
  xSemaphoreTake(can_rear_sensor_tx_mutex, portMAX_DELAY);
  can_rear_sensor_tx.REAR_SENSOR_Status.REAR_SENSOR_Error_Code =
      full_error_code;
  xSemaphoreGive(can_rear_sensor_tx_mutex);
}

/* Callback function ---------------------------------------------------------*/
// glibc callback function for printf
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// freertos callback functions for runtime stats
void configureTimerForRunTimeStats(void) { HAL_TIM_Base_Start_IT(&htim2); }
unsigned long getRunTimeCounterValue(void) { return get_10us(); }

// freertos callback function when task stack overflowed
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
  printf("Stack overflowed for %s\n", pcTaskName);
  Error_Handler();
}

// stm32_module callback function when assertion failed
void __module_assert_fail(const char *assertion, const char *file,
                          unsigned int line, const char *function) {
  printf("%s:%u: %s: STM32 module assertion `%s' failed.\n", file, line,
         function, assertion);
  Error_Handler();
}
