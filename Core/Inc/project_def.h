#ifndef PROJECT_DEF_H
#define PROJECT_DEF_H

// stm32 include
#include "main.h"

/* project config ------------------------------------------------------------*/
// build config
// #define PRODUCTION

#ifndef PRODUCTION
/* testing -------------------------------------------------------------------*/

#if 0
#define TESTING
#endif
#endif  // PRODUCTION

// freertos stack size
#define FREERTOS_STATS_TASK_STACK_SIZE 256

/* module config -------------------------------------------------------------*/
/* led -----------------------------------------------------------------------*/
#define NUM_LED_BUILTIN 1
#define NUM_LED_USER 1
#define NUM_LED (NUM_LED_BUILTIN + NUM_LED_USER)

// built in led
#define LED_BUILTIN_BASE 0
#define LED_BUILTIN(X) (LED_BUILTIN_BASE + X)

#define LED_BUILTIN_GREEN LED_BUILTIN(0)

// user led
#define LED_USER_BASE (LED_BUILTIN_BASE + NUM_LED_BUILTIN)
#define LED_USER(X) (LED_USER_BASE + X)

#define LED_BRAKE_LIGHT LED_USER(0)

#endif  // PROJECT_DEF_H
