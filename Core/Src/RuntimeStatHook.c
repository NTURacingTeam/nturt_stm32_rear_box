#include "main.h"
#include "FreeRTOS.h"
#include "RuntimeStatHook.h"

extern TIM_HandleTypeDef* const p_htim_rtosStatBase;

volatile unsigned long rtosStatTick = 0;

void configureTimerForRunTimeStats(void) {
	rtosStatTick = 0;
	HAL_TIM_Base_Start_IT(p_htim_rtosStatBase);
}

unsigned long getRunTimeCounterValue(void) {
	return rtosStatTick;
}

void IncRunTimeCounter(void) {
    rtosStatTick++;
}