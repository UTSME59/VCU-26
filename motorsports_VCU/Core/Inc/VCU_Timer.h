#ifndef __VCU_TIMER_H
#define __VCU_TIMER_H

#include "VCU_State.h"
#include <stdint.h>
#include "stm32f3xx_hal.h"

// -- WHEEL SPEED SENSOR CONFIG-- //
#define WSS_PULSE_PER_REVOLUTION 20	/**< @brief The number of pulses detected that corresponds to a single revolution. */
#define WSS_FILTER 5				/**< @brief The number of samples to average over when updating the time between pulses. */
#define WSS_CLOCK_PERIOD 0.000001	/**< @brief The time in seconds per clock count. */
#define WHEEL_CIRCUMFERENCE 1.57


typedef struct TIMERHANDLER
{
	TIM_HandleTypeDef *htim2;
	TIM_HandleTypeDef *htim3;
	TIM_HandleTypeDef *htim4;
	TIM_HandleTypeDef *htim5;
	TIM_HandleTypeDef *htim6;
	TIM_HandleTypeDef *htim7;

	uint8_t tim3_flag;
	uint8_t tim4_flag;
	uint8_t tim6_flag;
	uint8_t tim7_flag;

} TimerHandler;

typedef struct WSSTIMER
{
	uint32_t t0;  /**< @brief Front right wheel speed sensor previous clock time. */
	uint32_t t1;  /**< @brief Front right wheel speed sensor latest clock time. */
	uint32_t dt;  /**< @brief Front right wheel speed sensor difference between pulse. */
	uint32_t overflowcount; /**< @brief Count Clock Overflow. */
} WssTimer;


void timerInit(TimerHandler *htimer);
void timerStart();
void updateWSS(Wss *wss);
void Tmr_Error_Handler();

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



#endif /* __VCU_CANBUS_H */
