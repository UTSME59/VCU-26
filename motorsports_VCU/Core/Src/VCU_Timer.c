/*
 * VCU_Timer.c
 *
 *  Created on: 26 Mar 2026
 *      Author: NickM
 */
#include "VCU_Timer.h"
#include "stm32f3xx_hal.h"

TimerHandler *_htimer;
WssTimer _wss_fl = {0};
WssTimer _wss_fr = {0};



void timerInit(TimerHandler *htimer)
{
	_htimer = htimer;
}

void timerStart()
{
	// Timer flags for tracking periodic events
	  if (HAL_TIM_Base_Start_IT(_htimer->htim2) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}

	  if (HAL_TIM_Base_Start_IT(_htimer->htim3) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}

	  if (HAL_TIM_Base_Start_IT(_htimer->htim4) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}

	  if (HAL_TIM_Base_Start_IT(_htimer->htim5) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}

	  if (HAL_TIM_Base_Start_IT(_htimer->htim6) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}

	  if (HAL_TIM_Base_Start_IT(_htimer->htim7) != HAL_OK)
		{
		  /* Starting Error */
		  Tmr_Error_Handler();
		}


	  HAL_TIM_IC_Start_IT(_htimer->htim2, TIM_IT_CC3);
	  HAL_TIM_IC_Start_IT(_htimer->htim5, TIM_IT_CC2);
}


void updateWSS(Wss *wss)
{
	// Calculate FR Wheel Speed
	if (!_wss_fr.has_previous_pulse || (uint32_t)(HAL_GetTick() - _wss_fr.last_pulse_ms) >= WSS_TIMEOUT_MS || _wss_fr.dt == 0) {
		wss->fr = 0;
	} else {
		float rev_per_second = ( 1.0 / (float) ( _wss_fr.dt * (float) WSS_CLOCK_PERIOD ) ) / (float) WSS_PULSE_PER_REVOLUTION;
		wss->fr =  rev_per_second * (float) WHEEL_CIRCUMFERENCE * 360;
	}

	// Calculate FL Wheel Speed
	if (!_wss_fl.has_previous_pulse || (uint32_t)(HAL_GetTick() - _wss_fl.last_pulse_ms) >= WSS_TIMEOUT_MS || _wss_fl.dt == 0) {
		wss->fl = 0;
	} else {
		float rev_per_second = ( 1.0 / (float) ( _wss_fl.dt * (float) WSS_CLOCK_PERIOD ) ) / (float) WSS_PULSE_PER_REVOLUTION;
		wss->fl =  rev_per_second * (float) WHEEL_CIRCUMFERENCE * 360;
	}
}

void Tmr_Error_Handler()
{

}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    switch ((uint32_t)htim->Instance)
    {
        // FRONT RIGHT WHEEL - TIM2 Channel 3
        case (uint32_t)TIM2:
            if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
            {
                uint32_t now_ms = HAL_GetTick();

                // First pulse, or first pulse after a timeout
                if (!_wss_fr.has_previous_pulse || (uint32_t)(now_ms - _wss_fr.last_pulse_ms) >= WSS_TIMEOUT_MS) {
                    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

                    _wss_fr.t0 = capture;
                    _wss_fr.t1 = capture;
                    _wss_fr.dt = 0;
                    _wss_fr.overflowcount = 0;
                    _wss_fr.last_pulse_ms = now_ms;
                    _wss_fr.has_previous_pulse = 1;
                    break;
                }

                _wss_fr.last_pulse_ms = now_ms;

                // Calculate time between successive pulses
                _wss_fr.t0 = _wss_fr.t1;
                _wss_fr.t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

                uint32_t dt = 0;

                if (_wss_fr.overflowcount == 0) {
                    dt = _wss_fr.t1 - _wss_fr.t0;
                } else if (_wss_fr.overflowcount == 1) {
                    dt = _wss_fr.t1 + (htim->Init.Period + 1U) - _wss_fr.t0;
                }

                _wss_fr.overflowcount = 0;

                // Update filtered pulse period
                if (dt > 0) {
					if (_wss_fr.dt == 0) {
						_wss_fr.dt = dt;
					} else {
						_wss_fr.dt = ((_wss_fr.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
					}
				}
            }
            break;

        // FRONT LEFT WHEEL - TIM5 Channel 2
        case (uint32_t)TIM5:
            if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
                uint32_t now_ms = HAL_GetTick();

                // First pulse, or first pulse after a timeout
                if (!_wss_fl.has_previous_pulse || (uint32_t)(now_ms - _wss_fl.last_pulse_ms) >= WSS_TIMEOUT_MS) {
                    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                    _wss_fl.t0 = capture;
                    _wss_fl.t1 = capture;
                    _wss_fl.dt = 0;
                    _wss_fl.overflowcount = 0;
                    _wss_fl.last_pulse_ms = now_ms;
                    _wss_fl.has_previous_pulse = 1;
                    break;
                }

                _wss_fl.last_pulse_ms = now_ms;

                // Calculate time between successive pulses
                _wss_fl.t0 = _wss_fl.t1;
                _wss_fl.t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                uint32_t dt = 0;

                if (_wss_fl.overflowcount == 0) {
                    dt = _wss_fl.t1 - _wss_fl.t0;
                } else if (_wss_fl.overflowcount == 1) {
                    dt = _wss_fl.t1 + (htim->Init.Period + 1U) - _wss_fl.t0;
                }

                _wss_fl.overflowcount = 0;

                // Update filtered pulse period
                if (dt > 0) {
					if (_wss_fl.dt == 0) {
						_wss_fl.dt = dt;
					} else {
						_wss_fl.dt = ((_wss_fl.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
					}
				}
            }
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    switch((uint32_t)htim->Instance)    {


    	case (uint32_t) TIM2:
//			printf("TIM2\n");

			// keep track of clock overflow
			_wss_fr.overflowcount++;

    		// if more than 1 overflow per clock period then wheel has stopped
    		if (_wss_fr.overflowcount > 1) {
    			_wss_fr.dt = 0;
    		}

    		break;

        case (uint32_t) TIM3: // 200 Hz
			_htimer->tim3_flag = 1;

        	break;

        case (uint32_t) TIM4: // 40 Hz
			_htimer->tim4_flag = 1;

			break;

        case (uint32_t) TIM5:

			// keep track of clock overflow
        	_wss_fl.overflowcount++;

        	// if more than 1 overflow per clock period then wheel has stopped
			if (_wss_fl.overflowcount > 1) {
				_wss_fl.dt = 0;
			}

			break;


        case (uint32_t) TIM6: // 30 Hz
			_htimer->tim6_flag = 1;

			break;

        case (uint32_t) TIM7: // 1 Hz
            _htimer->tim7_flag = 1;

            break;
    }
}
