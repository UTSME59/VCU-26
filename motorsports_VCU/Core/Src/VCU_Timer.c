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
WssTimer _wss_rl = {0};
WssTimer _wss_rr = {0};



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

		HAL_TIM_IC_Start_IT(_htimer->htim2, TIM_CHANNEL_3); // FR
		HAL_TIM_IC_Start_IT(_htimer->htim2, TIM_CHANNEL_4); // RL

		HAL_TIM_IC_Start_IT(_htimer->htim5, TIM_CHANNEL_2); // FL
		HAL_TIM_IC_Start_IT(_htimer->htim5, TIM_CHANNEL_3); // RR
}


static uint16_t wheelSpeed(WssTimer *sample, uint32_t now_ms, uint8_t bit, uint8_t *fresh)
{
    uint32_t age_ms = now_ms - sample->last_sample_ms;

    if (sample->dt == 0 || age_ms >= WSS_TIMEOUT_MS) {
        return 0;
    }

    float speed = (float)WHEEL_CIRCUMFERENCE * 360.0f / (sample->dt * (float)WSS_CLOCK_PERIOD * WSS_PULSE_PER_REVOLUTION);

    if (speed >= UINT16_MAX) {
        return UINT16_MAX;
    }

    if (age_ms < WSS_TC_TIMEOUT_MS) {
        *fresh |= bit;
    }

    return (uint16_t)speed;
}

void updateWSS(Wss *wss)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    const WssTimer fl = _wss_fl;
    const WssTimer fr = _wss_fr;
    const WssTimer rl = _wss_rl;
    const WssTimer rr = _wss_rr;
    uint32_t now_ms = HAL_GetTick();
    __set_PRIMASK(irq_state);

    wss->fresh_mask = 0U;
    wss->fl = wheelSpeed(&fl, now_ms, 1U << 0, &wss->fresh_mask);
    wss->fr = wheelSpeed(&fr, now_ms, 1U << 1, &wss->fresh_mask);
    wss->rl = wheelSpeed(&rl, now_ms, 1U << 2, &wss->fresh_mask);
    wss->rr = wheelSpeed(&rr, now_ms, 1U << 3, &wss->fresh_mask);
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
            if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
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
                	_wss_fr.last_sample_ms = now_ms;  /* FR block, TIM2 CH3 */

					if (_wss_fr.dt == 0) {
						_wss_fr.dt = dt;
					} else {
						_wss_fr.dt = ((_wss_fr.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
					}
				}
            } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            	uint32_t now_ms = HAL_GetTick();

                // First pulse, or first pulse after a timeout
                if (!_wss_rl.has_previous_pulse || (uint32_t)(now_ms - _wss_rl.last_pulse_ms) >= WSS_TIMEOUT_MS) {
                    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

                    _wss_rl.t0 = capture;
                    _wss_rl.t1 = capture;
                    _wss_rl.dt = 0;
                    _wss_rl.overflowcount = 0;
                    _wss_rl.last_pulse_ms = now_ms;
                    _wss_rl.has_previous_pulse = 1;
                    break;
                }

                _wss_rl.last_pulse_ms = now_ms;

                // Calculate time between successive pulses
                _wss_rl.t0 = _wss_rl.t1;
                _wss_rl.t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

                uint32_t dt = 0;

                if (_wss_rl.overflowcount == 0) {
                    dt = _wss_rl.t1 - _wss_rl.t0;
                } else if (_wss_rl.overflowcount == 1) {
                    dt = _wss_rl.t1 + (htim->Init.Period + 1U) - _wss_rl.t0;
                }

                _wss_rl.overflowcount = 0;

                // Update filtered pulse period
                if (dt > 0) {
                	_wss_rl.last_sample_ms = now_ms;  /* RL block, TIM2 CH4 */

					if (_wss_rl.dt == 0) {
						_wss_rl.dt = dt;
					} else {
						_wss_rl.dt = ((_wss_rl.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
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
                	_wss_fl.last_sample_ms = now_ms;  /* FL block, TIM5 CH2 */

					if (_wss_fl.dt == 0) {
						_wss_fl.dt = dt;
					} else {
						_wss_fl.dt = ((_wss_fl.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
					}
				}
            } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            	uint32_t now_ms = HAL_GetTick();

                // First pulse, or first pulse after a timeout
                if (!_wss_rr.has_previous_pulse || (uint32_t)(now_ms - _wss_rr.last_pulse_ms) >= WSS_TIMEOUT_MS) {
                    uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

                    _wss_rr.t0 = capture;
                    _wss_rr.t1 = capture;
                    _wss_rr.dt = 0;
                    _wss_rr.overflowcount = 0;
                    _wss_rr.last_pulse_ms = now_ms;
                    _wss_rr.has_previous_pulse = 1;
                    break;
                }

                _wss_rr.last_pulse_ms = now_ms;

                // Calculate time between successive pulses
                _wss_rr.t0 = _wss_rr.t1;
                _wss_rr.t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

                uint32_t dt = 0;

                if (_wss_rr.overflowcount == 0) {
                    dt = _wss_rr.t1 - _wss_rr.t0;
                } else if (_wss_rr.overflowcount == 1) {
                    dt = _wss_rr.t1 + (htim->Init.Period + 1U) - _wss_rr.t0;
                }

                _wss_rr.overflowcount = 0;

                // Update filtered pulse period
                if (dt > 0) {
                	_wss_rr.last_sample_ms = now_ms;  /* RR block, TIM5 CH3 */

					if (_wss_rr.dt == 0) {
						_wss_rr.dt = dt;
					} else {
						_wss_rr.dt = ((_wss_rr.dt * (WSS_FILTER - 1)) + dt) / WSS_FILTER;
					}
				}
            }
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    switch((uint32_t)htim->Instance) {
    	case (uint32_t) TIM2:
//			printf("TIM2\n");

			// FR and RL share TIM2
    		_wss_fr.overflowcount++;
    		_wss_rl.overflowcount++;

    		 if (_wss_fr.overflowcount > 1) {
    			 _wss_fr.dt = 0;
    		 }

    		 if (_wss_rl.overflowcount > 1) {
    			 _wss_rl.dt = 0;
    		 }

    		break;

        case (uint32_t) TIM3: // 200 Hz
			_htimer->tim3_flag = 1;

        	break;

        case (uint32_t) TIM4: // 40 Hz
			_htimer->tim4_flag = 1;

			break;

        case (uint32_t) TIM5:

			// FL and RR share TIM5
        	_wss_fl.overflowcount++;
        	_wss_rr.overflowcount++;

        	if (_wss_fl.overflowcount > 1) {
        		 _wss_fl.dt = 0;
        	}

			if (_wss_rr.overflowcount > 1) {
				_wss_rr.dt = 0;
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
