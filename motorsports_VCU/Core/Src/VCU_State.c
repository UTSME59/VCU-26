/*
 * VCU_State.c
 *
 *  Created on: 26 Mar 2026
 *      Author: NickM
 */
#include <main.h>
#include <VCU_State.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include <stdlib.h>


static float clampUnit(float value) {
	if (value < 0) {
		return 0;
	}

	if (value > 1) {
		return 1;
	}

	return value;
}

void updateTorque(
    Torque *torque,
    const Throttle *throttle,
    const PM150_Data *motor,
    const OrionData *battery,
    PowerLimiter *limiter,
    uint8_t drive_allowed,
    uint32_t now_ms)
{
	/*
     * Calculate the driver's request independently.
     * Units are 0.1 Nm.
     */
	uint32_t request = ((uint32_t)throttle->pos * TORQUE_MAX) / TPS_POS_MAX;

	if (request > TORQUE_MAX) {
		request = TORQUE_MAX;
	}

	torque->driver_request = (uint16_t)request;

	/* Default to zero until the measurement checks pass. */
    torque->power_limit = 0U;
    float voltage = battery->pack_instant_voltage_v;
    float current = battery->pack_current_a * POWER_CURRENT_SIGN;
    uint32_t age_ms = now_ms - battery->power_received_ms;

    limiter->measured_kw = voltage * current * 0.001;

    limiter->data_valid =
    	battery->power_received 		&&
		age_ms <= POWER_DATA_TIMEOUT_MS &&
		isfinite(voltage) 				&&
		isfinite(current)				&&
		voltage >= 300					&&
		voltage <= 504					&&
		current >= -50					&&
		current <= 300;

    /*
     * Reset the PI when drive is not allowed, the driver
     * requests zero torque, or power measurements are invalid.
     */
    if (!limiter->data_valid || !drive_allowed || request == 0) {
    	limiter->integral = 0;
    	limiter->reduction = 0;
    	limiter->p_term = 0;
    	limiter->i_term = 0;
    	limiter->active = 0;

    	return;
    }

    /*
     * Apply the existing mechanical-power ceiling first.
     * At very low speed, the driver's request and TORQUE_MAX
     * govern the base torque.
     */
    float base = (float)request;
    float omega = fabsf((float)motor->mc_rpm) * 0.10471976; // rpm * 2pi/60

    if (omega > 1) {
    	float rpm_limit = (POWER_TARGET_KW * 1000 / omega) * 10;

    	if (rpm_limit < base) {
    		base = rpm_limit;
    	}
    }

    /*
     * Update the PI only when a fresh measurement arrives,
     * or when activating the controller.
     */
    if (!limiter->active || battery->power_sequence != limiter->previous_sequence) {
    	float dt = 0;

    	if (limiter->active) {
    		uint32_t elapsed = battery->power_received_ms - limiter->previous_sample_ms;

    		if (elapsed <= POWER_CONTROL_MAX_GAP_MS) {
    			dt = elapsed * 0.001;
    		} else {
                // Do not integrate across a long gap in the measurement stream
                limiter->integral = 0.0f;
    		}
    	}

    	float error = limiter->measured_kw - limiter->target_kw;
    	float proportional = limiter->kp * error;
    	float output = proportional + limiter->integral;

    	/*
         * Stop integration if it would push the output
         * further beyond its limits. Allow it to unwind.
         */
    	uint8_t winding_up =
    			(output >= 1 && error > 0) ||
				(output <= 0 && error < 0);

    	if (!winding_up) {
    		limiter->integral = clampUnit(limiter->integral + limiter->ki * error * dt);
    	}

    	limiter->p_term = proportional;
    	limiter->i_term = limiter->integral;
    	limiter->reduction = clampUnit(proportional + limiter->integral);
    	limiter->previous_sequence = battery->power_sequence;
    	limiter->previous_sample_ms = battery->power_received_ms;
    	limiter->active = 1;
    }

     /*
     * Convert the reduction fraction into a torque ceiling.
     * The final driver-request comparison occurs in CAN TX.
     */
    torque->power_limit = (uint16_t)(base * (1 - limiter->reduction));
}

void updateTractionLimiter(Torque *torque, TractionLimiter *tc, Wss *wss, uint8_t in_rtd, uint8_t enabled, uint32_t now_ms) {
	uint32_t elapsed_ms = now_ms - tc->last_update_ms;
	tc->last_update_ms = now_ms;
	float dt = fminf((float)elapsed_ms, TC_MAX_STEP_MS) * 0.001;

	float fl = wss->fl * 0.01;
	float fr = wss->fr * 0.01;
	float rl = wss->rl * 0.01;
	float rr = wss->rr * 0.01;
	float front_avg = 0.5 * (fl + fr);

	tc->data_valid =
		wss->fresh_mask == WSS_ALL_FRESH &&
		fl <= TC_MAX_SPEED_KMH &&
		fr <= TC_MAX_SPEED_KMH &&
		rl <= TC_MAX_SPEED_KMH &&
		rr <= TC_MAX_SPEED_KMH &&
		 fabsf(fl - fr) <= TC_FRONT_GAP_KMH + TC_FRONT_GAP_FRACTION * front_avg &&
        rl >= TC_REAR_MIN_FRACTION * fl &&
        rr >= TC_REAR_MIN_FRACTION * fr;

	uint8_t was_active = tc->active;
	uint16_t base = torque->power_limit;
	tc->active = TC_ENABLE &&
				 in_rtd &&
				 enabled &&
				 base > 0 &&
				 tc->data_valid &&
				 front_avg >= TC_MIN_SPEED_KMH;
	tc->slip_pct = 0;
	float target_reduction = 0;

	if (tc->active) {
		float slip_left = (rl - fl) / fl;
		float slip_right = (rr - fr) / fr;
		tc->slip_pct = 100 * fmaxf(slip_left, slip_right);

		float error = tc->slip_pct - TC_SLIP_REF_PCT;
		float p = tc->kp * error;

		if (!was_active || elapsed_ms > TC_MAX_STEP_MS) {
			tc->integral = 0;
		} else {
			float i_max = clampUnit(1 - p);
			float next_i = tc->integral + tc->ki * error *dt;

			if (error > 0 && tc->integral < i_max) {
				tc->integral = fminf(next_i, i_max);
			} else {
				tc->integral = fmaxf(next_i, 0);
			}
		}

		target_reduction = clampUnit(p + tc->integral);
	} else {
		tc->integral = 0;
	}

	if (!TC_ENABLE || !in_rtd || base == 0) {
		tc->reduction = 0;
	} else {
		float release_floor = tc->reduction - TC_RELEASE_PER_S * dt;
		tc->reduction = fmaxf(target_reduction, release_floor);
	}

	torque->traction_limit = (uint16_t)(base * (1 - tc->reduction));
}

void updateFaultLEDs(Faults *faults)
{
	if (faults->ams) {
		HAL_GPIO_WritePin(BMS_FAULT_LED_GPIO_Port, BMS_FAULT_LED_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BMS_FAULT_LED_GPIO_Port, BMS_FAULT_LED_Pin, GPIO_PIN_RESET);
	}

	if (faults->imd) {
		HAL_GPIO_WritePin(IMD_FAULT_LED_GPIO_Port, IMD_FAULT_LED_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(IMD_FAULT_LED_GPIO_Port, IMD_FAULT_LED_Pin, GPIO_PIN_RESET);
	}

	if (faults->pdoc) {
		HAL_GPIO_WritePin(PDOC_FAULT_LED_GPIO_Port, PDOC_FAULT_LED_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(PDOC_FAULT_LED_GPIO_Port, PDOC_FAULT_LED_Pin, GPIO_PIN_RESET);
	}

	if (faults->bspd) {
		HAL_GPIO_WritePin(BSPD_FAULT_LED_GPIO_Port, BSPD_FAULT_LED_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BSPD_FAULT_LED_GPIO_Port, BSPD_FAULT_LED_Pin, GPIO_PIN_RESET);
	}
}


void updateWarningLEDs(OrionData *orion) {
	// Less Than 50% Charge
	if ((orion->pack_voltage < 400) || (orion->low_cell < 3.4)) {
		orion->voltage_status = 2;
	} else if ((orion->pack_voltage < 430) || (orion->low_cell < 3.6)) {
		HAL_GPIO_WritePin(WARNING_CHARGE_OUT_GPIO_Port, WARNING_CHARGE_OUT_Pin, GPIO_PIN_SET);
		orion->voltage_status = 1;
	} else if ((orion->pack_voltage >= 400) || (orion->low_cell >= 3.6)) {
		orion->voltage_status = 0;
		HAL_GPIO_WritePin(WARNING_CHARGE_OUT_GPIO_Port, WARNING_CHARGE_OUT_Pin, GPIO_PIN_RESET);
	}

	// Temperature Control
	if (orion->cell_temperature > 50) {
		orion->temp_status = 2;
	} else if ((orion->cell_temperature > 45) && (orion->temp_status == 0 || orion->temp_status == 1)) {
		orion->temp_status = 1;
		HAL_GPIO_WritePin(WARNING_TEMP_OUT_GPIO_Port, WARNING_TEMP_OUT_Pin, GPIO_PIN_SET);
	} else if ((orion->cell_temperature) < 40 && (orion->temp_status == 0 || orion->temp_status == 1)) {
		HAL_GPIO_WritePin(WARNING_TEMP_OUT_GPIO_Port, WARNING_TEMP_OUT_Pin, GPIO_PIN_RESET);
	}
}

uint8_t checkFault(Faults *faults) {
	if (faults->ams || faults->pdoc || faults->imd || faults->bspd) { // Removed fault, legacy code 2024 - Gavin
		return 1;
	}
	else {
		return 0;
	}
}

static uint8_t apps_disagree_count = 0;

void checkAppsBrakePlausibilityError(AppsBrakeErrors *apps_brake_errors, Throttle *throttle, Brake *brake)
{

	if (throttle->tps1_range_error || throttle->tps2_range_error) {
		apps_brake_errors->tpsRange = 1;
	}

	if (brake->front_range_error || brake->rear_range_error) {
		apps_brake_errors->brakeRange = 1;
	}

	// CHECK APPS
	if (abs((int32_t)throttle->tps1 - (int32_t)throttle->tps2) > 10) {              //added in32_t since abs wants signed, 10 is from t.4.2
			if (apps_disagree_count < 5){											// 5 since run at 40hz, rules say 100ms
				apps_disagree_count++;												//T.4.2.5
			}
	}
	else{
			apps_disagree_count = 0;
	}
			apps_brake_errors->apps = (apps_disagree_count >= 5);

	// CHECK APPS BRK
	if (brake->avg > BRAKE_PRESSURE_APPS_THRESHOLD && throttle->pos > 25) {
		apps_brake_errors->appsBrake = 1;
	}

	// CLEAR APPS BRAKE PLAUSIBILITY IF THROTTLE LESS THAN 5%
	if (throttle->pos < 5){
		apps_brake_errors->appsBrake = 0;
	}

	apps_brake_errors->appsBrakePlausibility =
			(apps_brake_errors->tpsRange ||
			apps_brake_errors->brakeRange ||
			apps_brake_errors->apps ||
			apps_brake_errors->appsBrake);
}
