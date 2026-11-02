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


uint16_t calculateTorque(Throttle *throttle, PM150_Data *pm150)
{
	// Calculate Torque Command
	// ADD REDUNDANCY
	uint16_t _torque = (throttle->pos) * (TORQUE_MAX) / TPS_POS_MAX;

	float torque_ceiling = 0;
	float torque_nm = _torque * 0.1;

	// 75 kW power limit
	float mc_rads = 0.1047 * pm150->mc_rpm; // 2pi/60 * RPM
	torque_ceiling = (POWER_LIMIT / mc_rads);

	// Choose the lower of the two torque values
	if (torque_nm > torque_ceiling) {
		_torque = torque_ceiling * 10; // convert from Nm back to bits
	} else {
		_torque = torque_nm * 10;
	}

	if (_torque > TORQUE_MAX) {
		_torque = TORQUE_MAX;
	} else if (_torque < 0) {
		_torque = 0;
	}

	return _torque;
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

void checkAppsBrakePlausibilityError(AppsBrakeErrors *apps_brake_errors, Throttle *throttle, Brake *brake)
{

	if (throttle->tps1_range_error || throttle->tps1_range_error) {
		apps_brake_errors->tpsRange = 1;
	}

	if (brake->front_range_error || brake->rear_range_error) {
		apps_brake_errors->brakeRange = 1;
	}

	// CHECK APPS
	if (abs(throttle->tps1 - throttle->tps2) > 10) {
		apps_brake_errors->apps = 1;
	}
	else {
		apps_brake_errors->apps = 0;
	}

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
