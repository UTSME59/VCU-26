#include <VCU_ADC.h>
#include <stdio.h>
#include <math.h>



uint8_t print_throttle = 0;
uint8_t print_brake = 0;


AdcHandler *_hadc;

void adcInit(AdcHandler *hadc)
{
	_hadc = hadc;
}

// Counts every lost/failed SDADC conversion. Watch it in Live Expressions - it should stay flat.
volatile uint32_t sdadc_err_count = 0;

static void adcFilter(uint16_t *dst, uint32_t raw)
{
	uint16_t v = (uint16_t)(raw - 32780);
	*dst = (uint16_t)((*dst * (ADC_AVG_FILTER - 1) + v) / ADC_AVG_FILTER);
}

// One JSWSTART converts the whole injected group (every channel selected in the .ioc).
// Results are routed by the channel number tagged in JDATAR, not by read order, so a
// lost JEOC (overrun caused by interrupt latency) can no longer shift values between signals.
// Any HAL failure calls InjectedStop, which returns the HAL state to READY - previously a
// single missed JEOC left the handle stuck in STATE_INJ forever and every read returned 0.
static void readGroup(AdcHandler *hadc, SDADC_HandleTypeDef *h)
{
	uint32_t ch;
	uint32_t raw;

	if (HAL_SDADC_InjectedStart(h) != HAL_OK) {
		HAL_SDADC_InjectedStop(h);
		sdadc_err_count++;
		return;
	}

	for (uint32_t i = 0; i < h->InjectedChannelsNbr; i++) {
		if (HAL_SDADC_PollForInjectedConversion(h, 5) != HAL_OK) {
			HAL_SDADC_InjectedStop(h);
			sdadc_err_count++;
			return;
		}
		raw = HAL_SDADC_InjectedGetValue(h, &ch);

		// Pin -> channel mapping from EV_VCU.ioc
		if (h == hadc->hsdadc2) {
			if      (ch == 3) adcFilter(&hadc->tps1, raw);       // PE12 TPS1
			else if (ch == 2) adcFilter(&hadc->tps2, raw);       // PE13 TPS2
		} else {
			if      (ch == 8) adcFilter(&hadc->front, raw);      // PE8  BRAKE_FRONT
			else if (ch == 7) adcFilter(&hadc->rear, raw);       // PE9  BRAKE_REAR
			else if (ch == 1) adcFilter(&hadc->charge_en, raw);  // PE11 AIN4 CHARGE_EN_IN
		}
	}
}

void readADC(AdcHandler *hadc)
{
	readGroup(hadc, hadc->hsdadc2);   // TPS1, TPS2
	readGroup(hadc, hadc->hsdadc1);   // BRAKE_FRONT, BRAKE_REAR, CHARGE_EN
}


void readADC_test(AdcHandler *hadc) {
	uint32_t channel;
	int16_t adc_val;
	uint32_t error;

	// SDADC2 group
	printf("\nSDADC2\n");
	printf("\nstate (begin): %d\n", hadc->hsdadc2->State);

	error = HAL_SDADC_InjectedStart(hadc->hsdadc2);
	printf("\tstart error: %d\n", error);
	error = HAL_SDADC_PollForInjectedConversion(hadc->hsdadc2, 1000);
	printf("\tpoll error: %d\n", error);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
//	printf("[%d] - %d\n", channel, adc_val);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
//	printf("[%d] - %d\n", channel, adc_val);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
//	printf("[%d] - %d\n", channel, adc_val);


	if (hadc->hsdadc2->State == HAL_SDADC_STATE_INJ || HAL_SDADC_STATE_REG_INJ) {
		HAL_SDADC_InjectedStop(hadc->hsdadc2);
	}
	printf("\nstate (end): %d\n", hadc->hsdadc2->State);




	// SDADC1 group
	printf("\nSDADC1\n");

	printf("state (begin): %d\n", hadc->hsdadc1->State);

	error = HAL_SDADC_InjectedStart(hadc->hsdadc1);
	printf("\tstart error: %d\n", error);
	error = HAL_SDADC_PollForInjectedConversion(hadc->hsdadc1, 1000);
	printf("\tpoll error: %d\n", error);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
	printf("[%d] - %d\n", channel, adc_val);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
	printf("[%d] - %d\n", channel, adc_val);

	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel);// - 32780;
	if (adc_val < 0 ) {
		adc_val = 0;
	}
	printf("[%d] - %d\n", channel, adc_val);


	if (hadc->hsdadc1->State == HAL_SDADC_STATE_INJ || HAL_SDADC_STATE_REG_INJ) {
		HAL_SDADC_InjectedStop(hadc->hsdadc1);
	}
	printf("\nstate (end): %d\n", hadc->hsdadc1->State);

}


void updateThrottlePosition(AdcHandler *hadc, Throttle *throttle)
{
	uint32_t adc_val;
	static uint8_t tps1_oor = 0, tps2_oor = 0;

	// Convert Throttle Value to Position
	adc_val = constrain(hadc->tps1, TPS1_MIN, TPS1_MAX);
	throttle->tps1 = map(adc_val, TPS1_MIN, TPS1_MAX, TPS_POS_MIN, TPS_POS_MAX);

	adc_val = constrain(hadc->tps2, TPS2_MIN, TPS2_MAX);
	throttle->tps2 = map(adc_val, TPS2_MIN, TPS2_MAX, TPS_POS_MIN, TPS_POS_MAX);

	throttle->pos = (throttle->tps1 + throttle->tps2) / 2;

	// Range check: must persist 5 samples at 40 Hz (>100 ms) before latching (T.4.2.5)
	uint32_t adc_min = (uint32_t)(SENSOR_MIN * (1u << ADC_RESOLUTION));
	uint32_t adc_max = (uint32_t)(SENSOR_MAX * (1u << ADC_RESOLUTION));

	if (hadc->tps1 < adc_min || hadc->tps1 > adc_max) { if (tps1_oor < 5) tps1_oor++; }
	else tps1_oor = 0;
	if (hadc->tps2 < adc_min || hadc->tps2 > adc_max) { if (tps2_oor < 5) tps2_oor++; }
	else tps2_oor = 0;

	if (tps1_oor >= 5) throttle->tps1_range_error = 1;
	if (tps2_oor >= 5) throttle->tps2_range_error = 1;
}

void updateBrakePressure(AdcHandler *hadc, Brake *brake)
{
	uint32_t adc_val;   // was int16_t: raw > 32767 wrapped negative -> pressure read 0
	static uint8_t front_oor = 0, rear_oor = 0;

	// Convert Brake Value to Pressure
	adc_val = constrain(hadc->front, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX);
	brake->front = map(adc_val, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	adc_val = constrain(hadc->rear, BRAKE_REAR_MIN, BRAKE_REAR_MAX);
	brake->rear = map(adc_val, BRAKE_REAR_MIN, BRAKE_REAR_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	brake->avg = (brake->front + brake->rear) / 2;

	// Range check: must persist 5 samples at 40 Hz (>100 ms) before latching (T.4.3.3)
	uint32_t adc_min = (uint32_t)(SENSOR_MIN * (1u << ADC_RESOLUTION));
	uint32_t adc_max = (uint32_t)(SENSOR_MAX * (1u << ADC_RESOLUTION));

	if (hadc->front < adc_min || hadc->front > adc_max) { if (front_oor < 5) front_oor++; }
	else front_oor = 0;
	if (hadc->rear < adc_min || hadc->rear > adc_max)   { if (rear_oor < 5) rear_oor++; }
	else rear_oor = 0;

	if (front_oor >= 5) brake->front_range_error = 1;
	if (rear_oor  >= 5) brake->rear_range_error  = 1;
}

// NEED TO DELETE AND FIX FUNCTION
//apps_brake_errors.appsBrakePlausibility = 0;

float constrain(float val, float lower, float upper)
{
float newVal = val;
if(val < lower) newVal = lower;
if (val > upper) newVal = upper;

return newVal;
}

float map(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
	return (val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void print_ADC(AdcHandler *hadc)
{

	if (print_throttle){
		printf("TPS1: [%d],", hadc->tps1);
		printf("TPS2: [%d]", hadc->tps2);
	}

	if (print_brake){
		printf("\tBrakeF: [%d],", hadc->front);
		printf("BrakeR: [%d]", hadc->rear);
	}

	if (print_throttle || print_brake){
		printf("\n");
	}

}
