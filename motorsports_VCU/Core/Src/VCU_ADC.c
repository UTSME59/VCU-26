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

void readADC(AdcHandler *hadc)
{
	uint32_t channel;
	uint16_t adc_val;


	// Read Suspot FL
	HAL_SDADC_InjectedStart(hadc->hsdadc2);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc2, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel) - 32780;
	hadc->suspot_fl = (hadc->suspot_fl * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;

	// Read Throttle 2
	HAL_SDADC_InjectedStart(hadc->hsdadc2);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc2, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel) - 32780;
	hadc->tps2 = (hadc->tps2 * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;

	// Read Throttle 1
	HAL_SDADC_InjectedStart(hadc->hsdadc2);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc2, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc2, &channel) - 32780;
	//printf("\n[TPS1] ADC VAL: %u, CHANNEL: %d\n", adc_val, channel);
	hadc->tps1 = (hadc->tps1 * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;



	// Read Suspot FR
	HAL_SDADC_InjectedStart(hadc->hsdadc1);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc1, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel) - 32780;
	hadc->suspot_fr = (hadc->suspot_fr * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;

	// Read Rear Brake
	HAL_SDADC_InjectedStart(hadc->hsdadc1);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc1, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel) - 32780;
	hadc->rear = (hadc->rear * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;

	// Read Front Brake
	HAL_SDADC_InjectedStart(hadc->hsdadc1);
	HAL_SDADC_PollForInjectedConversion(hadc->hsdadc1, 1000);
	adc_val = HAL_SDADC_InjectedGetValue(hadc->hsdadc1, &channel) - 32780;
	hadc->front = (hadc->front * (ADC_AVG_FILTER - 1) + adc_val) / ADC_AVG_FILTER;

}


void readADC_test(AdcHandler *hadc) {
	uint32_t channel;
	int16_t adc_val;
	uint32_t error;

	// Read Suspot FL
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




	// Read Suspot FL
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

	// Convert Throttle Value to Position
	adc_val = constrain(hadc->tps1, TPS1_MIN, TPS1_MAX);
	throttle->tps1 = map(adc_val, TPS1_MIN, TPS1_MAX, TPS_POS_MIN, TPS_POS_MAX);


	adc_val = constrain(hadc->tps2, TPS2_MIN, TPS2_MAX);
	throttle->tps2 = map(adc_val, TPS2_MIN, TPS2_MAX, TPS_POS_MIN, TPS_POS_MAX);


	throttle->pos = (throttle->tps1 + throttle->tps2) / 2;

	// RANGE CHECK
	uint16_t adc_min = SENSOR_MIN * pow(2, ADC_RESOLUTION);
	uint16_t adc_max = SENSOR_MAX * pow(2, ADC_RESOLUTION);
	if (hadc->tps1 < adc_min || hadc->tps1 > adc_max) {
		throttle->tps1_range_error = 1;
	}
	if (hadc->tps2 < adc_min || hadc->tps2 > adc_max) {
		throttle->tps2_range_error = 1;
	}
}

void updateBrakePressure(AdcHandler *hadc, Brake *brake)
{
	int16_t adc_val;

	// Convert Brake Value to Pressure
	adc_val = constrain(hadc->front, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX);
	brake->front = map(adc_val, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	adc_val = constrain(hadc->rear, BRAKE_REAR_MIN, BRAKE_REAR_MAX);
	brake->rear = map(adc_val, BRAKE_REAR_MIN, BRAKE_REAR_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	brake->avg = (brake->front + brake->rear) / 2;


	// Range Check
	uint16_t adc_min = SENSOR_MIN * pow(2, ADC_RESOLUTION);
	uint16_t adc_max = SENSOR_MAX * pow(2, ADC_RESOLUTION);
	if (hadc->front < adc_min || hadc->front > adc_max) {
		brake->front_range_error = 1;
	}
	if (hadc->rear < adc_min || hadc->rear > adc_max) {
		brake->rear_range_error = 1;
	}
}

void updateSuspot(AdcHandler *hadc, Suspot *suspot)
{
	int16_t adc_val;

	// Convert Suspot to Distance
	adc_val = constrain(hadc->suspot_fl, SUSPOT_FL_MIN, SUSPOT_FL_MAX);
	suspot->fl = map(adc_val, SUSPOT_FL_MIN, SUSPOT_FL_MAX, SUSPOT_DIST_MIN, SUSPOT_DIST_MAX);

	adc_val = constrain(hadc->suspot_fr, SUSPOT_FR_MIN, SUSPOT_FR_MAX);
	suspot->fr = map(adc_val, SUSPOT_FR_MIN, SUSPOT_FR_MAX, SUSPOT_DIST_MIN, SUSPOT_DIST_MAX);
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
