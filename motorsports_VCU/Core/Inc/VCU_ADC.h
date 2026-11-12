#ifndef __VCU_ADC_H
#define __VCU_ADC_H



#include "VCU_State.h"
#include "stm32f3xx_hal.h"
#include <stdint.h>



// Sensor Definitions
#define ADC_AVG_FILTER 4        /**< @brief Number of samples to average for the moving average filter. */
#define ADC_RESOLUTION 16       /**< @brief ADC resolution in bits. */

// -- THROTTLE CONFIG -- //
#define TPS1_MIN 27000          /**< @brief The minimum raw ADC value for Throttle Position Sensor 1. */
#define TPS1_MAX 36900          /**< @brief The maximum raw ADC value for Throttle Position Sensor 1. */
#define TPS2_MIN 17000          /**< @brief The minimum raw ADC value for Throttle Position Sensor 2. */
#define TPS2_MAX 25850          /**< @brief The maximum raw ADC value for Throttle Position Sensor 2. */



// -- BRAKE CONFIG -- //
#define BRAKE_FRONT_MIN 7000    /**< @brief The minimum raw ADC value for the front brake pressure sensor. */
#define BRAKE_FRONT_MAX 58900   /**< @brief The maximum raw ADC value for the front brake pressure sensor. */
#define BRAKE_REAR_MIN 6100     /**< @brief The minimum raw ADC value for the rear brake pressure sensor. */
#define BRAKE_REAR_MAX 58900    /**< @brief The maximum raw ADC value for the rear brake pressure sensor. */

#define BRAKE_PRESSURE_MIN 0    /**< @brief The minimum calibrated brake pressure value (e.g., in PSI). */
#define BRAKE_PRESSURE_MAX 13789/**< @brief The maximum calibrated brake pressure value (e.g., in PSI). */


// -- SENSOR PLAUSIBILITY -- //
#define SENSOR_MIN  0.05         /**< @brief Minimum plausible sensor reading as a percentage (10%). */
#define SENSOR_MAX  0.9         /**< @brief Maximum plausible sensor reading as a percentage (90%). */


/** @brief Holds raw ADC values from sensors for diagnostics. */
typedef struct ADCHANDLER
{
  SDADC_HandleTypeDef *hsdadc1;
  SDADC_HandleTypeDef *hsdadc2;

  uint16_t tps1;  /**< @brief Raw ADC value from throttle sensor 1. */
  uint16_t tps2;  /**< @brief Raw ADC value from throttle sensor 2. */
  uint16_t front; /**< @brief Raw ADC value from front brake sensor. */
  uint16_t rear;  /**< @brief Raw ADC value from rear brake sensor. */
  uint16_t charge_en;	/**< @brief Raw ADC value from CHARGE_EN_IN (AIN4/PE11, 300k pull-up to 5V). ~65500 open, ~0 when pulled low. */
} AdcHandler;

void adcInit(AdcHandler *adc);

void readADC(AdcHandler *adc);
void readADC_test(AdcHandler *adc);

void updateThrottlePosition(AdcHandler *adc, Throttle *throttle);
void updateBrakePressure(AdcHandler *adc, Brake *brake);

float constrain(float val, float lower, float upper);
float map(float val, float fromLow, float fromHigh, float toLow, float toHigh);

void print_ADC(AdcHandler *hadc);

#endif /* __VCU_ADC_H */
