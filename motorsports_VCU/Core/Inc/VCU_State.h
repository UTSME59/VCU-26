#ifndef __VCU_STATE_H
#define __VCU_STATE_H


#include <stdint.h>

#define TPS_POS_MIN 0          /**< @brief The minimum mapped value for Throttle Position Sensor. */
#define TPS_POS_MAX 100       /**< @brief The maximum mapped value for Throttle Position Sensor. */


// -- TORQUE CONFIG -- //
#define TORQUE_MAX 2290          /**< @brief The maximum torque command value to be sent. */
#define BRAKE_PRESSURE_START_THRESHOLD 500 /**< @brief Pressure threshold to trigger RTD. */
#define BRAKE_PRESSURE_APPS_THRESHOLD 500 /**< @brief Pressure threshold to trigger APPS/Brake plausibility check. */
#define POWER_LIMIT 75000.0f          /**< @brief The maximum torque command value to be sent. */

// Car States Definition
typedef enum CARSTATE
{
  IDLE,
  PRECHARGED,
  RTD,
  FAULT
} CarState;


/** @brief Flags for APPS (Accelerator Pedal Position Sensor) and Brake plausibility errors. */


/** @brief Flags for APPS (Accelerator Pedal Position Sensor) and Brake plausibility errors. */
typedef struct APPSBRAKSERRORS
{
  uint8_t appsBrakePlausibility; /**< @brief Master flag, true if any plausibility error is active. */
  uint8_t tpsRange;     /**< @brief Flag for throttle sensor out-of-range error. */
  uint8_t brakeRange;   /**< @brief Flag for brake sensor out-of-range error. */
  uint8_t apps;         /**< @brief Flag for disagreement between the two throttle sensors. */
  uint8_t appsBrake;    /**< @brief Flag for braking while throttle is over 25%. */
} AppsBrakeErrors;


// Fault LEDs
typedef struct FAULTS
{
	uint8_t ams;
	uint8_t pdoc;
	uint8_t imd;
	uint8_t bspd;
} Faults;

// PDM State
typedef struct PDM_STATUS
{
	uint8_t standby;
	uint8_t precharged;
	uint8_t rtd;
	uint8_t drive_enable;
} PDM_Status;

// Accumulator State
typedef struct ORIONDATA
{
	float   pack_voltage;
	uint8_t cell_temperature;
	float   low_cell;
	uint8_t voltage_status;
	uint8_t temp_status;
} OrionData;

typedef struct C185_DATA
{
	uint16_t fr; /**< @brief Front Right from C185 (km/h x10). */
	uint16_t fl; /**< @brief Front Left from C185  (km/h x10). */
	uint16_t rr; /**< @brief Rear Right from C185  (km/h x10). */
	uint16_t rl; /**< @brief Rear Left from C185   (km/h x10). */
	uint16_t slip_ratio; /**< @brief Calculated Slip Ratio. */
} C185_Data;

typedef struct PM150_DATA
{
	uint16_t mc_rpm; /**< @brief Front Right from C185 (km/h x10). */
} PM150_Data;

/** @brief Holds processed data for the throttle sensors. */
typedef struct THROTTLE
{
	uint16_t tps1;    /**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t tps1_range_error;    /**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t tps2;    /**< @brief Processed position of throttle sensor 2 (0-100%). */
	uint16_t tps2_range_error;    /**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t pos;     /**< @brief Averaged throttle position (0-100%). */
} Throttle;

/** @brief Holds processed data for the brake pressure sensors. */
typedef struct BRAKE
{
	uint16_t front; /**< @brief Processed front brake pressure. */
	uint16_t front_range_error;    /**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t rear;  /**< @brief Processed rear brake pressure. */
	uint16_t rear_range_error;    /**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t avg;   /**< @brief Averaged brake pressure. */
} Brake;

/** @brief Holds processed data for the suspension potentiometer sensors. */
typedef struct SUSPOT
{
    uint16_t fl; /**< @brief Processed front left suspot distance. */
    uint16_t fr;  /**< @brief Processed front right suspot distance. */
} Suspot;


/** @brief Holds processed data for the wheel speed sensor potentiometer sensors. */
typedef struct WSS
{
	uint16_t fl;	/**< @brief Processed front left wheel speed (km/h) x100. */
	uint16_t fr;	/**< @brief Processed front right wheel speed (km/h) x100. */
} Wss;

uint16_t calculateTorque(Throttle *throttle, PM150_Data *pm150);
void updateFaultLEDs(Faults *faults);
void updateWarningLEDs(OrionData *orion);
uint8_t checkFault(Faults *faults);
void checkAppsBrakePlausibilityError(AppsBrakeErrors *apps_brake_errors, Throttle *throttle, Brake *brake);


#endif /* __VCU_STATE_H */

