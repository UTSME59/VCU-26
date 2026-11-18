#ifndef __VCU_STATE_H
#define __VCU_STATE_H


#include <stdint.h>

#define TPS_POS_MIN 0          					/**< @brief The minimum mapped value for Throttle Position Sensor. */
#define TPS_POS_MAX 100       			    	/**< @brief The maximum mapped value for Throttle Position Sensor. */


// -- TORQUE CONFIG -- //
#define TORQUE_MAX 						2200    /**< @brief The maximum torque command value to be sent. */
#define BRAKE_PRESSURE_START_THRESHOLD  500 	/**< @brief Pressure threshold to trigger RTD. */
#define BRAKE_PRESSURE_APPS_THRESHOLD   500  	/**< @brief Pressure threshold to trigger APPS/Brake plausibility check. */

#define POWER_TARGET_KW					75
#define POWER_CURRENT_SIGN				1	    /**< @brief The BMS current direction (1: discharge positive). */
#define POWER_KP						0.01
#define POWER_KI						0.1
#define POWER_DATA_TIMEOUT_MS			40
#define POWER_CONTROL_MAX_GAP_MS 		100

#define TC_ENABLE               		1     /* 0 = disabled, 1 = use dash switch */
#define TC_SLIP_REF_PCT         		3
#define TC_MIN_SPEED_KMH       			10
#define TC_MAX_SPEED_KMH      			120
#define TC_FRONT_GAP_KMH        		1
#define TC_FRONT_GAP_FRACTION   		0.20
#define TC_REAR_MIN_FRACTION    		0.50

#define TC_KP                   		0.02 /* Reduction fraction per percentage point */
#define TC_KI                   		0.10 /* Reduction fraction per percentage point-second */
#define TC_RELEASE_PER_S        		1.0  /* Restore up to 100% of available torque per second */
#define TC_MAX_STEP_MS         			75

#define WSS_ALL_FRESH           0x0FU

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

	float pack_current_a;
	float pack_instant_voltage_v;
	uint32_t power_received_ms;
	uint32_t power_sequence;
	uint32_t power_received;
} OrionData;

typedef struct C185_DATA
{
	uint16_t fr; 			/**< @brief Front Right from C185 (km/h x10). */
	uint16_t fl; 			/**< @brief Front Left from C185  (km/h x10). */
	uint16_t rr; 			/**< @brief Rear Right from C185  (km/h x10). */
	uint16_t rl; 			/**< @brief Rear Left from C185   (km/h x10). */
	uint16_t slip_ratio; 	/**< @brief Calculated Slip Ratio. */
} C185_Data;

typedef struct PM150_DATA
{
	int32_t mc_rpm; 		/**< @brief Front Right from C185 (km/h x10). */
} PM150_Data;

/** @brief Holds processed data for the throttle sensors. */
typedef struct THROTTLE
{
	uint16_t tps1;    				/**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t tps1_range_error;    	/**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t tps2;    				/**< @brief Processed position of throttle sensor 2 (0-100%). */
	uint16_t tps2_range_error;    	/**< @brief Processed position of throttle sensor 1 (0-100%). */
	uint16_t pos;     				/**< @brief Averaged throttle position (0-100%). */
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

typedef struct TORQUE
{
    /* All torque fields use 0.1 Nm units. */
    uint16_t driver_request;
    uint16_t power_limit;
    uint16_t traction_limit;
    uint16_t command;
} Torque;

typedef struct POWERLIMITER
{
    float target_kw;
    float kp;
    float ki;

    float integral;
    float measured_kw;
    float reduction;

    float p_term;
    float i_term;

    uint32_t previous_sequence;
    uint32_t previous_sample_ms;

    uint8_t active;
    uint8_t data_valid;
} PowerLimiter;

typedef struct TRACTIONLIMITER
{
	 float kp;
	 float ki;
	 float integral;
	 float reduction;
	 float slip_pct;
	 uint32_t last_update_ms;
	 uint8_t active;
	 uint8_t data_valid;
} TractionLimiter;

/** @brief Holds processed data for the wheel speed sensor potentiometer sensors. */
typedef struct WSS
{
	uint16_t fl;	/**< @brief Processed front left wheel speed (km/h) x100. */
	uint16_t fr;	/**< @brief Processed front right wheel speed (km/h) x100. */
	uint16_t rl;
	uint16_t rr;
	uint8_t fresh_mask;
} Wss;

void updateTorque(Torque *torque, Throttle *throttle, PM150_Data *motor, OrionData *battery, PowerLimiter *limiter, uint8_t drive_allowed, uint32_t now_ms);
void updateTractionLimiter(Torque *torque, TractionLimiter *tc, Wss *wss, uint8_t in_rtd, uint8_t enabled, uint32_t now_ms);
void updateFaultLEDs(Faults *faults);
void updateWarningLEDs(OrionData *orion);
uint8_t checkFault(Faults *faults);
void checkAppsBrakePlausibilityError(AppsBrakeErrors *apps_brake_errors, Throttle *throttle, Brake *brake);


#endif /* __VCU_STATE_H */

