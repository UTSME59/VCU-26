#ifndef __VCU_CANBUS_H
#define __VCU_CANBUS_H


#include <VCU_State.h>
#include <stdint.h>
#include "stm32f3xx_hal.h"

typedef enum MESSAGEPRIORITY
{
  CRITICAL,
  NOT_CRITICAL
} MessagePriority;

//--- CANBUS Messages ---//
typedef struct CANBUSRXDATA {

	// Rx Variables
	Faults faults;
	PDM_Status pdm;
	OrionData orion;
	C185_Data c185;
	PM150_Data pm150;

} CanbusRxData;


// CANBUS Receive IDs
#define Rx_PDM_0_ID 0x520
#define Rx_PDM_1_ID 0x531
#define Rx_PDM_2_ID 0x0B0
#define Rx_PDM_3_ID 0x534
#define Rx_C185_0_ID 0x700 // Wheel Speeds
#define Rx_C185_1_ID 0x701 // Slip Ratio, etc.

#define CAN_ID_LIST_LENGTH 6
#define CAN_ID_LIST Rx_PDM_0_ID, Rx_PDM_1_ID, Rx_PDM_2_ID, Rx_PDM_3_ID, Rx_C185_0_ID, Rx_C185_1_ID


// CANBUS Transmit IDs
#define Tx_TORQUE_ID 0x0C0
#define Tx_TORQUE_DLC 8

#define Tx_THROTTLE_ID 0x244
#define Tx_THROTTLE_DLC 3

#define Tx_BRAKE_ID 0x245
#define Tx_BRAKE_DLC 8

#define Tx_SENSOR_DATA_ID 0x246
#define Tx_SENSOR_DATA_DLC 8

#define Tx_SWITCH_ID 0x650
#define Tx_SWITCH_DLC 2

#define Tx_STATE_ID 0x124
#define Tx_STATE_DLC 8

//uint8_t canIdList[CAN_ID_LIST_LENGTH] = {CAN_ID_LIST};

// Configuration
void canInit(CanbusRxData *rxData);
void configFilters(CAN_HandleTypeDef *hcan, const uint16_t *idList, uint16_t length);

// Trensmit Function
void sendTorqueCommand(CAN_HandleTypeDef *hcan, uint16_t torque, uint8_t apps_brake_error, CarState carState, MessagePriority msgPriority);
void sendThrottlePosition(CAN_HandleTypeDef *hcan, Throttle *throttle, MessagePriority msgPriority);
void sendBrakePressure(CAN_HandleTypeDef *hcan, Brake *brake, CarState carState, MessagePriority msgPriority);
void sendSensorData(CAN_HandleTypeDef *hcan, Wss *wss, MessagePriority msgPriority);
void sendStartSw(CAN_HandleTypeDef *hcan, MessagePriority msgPriority);
void sendState(CAN_HandleTypeDef *hcan, AppsBrakeErrors *apps_brake_errors, CarState carState, MessagePriority msgPriority);
void CAN_PrintLEC(CAN_HandleTypeDef *hcan);

// Error Handlers
void Rx_Error_Handler(CAN_HandleTypeDef *hcan);
void Tx_Error_Handler(CAN_HandleTypeDef *hcan);

// Receive Message Print
void print_CANBUS_RxData(CAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[]);


// Callback Functions
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#endif /* __VCU_CANBUS_H */
