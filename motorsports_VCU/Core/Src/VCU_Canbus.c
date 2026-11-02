/*
 * VCU_Canbus.c
 *
 *  Created on: 26 Mar 2026
 *      Author: NickM
 */


#include <VCU_Canbus.h>
#include <string.h>

uint8_t print_canbus = 0;


//--- CANBUS Messages ---//

//  Handler
CanbusRxData *_rxData;

void canInit(CanbusRxData *rxData) {

	_rxData = rxData;
}



void configFilters(CAN_HandleTypeDef *hcan, const uint16_t *idList, uint16_t length)
{
    CAN_FilterTypeDef f = {0};

    // ---- Bank 0: accept idA and idB into FIFO0 ----

    f.FilterMode           = CAN_FILTERMODE_IDLIST;      // LIST mode = exact IDs
    f.FilterScale          = CAN_FILTERSCALE_32BIT;      // one ID per 32-bit slot
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;

    uint32_t filterBank = 0;
    uint16_t idA = 0;
    uint16_t idB = 0;

    for (int i=0; i < length && filterBank <= 13; i+=2) {

        if (i+1 < length) {
        	idA = idList[i];
        	idB = idList[i+1];
        }
        else {
        	idA = idList[i];
        	idB = 0;
        }

        // Standard ID is placed in bits [31:21] => shift by 5 for High register
        f.FilterBank       = filterBank;                          // use bank 0
        f.FilterIdHigh     = ((idA & 0x7FF) << 5);
        f.FilterIdLow      = 0x0000; // IDE=0,RTR=0
        f.FilterMaskIdHigh = ((idB & 0x7FF) << 5);
        f.FilterMaskIdLow  = 0x0000;

        HAL_CAN_ConfigFilter(hcan, &f);

        filterBank++;
    }
}


void _send(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *TxHeader,
        const uint8_t TxData[], uint32_t *TxMailbox, MessagePriority msgPriority)
{
	uint8_t mailboxFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);

	// Empty lowest priority Mailbox
	if (mailboxFreeLevel == 0 && msgPriority == CRITICAL) {

		// Get Mailbox with lowest Priority
		uint32_t mailbox = (hcan->Instance->TSR & ( CAN_TSR_LOW0 | CAN_TSR_LOW1 | CAN_TSR_LOW2 ) ) >> CAN_TSR_LOW0_Pos;

		// Abort Mailbox with lowest Priority
		HAL_CAN_AbortTxRequest(hcan, mailbox);

		mailboxFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
	}

	if (mailboxFreeLevel > 0) {
		if (HAL_CAN_AddTxMessage(hcan, TxHeader, TxData, TxMailbox) != HAL_OK) {
		  /* Transmission request Error */
		  HAL_CAN_ErrorCallback(hcan);
		  Tx_Error_Handler(hcan);
		}
	}
}


void sendStartSw(CAN_HandleTypeDef *hcan, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_SWITCH_DLC];
	uint32_t              TxMailbox;

	// Switch
	TxHeader.StdId = Tx_SWITCH_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_SWITCH_DLC;

	TxData[0] = 1; // Low Byte

	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}


void sendThrottlePosition(CAN_HandleTypeDef *hcan, Throttle *throttle, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_THROTTLE_DLC];
	uint32_t              TxMailbox;

	// Throttle
	TxHeader.StdId = Tx_THROTTLE_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_THROTTLE_DLC;

	// Transmit Torque
	TxData[0] = (uint8_t)(throttle->pos & 0xff);
	TxData[1] = (uint8_t)(throttle->tps1 & 0xff);
	TxData[2] = (uint8_t)(throttle->tps2 & 0xff);
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}

void sendBrakePressure(CAN_HandleTypeDef *hcan, Brake *brake, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_BRAKE_DLC];
	uint32_t              TxMailbox;

	// Brake
	TxHeader.StdId = Tx_BRAKE_ID; // brake message ID
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_BRAKE_DLC;

	TxData[0] = ((brake->avg >> 8) & 0xff);
	TxData[1] = ((brake->avg ) & 0xff);
	TxData[2] = ((brake->rear >> 8) & 0xff);
	TxData[3] = ((brake->rear) & 0xff);
	TxData[4] = ((brake->front  >> 8) & 0xff);
	TxData[5] = ((brake->front) & 0xff);

	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}

void sendTorqueCommand(CAN_HandleTypeDef *hcan, uint16_t torque, uint8_t apps_brake_error, CarState carState, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_TORQUE_DLC];
	uint32_t              TxMailbox;

	// Torque
	TxHeader.StdId = Tx_TORQUE_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_TORQUE_DLC;

	if (apps_brake_error){
		torque = 0;
	}

	if (carState == RTD) {
		TxData[5] = 1; // Enable inverter
	}

	// Transmit Torque
	TxData[0] = (uint8_t)(torque & 0xFF); // Low Byte
	TxData[1] = (uint8_t)((torque >> 8) & 0xFF); // High Byte
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0; //Forward mode
	TxData[6] = 0;
	TxData[7] = 0;

	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}

void sendSensorData(CAN_HandleTypeDef *hcan, Wss *wss, Suspot *suspot, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_SENSOR_DATA_DLC];
	uint32_t              TxMailbox;

	// Sensor Data
	TxHeader.StdId = Tx_SENSOR_DATA_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_SENSOR_DATA_DLC;

	TxData[0] = (uint8_t) (suspot->fr & 0xFF);
	TxData[1] = (uint8_t) ((suspot->fr >> 8) & 0xFF);
	TxData[2] = (uint8_t) (suspot->fl & 0xFF);
	TxData[3] = (uint8_t) ((suspot->fl >> 8) & 0xFF);
	TxData[4] = (uint8_t) (wss->fr & 0xFF);
	TxData[5] = (uint8_t) ((wss->fr >> 8) & 0xFF);
	TxData[6] = (uint8_t) (wss->fl & 0xFF);
	TxData[7] = (uint8_t) ((wss->fl >> 8) & 0xFF);

	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}


void sendState(CAN_HandleTypeDef *hcan, AppsBrakeErrors *apps_brake_errors, CarState carState, MessagePriority msgPriority)
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t               TxData[Tx_STATE_DLC];
	uint32_t              TxMailbox;

	// State
	TxHeader.StdId = Tx_STATE_ID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = Tx_STATE_DLC;

	TxData[0] = (uint8_t) carState;
	TxData[1] = apps_brake_errors->appsBrakePlausibility;
	TxData[2] = apps_brake_errors->tpsRange;
	TxData[3] = apps_brake_errors->brakeRange;
	TxData[4] = apps_brake_errors->apps;
	TxData[5] = apps_brake_errors->appsBrake;
//	TxDataState[6] = faults.bspd;


	_send(hcan, &TxHeader, TxData, &TxMailbox, msgPriority);
}


void Rx_Error_Handler(CAN_HandleTypeDef *hcan)
{

}


void Tx_Error_Handler(CAN_HandleTypeDef *hcan)
{

}


void CAN_PrintLEC(CAN_HandleTypeDef *hcan)
{
    uint32_t lec = (hcan->Instance->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;

    switch (lec) {
        case 0x0: printf("  LEC: No Error\n"); break;
        case 0x1: printf("  LEC: Stuff Error\n"); break;
        case 0x2: printf("  LEC: Form Error\n"); break;
        case 0x3: printf("  LEC: ACK Error\n"); break;
        case 0x4: printf("  LEC: Bit Recessive Error\n"); break;
        case 0x5: printf("  LEC: Bit Dominant Error\n"); break;
        case 0x6: printf("  LEC: CRC Error\n"); break;
        case 0x7: printf("  LEC: (No change / cleared by SW)\n"); break;
        default:  printf("  LEC: Unknown (%lu)\n", lec); break;
    }
}


void print_CANBUS_RxData(CAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[]) {

	printf("[%d] -  ", (int) RxHeader->StdId);
	printf("RxData: ");
	for (int i = 0; i < RxHeader->DLC; i++){
		printf(" %d,", RxData[i]);
	}
	printf("\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t               RxData[8];

  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	printf("HAL_CAN_GetRxMessage() Failed\n");
	HAL_CAN_ErrorCallback(hcan);
	Rx_Error_Handler(hcan);
  }


  if (RxHeader.StdId == Rx_C185_0_ID) {
//	  printf("Received C185_0\n");
	  _rxData->c185.fr = (((uint16_t) RxData[4]) << 8 | (uint16_t) RxData[5]);
	  _rxData->c185.fl = (((uint16_t) RxData[6])  << 8 | (uint16_t) RxData[7]);
	  _rxData->c185.rr = (((uint16_t) RxData[0]) << 8 | (uint16_t) RxData[1]);
	  _rxData->c185.rl = (((uint16_t) RxData[2]) << 8 | (uint16_t) RxData[3]);
  }

  if (RxHeader.StdId == Rx_C185_1_ID) {
	  _rxData->c185.slip_ratio = ((uint16_t) RxData[0] << 8 | (uint16_t) RxData[1]);
  }

  // PDM Message0
  if (RxHeader.StdId == Rx_PDM_0_ID) {
	  _rxData->faults.ams = RxData[0];
	  _rxData->faults.pdoc = RxData[1];
	  _rxData->faults.bspd = RxData[2];
	  _rxData->faults.imd = RxData[3];
	  _rxData->pdm.standby = RxData[4];
	  _rxData->pdm.precharged = RxData[5];
	  _rxData->pdm.rtd = RxData[6];
	  _rxData->pdm.drive_enable = RxData[7];
  }

  // Accumulator Message
  if (RxHeader.StdId == Rx_PDM_1_ID) {
      // Bytes 0–1: Pack Summed Voltage, 0.1 V units, big-endian
      uint16_t raw_pack_sum_0  = ((uint16_t)RxData[0] << 8);
      uint16_t raw_pack_sum_1  = (uint16_t)RxData[1];
      uint16_t raw_sum         = raw_pack_sum_0 | raw_pack_sum_1;
      float    pack_voltage_V  = raw_sum * 0.01f;   // convert to volts

      // Byte 2: High Temperature, signed, 1 °C units
      int8_t  raw_high_temp  = (int8_t)RxData[2];
      int16_t high_temp_C    = (int16_t)raw_high_temp;

      // Byte 3: Low Cell Voltage
      uint16_t low_cell_voltage_0 = ((uint16_t)RxData[3] << 8);
      uint16_t low_cell_voltage_1 = (uint16_t)RxData[4];
      uint16_t low_cell_voltage   = low_cell_voltage_0 | low_cell_voltage_1;

      // store somewhere global / struct if you want
      _rxData->orion.pack_voltage      = pack_voltage_V;
      _rxData->orion.cell_temperature  = high_temp_C;
      _rxData->orion.low_cell 		 = (float)low_cell_voltage * 0.0001f;
//        orion_status.pack_voltage      = 500;
//        orion_status.cell_temperature  = 51;
//        orion_status.low_cell 		   = 4;
  }

  // MC Message for Motor RPM
  if (RxHeader.StdId == Rx_PDM_2_ID) {
	  // Motor Speed (bytes 4-5)
	  int16_t mc_rpm_raw = (int16_t)(RxData[4] | (RxData[5] << 8));
	  _rxData->pm150.mc_rpm = mc_rpm_raw * (-1);
  }

  if (print_canbus){
	  print_CANBUS_RxData(&RxHeader, RxData);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);

    printf("CAN ERROR: 0x%08lX\n", err);

    if (err & HAL_CAN_ERROR_EWG) printf("  Error Warning (EWG)\n");
    if (err & HAL_CAN_ERROR_EPV) printf("  Error Passive (EPV)\n");
    if (err & HAL_CAN_ERROR_BOF) printf("  Bus-Off (BOF)\n");

    if (err & HAL_CAN_ERROR_STF) printf("  Stuff error\n");
    if (err & HAL_CAN_ERROR_FOR) printf("  Form error\n");
    if (err & HAL_CAN_ERROR_ACK) printf("  ACK error (no ACK)\n");
    if (err & HAL_CAN_ERROR_BR)  printf("  Bit recessive error\n");
    if (err & HAL_CAN_ERROR_BD)  printf("  Bit dominant error\n");
    if (err & HAL_CAN_ERROR_CRC) printf("  CRC error\n");

    // Optional: RX FIFO overruns if your HAL exposes these on F3
#ifdef HAL_CAN_ERROR_FOV0
    if (err & HAL_CAN_ERROR_FOV0) { printf("  RX FIFO0 overrun\n"); __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0); }
#endif
#ifdef HAL_CAN_ERROR_FOV1
    if (err & HAL_CAN_ERROR_FOV1) { printf("  RX FIFO1 overrun\n"); __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1); }
#endif

    CAN_PrintLEC(hcan);

    // ---- Clear error sources (F3 way) ----
    // 1) Clear the general error interrupt flag
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);

    // 2) Clear LEC by writing 0b111 to ESR[LEC] (F3: no CAN_FLAG_LEC macro)
    //    Setting all three LEC bits to 1 resets them to "no error".
    hcan->Instance->ESR |= CAN_ESR_LEC;

    // 3) Optional: handle Bus-Off recovery if AutoBusOff is DISABLE
    if (err & HAL_CAN_ERROR_BOF) {
        HAL_CAN_Stop(hcan);
        // Leave/enter init mode to force recovery (simple approach)
        // (HAL_CAN_Start will bring it back to normal mode)
        if (HAL_CAN_Start(hcan) == HAL_OK) {
            printf("  Bus-off recovery: restart OK\n");
        } else {
            printf("  Bus-off recovery: restart FAILED\n");
        }
    }
}





