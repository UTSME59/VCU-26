	/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2025 STMicroelectronics.
	  * All rights reserved.
	  *
	  * This software is licensed under terms that can be found in the LICENSE file
	  * in the root directory of this software component.
	  * If no LICENSE file comes with this software, it is provided AS-IS.
	  *
	  ******************************************************************************
	  */
	/* USER CODE END Header */
	/* Includes ------------------------------------------------------------------*/
	#include "main.h"

	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */
	#include <stdint.h>
	#include <stdio.h>
	#include <math.h>
	#include <stdlib.h>
	#include <VCU_State.h>
	#include <VCU_Canbus.h>
	#include <VCU_ADC.h>
	#include <VCU_Timer.h>

	/* USER CODE END Includes */

	/* Private typedef -----------------------------------------------------------*/
	/* USER CODE BEGIN PTD */

	/* USER CODE END PTD */

	/* Private define ------------------------------------------------------------*/
	/* USER CODE BEGIN PD */

	/* USER CODE END PD */

	/* Private macro -------------------------------------------------------------*/
	/* USER CODE BEGIN PM */

	/* USER CODE END PM */

	/* Private variables ---------------------------------------------------------*/
	CAN_HandleTypeDef hcan;

	SDADC_HandleTypeDef hsdadc1;
	SDADC_HandleTypeDef hsdadc2;

	TIM_HandleTypeDef htim2;
	TIM_HandleTypeDef htim3;
	TIM_HandleTypeDef htim4;
	TIM_HandleTypeDef htim5;
	TIM_HandleTypeDef htim6;
	TIM_HandleTypeDef htim7;

	/* USER CODE BEGIN PV */
	Throttle throttle = {0};
	Brake brake = {0};
	Wss wss = {0};
	AppsBrakeErrors apps_brake_errors = {0};

	CanbusRxData rxData = {0};

	AdcHandler hadc = {
		&hsdadc1,
		&hsdadc2,
		0,0,0,0,0

	};

	volatile uint8_t regen_sw = 0;
	volatile uint8_t tc_sw = 0;
	volatile uint8_t start_btn = 0;

	TimerHandler htimer = {
		&htim2,
		&htim3,
		&htim4,
		&htim5,
		&htim6,
		&htim7,
		0,0,0,0
	  };

	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_CAN_Init(void);
	static void MX_TIM2_Init(void);
	static void MX_SDADC1_Init(void);
	static void MX_SDADC2_Init(void);
	static void MX_TIM4_Init(void);
	static void MX_TIM5_Init(void);
	static void MX_TIM6_Init(void);
	static void MX_TIM3_Init(void);
	static void MX_TIM7_Init(void);
	/* USER CODE BEGIN PFP */


	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */

	// For printouts - enable printf to SWO (ST-Link V3)
	int _write(int file, char *ptr, int len)
	{
	  // Send data to SWO (Serial Wire Output) via ITM
	  for (int i = 0; i < len; i++) {
		ITM_SendChar(ptr[i]);
	  }
	  return len;
	}

	// for the swv trace
	void ITM_SendWord(uint8_t port, uint32_t value) {
		if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port))) {
			while (ITM->PORT[port].u32 == 0);
			ITM->PORT[port].u32 = value;
		}
	}


	/* USER CODE END 0 */

	/**
	  * @brief  The application entry point.
	  * @retval int
	  */
	int main(void)
	{

	  /* USER CODE BEGIN 1 */
		CarState carState = IDLE;

	  /* USER CODE END 1 */

	  /* MCU Configuration--------------------------------------------------------*/

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	  HAL_Init();

	  /* USER CODE BEGIN Init */

	  /* USER CODE END Init */

	  /* Configure the system clock */
	  SystemClock_Config();

	  /* USER CODE BEGIN SysInit */
	  canInit(&rxData);
	  adcInit(&hadc);
	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_CAN_Init();
	  MX_TIM2_Init();
	  MX_SDADC1_Init();
	  MX_SDADC2_Init();
	  MX_TIM4_Init();
	  MX_TIM5_Init();
	  MX_TIM6_Init();
	  MX_TIM3_Init();
	  MX_TIM7_Init();
	  /* USER CODE BEGIN 2 */
	  // Initialize SWO for debug output
	  // Enable TRCENA bit in Debug Exception and Monitor Control Register
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  // Enable stimulus port 0 in ITM Trace Enable Register
	  ITM->TER = 0x1;
	  // Configure ITM Control Register
	  ITM->TCR = 0x1000D;

	  uint16_t torque = 0;

	  // Sensors
	  /*Throttle throttle = {0};
	  Brake brake = {0};
	  Wss wss = {0};
	  AppsBrakeErrors apps_brake_errors = {0};*/

	  // CANBUS Receive Messages
	  //CanbusRxData rxData = {0};
	  // Init Canbus Messages
	  canInit(&rxData);

	  // ADC Config
	  /*AdcHandler hadc = {
		&hsdadc1,
		&hsdadc2,
		0,0,0,0,0,0
	  };*/

	  adcInit(&hadc);


	  // Timer Config
	  /*TimerHandler htimer = {
		&htim2,
		&htim3,
		&htim4,
		&htim5,
		&htim6,
		&htim7,
		0,0,0,0
	  };*/

	  timerInit(&htimer);
	  timerStart();


	  // Arbitrary delay
	  HAL_Delay(300);

	  for (int i = 0; i < 20; i++) {			//added to fix the brake range error on startup, lets 4 sample ADC filter settle before range check
		  readADC(&hadc);
		  HAL_Delay(5);
	  }
	  htimer.tim3_flag = 0;
	  htimer.tim4_flag = 0;
	  htimer.tim6_flag = 0;

	  printf("FINISHED INITIALISATION\n");
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */

	  // Track Canbus Tx Messages (Used to Stagger Transmissions)
	  uint32_t critical_counter = 0;
	  const uint32_t total_critical = 2;
	  uint32_t non_critical_counter = 0;
	  const uint32_t total_non_critical = 3;

	  while (1)
	  {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		switch (carState)
		{
		  case IDLE:
			updateFaultLEDs(&rxData.faults);

			if (rxData.pdm.precharged) {
			  carState = PRECHARGED;
			  printf("\tNext State: PRECHARGED\n");
			}
			break;

		  case PRECHARGED:
			if (checkFault(&rxData.faults)) {
				  carState = FAULT;
				  printf("\tNext State: FAULT\n");
			}
			updateFaultLEDs(&rxData.faults);

			// Poll Start Button
			if (!HAL_GPIO_ReadPin(START_BUTTON_PIN_GPIO_Port, START_BUTTON_PIN_Pin) && brake.avg > BRAKE_PRESSURE_START_THRESHOLD)
			{
			  sendStartSw(&hcan, NOT_CRITICAL);
			  // NextState = RTD
			  carState = RTD;
			  printf("\tNext State: RTD\n");
			}
			break;

		  case RTD:
			// Send Torque to MC
			if (checkFault(&rxData.faults)) {
				  carState = FAULT;
				  printf("\tNext State: FAULT\n");
			}
			break;

		  case FAULT:
			updateFaultLEDs(&rxData.faults);

			break;

		  default:
			  carState = FAULT;
			  break;
		}


		// Timed Operations

		/*
		 *
		 * ----- READ ADC -----
		 *
		 *  f=200Hz
		 */
		if (htimer.tim3_flag) {
			readADC(&hadc);

			updateFaultLEDs(&rxData.faults);
			updateWarningLEDs(&rxData.orion);

			htimer.tim3_flag = 0; // reset

			regen_sw = (HAL_GPIO_ReadPin(Regen_Switch_GPIO_Port, Regen_Switch_Pin) == GPIO_PIN_SET);
			tc_sw    = (HAL_GPIO_ReadPin(TC_Switch_GPIO_Port,    TC_Switch_Pin)    == GPIO_PIN_SET);
			start_btn = (HAL_GPIO_ReadPin(START_BUTTON_PIN_GPIO_Port, START_BUTTON_PIN_Pin) == GPIO_PIN_RESET);
		}

		/*
		 *
		 * ----- CRITICAL MESSAGES -----
		 *
		 *  f=40Hz
		 *  Stagger message transmission so each message is 40Hz / 2 = 20Hz
		*/
		if (htimer.tim4_flag) {
			// Get Throttle Position from ADC Measurement

			// Update Brake and Throttle Values
			updateThrottlePosition(&hadc,&throttle);
			updateBrakePressure(&hadc,&brake);
			checkAppsBrakePlausibilityError(&apps_brake_errors, &throttle, &brake);


			// --- ALTERNATE MESSAGE TRANSMISSION TO AVOID MAILBOX OVERFLOW --- //

			// Send Throttle
			if(critical_counter % total_critical == 0) {

				// Calculate Torque Value
				torque = calculateTorque(&throttle, &rxData.pm150);
				sendTorqueCommand(&hcan, torque, apps_brake_errors.appsBrakePlausibility, carState, CRITICAL);
			}

			// Send Brake
			else if(critical_counter % total_critical == 1) {
				sendBrakePressure(&hcan, &brake, carState, CRITICAL);
			}

			critical_counter++;
			htimer.tim4_flag = 0; // reset
		}

		/*
		 * ----- NON-CRITICAL MESSAGES -----
		 *
		 *  f=30Hz
		 *  Stagger message transmission so each message is 30Hz / 3 = 10Hz
		 */
		if (htimer.tim6_flag) {

			// --- ALTERNATE MESSAGE TRANSMISSION TO AVOID MAILBOX OVERFLOW --- //

			if(non_critical_counter % total_non_critical == 0) {
				updateWSS(&wss);
				sendSensorData(&hcan, &wss, NOT_CRITICAL);
			}

			else if(non_critical_counter % total_non_critical == 1) {
				sendState(&hcan, &apps_brake_errors, carState, NOT_CRITICAL);
			}

			else if (non_critical_counter % total_non_critical == 2) {
				sendThrottlePosition(&hcan, &throttle, NOT_CRITICAL);
			}

			non_critical_counter++;
			htimer.tim6_flag = 0; // reset
		}

		// Non-Critical Messages - 1Hz
		if (htimer.tim7_flag) {

	//    	if (counter++ % 2 == 0) {
	//    		readADC_test(&hsdadc1, &hsdadc2);
	//    	}

	//    	void print_ADC(void);

			htimer.tim7_flag = 0;
		}

		//--- END CAN TEST ---///
	  }
	  /* USER CODE END 3 */
	}

	/**
	  * @brief System Clock Configuration
	  * @retval None
	  */
	void SystemClock_Config(void)
	{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDADC;
	  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV8;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
		Error_Handler();
	  }
	  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
	  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG2);
	}

	/**
	  * @brief CAN Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_CAN_Init(void)
	{

	  /* USER CODE BEGIN CAN_Init 0 */
	  printf("CAN INIT STARTTING\n");

	  /* USER CODE END CAN_Init 0 */

	  /* USER CODE BEGIN CAN_Init 1 */

	  /* USER CODE END CAN_Init 1 */
	  hcan.Instance = CAN;
	  hcan.Init.Prescaler = 2;
	  hcan.Init.Mode = CAN_MODE_NORMAL;
	  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
	  hcan.Init.TimeTriggeredMode = DISABLE;
	  hcan.Init.AutoBusOff = DISABLE;
	  hcan.Init.AutoWakeUp = DISABLE;
	  hcan.Init.AutoRetransmission = DISABLE;
	  hcan.Init.ReceiveFifoLocked = DISABLE;
	  hcan.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN CAN_Init 2 */
	  // Configure the CAN filter
	//  CAN_FilterTypeDef sFilterConfig;
	//  uint32_t id1 = (CANID_PDM_0 << 21);
	//  uint32_t id2 = (CANID_PDM_2 << 21);
	//  sFilterConfig.FilterBank = 0;
	//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	//  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//  sFilterConfig.FilterIdHigh = (id1 >> 16);
	//  sFilterConfig.FilterIdLow = (id1 & 0xFFFF);
	//  sFilterConfig.FilterMaskIdHigh = (id2 >> 16);
	//  sFilterConfig.FilterMaskIdLow = (id2 & 0xFFFF);
	//  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	//  sFilterConfig.FilterActivation = ENABLE;
	//  sFilterConfig.SlaveStartFilterBank = 0;
	//
	//  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
	//	  printf("HAL CAN CONFIG FILTER FAILED\n");
	//      Error_Handler();
	//  }

	  /* USER CODE BEGIN CAN_Init 2 */


	  // Whitelist only these 4 IDs (standard 11-bit)

	  uint16_t canIdList[CAN_ID_LIST_LENGTH] = {CAN_ID_LIST};
	  configFilters(&hcan, canIdList, CAN_ID_LIST_LENGTH);

	  // Start CAN and enable RX interrupt (as you already do)
	  if (HAL_CAN_Start(&hcan) != HAL_OK) {
		  printf("CAN START FAILED\n");
		  Error_Handler();
	  }
	  if (HAL_CAN_ActivateNotification(
			  &hcan,
			  CAN_IT_RX_FIFO0_MSG_PENDING      |
			  CAN_IT_ERROR_WARNING             |
			  CAN_IT_ERROR_PASSIVE             |
			  CAN_IT_BUSOFF                    |
			  CAN_IT_LAST_ERROR_CODE           |
			  CAN_IT_ERROR)                    != HAL_OK) {
		  printf("CAN ACTIVATE (error IRQs) FAILED\n");
		  Error_Handler();
	  }
	  /* USER CODE END CAN_Init 2 */

	}

	/**
	  * @brief SDADC1 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_SDADC1_Init(void)
	{

	  /* USER CODE BEGIN SDADC1_Init 0 */

	  /* USER CODE END SDADC1_Init 0 */

	  SDADC_ConfParamTypeDef ConfParamStruct = {0};

	  /* USER CODE BEGIN SDADC1_Init 1 */

	  /* USER CODE END SDADC1_Init 1 */

	  /** Configure the SDADC low power mode, fast conversion mode,
	  slow clock mode and SDADC1 reference voltage
	  */
	  hsdadc1.Instance = SDADC1;
	  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
	  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
	  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
	  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
	  hsdadc1.InjectedTrigger = SDADC_SOFTWARE_TRIGGER;
	  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Mode
	  */
	  if (HAL_SDADC_SelectInjectedDelay(&hsdadc1, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_SDADC_InjectedConfigChannel(&hsdadc1, SDADC_CHANNEL_1|SDADC_CHANNEL_7
								  |SDADC_CHANNEL_8, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Set parameters for SDADC configuration 0 Register
	  */
	  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
	  ConfParamStruct.Gain = SDADC_GAIN_1;
	  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
	  ConfParamStruct.Offset = 0;
	  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Regular Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_1, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_7, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN SDADC1_Init 2 */
	  if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
	  {
		/* An error occurs during the starting phase of the calibration */
		  Error_Handler();
	  }

	  /* Pool for the end of calibration */
	  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, 1000) != HAL_OK)
	  {
		/* An error occurs while waiting for the end of the calibration */
		  Error_Handler();
	  }
	  /* USER CODE END SDADC1_Init 2 */

	}

	/**
	  * @brief SDADC2 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_SDADC2_Init(void)
	{

	  /* USER CODE BEGIN SDADC2_Init 0 */

	  /* USER CODE END SDADC2_Init 0 */

	  SDADC_ConfParamTypeDef ConfParamStruct = {0};

	  /* USER CODE BEGIN SDADC2_Init 1 */

	  /* USER CODE END SDADC2_Init 1 */

	  /** Configure the SDADC low power mode, fast conversion mode,
	  slow clock mode and SDADC1 reference voltage
	  */
	  hsdadc2.Instance = SDADC2;
	  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
	  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
	  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
	  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_EXT;
	  hsdadc2.InjectedTrigger = SDADC_SOFTWARE_TRIGGER;
	  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Mode
	  */
	  if (HAL_SDADC_SelectInjectedDelay(&hsdadc2, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_SDADC_InjectedConfigChannel(&hsdadc2, SDADC_CHANNEL_3|SDADC_CHANNEL_2, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Set parameters for SDADC configuration 0 Register
	  */
	  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
	  ConfParamStruct.Gain = SDADC_GAIN_1;
	  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
	  ConfParamStruct.Offset = 0;
	  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_3, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the Injected Channel
	  */
	  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN SDADC2_Init 2 */
	  if (HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
	  {
		/* An error occurs during the starting phase of the calibration */
		  Error_Handler();
	  }

	  /* Pool for the end of calibration */
	  if (HAL_SDADC_PollForCalibEvent(&hsdadc2, 1000) != HAL_OK)
	  {
		/* An error occurs while waiting for the end of the calibration */
		  Error_Handler();
	  }
	  /* USER CODE END SDADC2_Init 2 */

	}

	/**
	  * @brief TIM2 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM2_Init(void)
	{

	  /* USER CODE BEGIN TIM2_Init 0 */

	  /* USER CODE END TIM2_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_IC_InitTypeDef sConfigIC = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 7;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = 9999999;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	  sConfigIC.ICFilter = 0;
	  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM2_Init 2 */

	  /* USER CODE END TIM2_Init 2 */

	}

	/**
	  * @brief TIM3 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM3_Init(void)
	{

	  /* USER CODE BEGIN TIM3_Init 0 */

	  /* USER CODE END TIM3_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM3_Init 1 */

	  /* USER CODE END TIM3_Init 1 */
	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 7;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 4999;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM3_Init 2 */

	  /* USER CODE END TIM3_Init 2 */

	}

	/**
	  * @brief TIM4 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM4_Init(void)
	{

	  /* USER CODE BEGIN TIM4_Init 0 */

	  /* USER CODE END TIM4_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM4_Init 1 */

	  /* USER CODE END TIM4_Init 1 */
	  htim4.Instance = TIM4;
	  htim4.Init.Prescaler = 7;
	  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim4.Init.Period = 24999;
	  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM4_Init 2 */

	  /* USER CODE END TIM4_Init 2 */

	}

	/**
	  * @brief TIM5 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM5_Init(void)
	{

	  /* USER CODE BEGIN TIM5_Init 0 */

	  /* USER CODE END TIM5_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_IC_InitTypeDef sConfigIC = {0};

	  /* USER CODE BEGIN TIM5_Init 1 */

	  /* USER CODE END TIM5_Init 1 */
	  htim5.Instance = TIM5;
	  htim5.Init.Prescaler = 7;
	  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim5.Init.Period = 9999999;
	  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	  sConfigIC.ICFilter = 0;
	  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM5_Init 2 */


	  /* USER CODE END TIM5_Init 2 */

	}

	/**
	  * @brief TIM6 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM6_Init(void)
	{

	  /* USER CODE BEGIN TIM6_Init 0 */

	  /* USER CODE END TIM6_Init 0 */

	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM6_Init 1 */

	  /* USER CODE END TIM6_Init 1 */
	  htim6.Instance = TIM6;
	  htim6.Init.Prescaler = 7;
	  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim6.Init.Period = 33332;
	  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM6_Init 2 */

	  /* USER CODE END TIM6_Init 2 */

	}

	/**
	  * @brief TIM7 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_TIM7_Init(void)
	{

	  /* USER CODE BEGIN TIM7_Init 0 */

	  /* USER CODE END TIM7_Init 0 */

	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM7_Init 1 */

	  /* USER CODE END TIM7_Init 1 */
	  htim7.Instance = TIM7;
	  htim7.Init.Prescaler = 799;
	  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim7.Init.Period = 9999;
	  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN TIM7_Init 2 */

	  /* USER CODE END TIM7_Init 2 */

	}

	/**
	  * @brief GPIO Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_GPIO_Init(void)
	{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /* USER CODE BEGIN MX_GPIO_Init_1 */

	  /* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(BSPD_FAULT_LED_GPIO_Port, BSPD_FAULT_LED_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, WARNING_TEMP_OUT_Pin|BMS_FAULT_LED_Pin|PDOC_FAULT_LED_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOF, IMD_FAULT_LED_Pin|WARNING_CHARGE_OUT_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : Regen_Switch_Pin TC_Switch_Pin */
	  GPIO_InitStruct.Pin = Regen_Switch_Pin|TC_Switch_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pin : BSPD_FAULT_LED_Pin */
	  GPIO_InitStruct.Pin = BSPD_FAULT_LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(BSPD_FAULT_LED_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : WARNING_TEMP_OUT_Pin BMS_FAULT_LED_Pin PDOC_FAULT_LED_Pin */
	  GPIO_InitStruct.Pin = WARNING_TEMP_OUT_Pin|BMS_FAULT_LED_Pin|PDOC_FAULT_LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : IMD_FAULT_LED_Pin WARNING_CHARGE_OUT_Pin */
	  GPIO_InitStruct.Pin = IMD_FAULT_LED_Pin|WARNING_CHARGE_OUT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pins : START_BUTTON_PIN_Pin BSPD_FAULT_IN_Pin */
	  GPIO_InitStruct.Pin = START_BUTTON_PIN_Pin|BSPD_FAULT_IN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* USER CODE BEGIN MX_GPIO_Init_2 */

	  /* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */



	/* USER CODE END 4 */

	/**
	  * @brief  This function is executed in case of error occurrence.
	  * @retval None
	  */
	void Error_Handler(void)
	{
	  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */

	  __disable_irq();
	  while (1)
	  {
	  }
	  /* USER CODE END Error_Handler_Debug */
	}
	#ifdef USE_FULL_ASSERT
	/**
	  * @brief  Reports the name of the source file and the source line number
	  *         where the assert_param error has occurred.
	  * @param  file: pointer to the source file name
	  * @param  line: assert_param error line source number
	  * @retval None
	  */
	void assert_failed(uint8_t *file, uint32_t line)
	{
	  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	  /* USER CODE END 6 */
	}
	#endif /* USE_FULL_ASSERT */
