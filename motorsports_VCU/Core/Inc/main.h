/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_BUTTON_PIN_Pin GPIO_PIN_2
#define START_BUTTON_PIN_GPIO_Port GPIOE
#define Regen_Switch_Pin GPIO_PIN_3
#define Regen_Switch_GPIO_Port GPIOE
#define TC_Switch_Pin GPIO_PIN_5
#define TC_Switch_GPIO_Port GPIOE
#define BSPD_FAULT_LED_Pin GPIO_PIN_6
#define BSPD_FAULT_LED_GPIO_Port GPIOE
#define WARNING_TEMP_OUT_Pin GPIO_PIN_13
#define WARNING_TEMP_OUT_GPIO_Port GPIOC
#define BMS_FAULT_LED_Pin GPIO_PIN_14
#define BMS_FAULT_LED_GPIO_Port GPIOC
#define PDOC_FAULT_LED_Pin GPIO_PIN_15
#define PDOC_FAULT_LED_GPIO_Port GPIOC
#define IMD_FAULT_LED_Pin GPIO_PIN_9
#define IMD_FAULT_LED_GPIO_Port GPIOF
#define WARNING_CHARGE_OUT_Pin GPIO_PIN_10
#define WARNING_CHARGE_OUT_GPIO_Port GPIOF
#define BRAKE_FRONT_IN_Pin GPIO_PIN_8
#define BRAKE_FRONT_IN_GPIO_Port GPIOE
#define BRAKE_REAR_IN_Pin GPIO_PIN_9
#define BRAKE_REAR_IN_GPIO_Port GPIOE
#define TPS1_IN_Pin GPIO_PIN_12
#define TPS1_IN_GPIO_Port GPIOE
#define TPS2_IN1_Pin GPIO_PIN_13
#define TPS2_IN1_GPIO_Port GPIOE
#define WHL_SPEED_FR_Pin GPIO_PIN_9
#define WHL_SPEED_FR_GPIO_Port GPIOA
#define WHL_SPEED_FL_Pin GPIO_PIN_11
#define WHL_SPEED_FL_GPIO_Port GPIOA
#define CHARGE_EN_IN_Pin GPIO_PIN_8
#define CHARGE_EN_IN_GPIO_Port GPIOB
#define BSPD_FAULT_IN_Pin GPIO_PIN_9
#define BSPD_FAULT_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
