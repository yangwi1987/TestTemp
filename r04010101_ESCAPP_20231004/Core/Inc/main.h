/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_VP_Pin GPIO_PIN_2
#define PWM_VP_GPIO_Port GPIOE
#define PWM_WP_Pin GPIO_PIN_3
#define PWM_WP_GPIO_Port GPIOE
#define PWM_VN_Pin GPIO_PIN_4
#define PWM_VN_GPIO_Port GPIOE
#define PWM_WN_Pin GPIO_PIN_5
#define PWM_WN_GPIO_Port GPIOE
#define HWOCP_Pin GPIO_PIN_9
#define HWOCP_GPIO_Port GPIOF
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define P13V_AD_Pin GPIO_PIN_2
#define P13V_AD_GPIO_Port GPIOC
#define ISE_U_AD_Pin GPIO_PIN_0
#define ISE_U_AD_GPIO_Port GPIOA
#define ISE_V_AD_Pin GPIO_PIN_1
#define ISE_V_AD_GPIO_Port GPIOA
#define ISE_W_AD_Pin GPIO_PIN_2
#define ISE_W_AD_GPIO_Port GPIOA
#define BAT_VDC_AD_Pin GPIO_PIN_3
#define BAT_VDC_AD_GPIO_Port GPIOA
#define DAC1_DBG1_Pin GPIO_PIN_4
#define DAC1_DBG1_GPIO_Port GPIOA
#define DAC1_DBG2_Pin GPIO_PIN_5
#define DAC1_DBG2_GPIO_Port GPIOA
#define PCU_NTC_2_AD_Pin GPIO_PIN_7
#define PCU_NTC_2_AD_GPIO_Port GPIOA
#define PCU_NTC_0_AD_Pin GPIO_PIN_4
#define PCU_NTC_0_AD_GPIO_Port GPIOC
#define PCU_NTC_1_AD_Pin GPIO_PIN_5
#define PCU_NTC_1_AD_GPIO_Port GPIOC
#define HW_ID1_AD_Pin GPIO_PIN_1
#define HW_ID1_AD_GPIO_Port GPIOB
#define HW_ID1_ADB2_Pin GPIO_PIN_2
#define HW_ID1_ADB2_GPIO_Port GPIOB
#define FOIL_AD_Pin GPIO_PIN_7
#define FOIL_AD_GPIO_Port GPIOE
#define MOTOR_NTC_0_AD_Pin GPIO_PIN_9
#define MOTOR_NTC_0_AD_GPIO_Port GPIOE
#define SAFTYSSR_Pin GPIO_PIN_10
#define SAFTYSSR_GPIO_Port GPIOE
#define FOIL_DI2_Pin GPIO_PIN_11
#define FOIL_DI2_GPIO_Port GPIOE
#define FOIL_DI3_Pin GPIO_PIN_12
#define FOIL_DI3_GPIO_Port GPIOE
#define MCU_State_LED_Pin GPIO_PIN_11
#define MCU_State_LED_GPIO_Port GPIOB
#define CAN_0_RX_Pin GPIO_PIN_12
#define CAN_0_RX_GPIO_Port GPIOB
#define CAN_0_TX_Pin GPIO_PIN_13
#define CAN_0_TX_GPIO_Port GPIOB
#define CAN_1_RX_Pin GPIO_PIN_11
#define CAN_1_RX_GPIO_Port GPIOA
#define CAN_1_TX_Pin GPIO_PIN_12
#define CAN_1_TX_GPIO_Port GPIOA
#define F_SPI_CS_Pin GPIO_PIN_15
#define F_SPI_CS_GPIO_Port GPIOA
#define F_SPI_CK_Pin GPIO_PIN_3
#define F_SPI_CK_GPIO_Port GPIOB
#define F_SPI_MISO_Pin GPIO_PIN_4
#define F_SPI_MISO_GPIO_Port GPIOB
#define F_SPI_MOSI_Pin GPIO_PIN_5
#define F_SPI_MOSI_GPIO_Port GPIOB
#define F_SPI_HOLD_Pin GPIO_PIN_6
#define F_SPI_HOLD_GPIO_Port GPIOB
#define BUF_ENA_Pin GPIO_PIN_7
#define BUF_ENA_GPIO_Port GPIOB
#define BUF_FB_Pin GPIO_PIN_9
#define BUF_FB_GPIO_Port GPIOB
#define PWM_UN_Pin GPIO_PIN_0
#define PWM_UN_GPIO_Port GPIOE
#define PWM_UP_Pin GPIO_PIN_1
#define PWM_UP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

// If voltage control is PWR_REGULATOR_VOLTAGE_SCALE1_BOOST, then system clock frequency is 170 MHz.
#define SYSTEM_CLK_FREQ 170000000 		// unit in Hz
#define INITIAL_CURRENT_LOOP_FREQ 10000 // unit in Hz
#define INITIAL_REPET_COUNTER 1
#define INITIAL_PWM_PERIOD ( SYSTEM_CLK_FREQ / (INITIAL_CURRENT_LOOP_FREQ * ( INITIAL_REPET_COUNTER + 1 )) ) // unit in counter

#if USE_UART_COMMUNICATION
extern UART_HandleTypeDef huart3;
#endif

enum E_TargetID {
	TARGET_ID_GLOBAL = 0,
	TARGET_ID_AXIS1,
	TARGET_ID_AXIS2
};


#define SAFETY_SENSOR_SIGNAL_CONNECTED  GPIO_PIN_RESET
#define SAFETY_SENSOR_SIGNAL_DISCONNECTED  GPIO_PIN_SET
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
