/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"
// #include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include <stdio.h>
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
#define MSPI_SCLK_Pin GPIO_PIN_2
#define MSPI_SCLK_GPIO_Port GPIOE
#define MSPI_CS_Pin GPIO_PIN_4
#define MSPI_CS_GPIO_Port GPIOE
#define MSPI_MISO_Pin GPIO_PIN_5
#define MSPI_MISO_GPIO_Port GPIOE
#define MSPI_MOSI_Pin GPIO_PIN_6
#define MSPI_MOSI_GPIO_Port GPIOE
#define Debug_DO2_Pin GPIO_PIN_14
#define Debug_DO2_GPIO_Port GPIOC
#define Debug_DO1_Pin GPIO_PIN_15
#define Debug_DO1_GPIO_Port GPIOC
#define E5V_FB_ADC_Pin GPIO_PIN_1
#define E5V_FB_ADC_GPIO_Port GPIOC
#define MP_PWM_TIM20_CH2_Pin GPIO_PIN_2
#define MP_PWM_TIM20_CH2_GPIO_Port GPIOC
#define MOTOR_NTC_0_AD_Pin GPIO_PIN_3
#define MOTOR_NTC_0_AD_GPIO_Port GPIOC
#define MP_B_TIM2_CH1_Pin GPIO_PIN_0
#define MP_B_TIM2_CH1_GPIO_Port GPIOA
#define MP_A_TIM2_CH2_Pin GPIO_PIN_1
#define MP_A_TIM2_CH2_GPIO_Port GPIOA
#define Debug1_DAC_Pin GPIO_PIN_4
#define Debug1_DAC_GPIO_Port GPIOA
#define MP_Z_TIM2_ETR_Pin GPIO_PIN_5
#define MP_Z_TIM2_ETR_GPIO_Port GPIOA
#define HWOCP_BKIN_Pin GPIO_PIN_6
#define HWOCP_BKIN_GPIO_Port GPIOA
#define MOTOR_NTC_2_AD_Pin GPIO_PIN_7
#define MOTOR_NTC_2_AD_GPIO_Port GPIOA
#define ISEN_UFault_DI_Pin GPIO_PIN_4
#define ISEN_UFault_DI_GPIO_Port GPIOC
#define MOTOR_NTC_1_AD_Pin GPIO_PIN_5
#define MOTOR_NTC_1_AD_GPIO_Port GPIOC
#define HW_ID1_ADC_Pin GPIO_PIN_0
#define HW_ID1_ADC_GPIO_Port GPIOB
#define HW_ID2_ADC_Pin GPIO_PIN_1
#define HW_ID2_ADC_GPIO_Port GPIOB
#define ISE_W_AD_Pin GPIO_PIN_2
#define ISE_W_AD_GPIO_Port GPIOB
#define PreC_FB_ADC_Pin GPIO_PIN_7
#define PreC_FB_ADC_GPIO_Port GPIOE
#define ISE_V_AD_Pin GPIO_PIN_8
#define ISE_V_AD_GPIO_Port GPIOE
#define ISE_U_AD_Pin GPIO_PIN_9
#define ISE_U_AD_GPIO_Port GPIOE
#define Acc__FB1_1_ADC_Pin GPIO_PIN_10
#define Acc__FB1_1_ADC_GPIO_Port GPIOE
#define DiagEN_DO_Pin GPIO_PIN_11
#define DiagEN_DO_GPIO_Port GPIOE
#define EA5V_FB_ADC_Pin GPIO_PIN_13
#define EA5V_FB_ADC_GPIO_Port GPIOE
#define Acc__FB2_1_ADC_Pin GPIO_PIN_14
#define Acc__FB2_1_ADC_GPIO_Port GPIOE
#define ISEN_WFault_DI_Pin GPIO_PIN_15
#define ISEN_WFault_DI_GPIO_Port GPIOE
#define ISEN_VFault_DI_Pin GPIO_PIN_10
#define ISEN_VFault_DI_GPIO_Port GPIOB
#define ES5V_FB_ADC_Pin GPIO_PIN_11
#define ES5V_FB_ADC_GPIO_Port GPIOB
#define CAN_0_RX_Pin GPIO_PIN_12
#define CAN_0_RX_GPIO_Port GPIOB
#define CAN_0_TX_Pin GPIO_PIN_13
#define CAN_0_TX_GPIO_Port GPIOB
#define Rear_Fault_DI_Pin GPIO_PIN_14
#define Rear_Fault_DI_GPIO_Port GPIOB
#define Front_Fault_DI_Pin GPIO_PIN_15
#define Front_Fault_DI_GPIO_Port GPIOB
#define UART_TX_1_Pin GPIO_PIN_8
#define UART_TX_1_GPIO_Port GPIOD
#define UART_RX_1_Pin GPIO_PIN_9
#define UART_RX_1_GPIO_Port GPIOD
#define INV_P_FB__ADC_Pin GPIO_PIN_10
#define INV_P_FB__ADC_GPIO_Port GPIOD
#define Mos1_T_1_ADC_Pin GPIO_PIN_11
#define Mos1_T_1_ADC_GPIO_Port GPIOD
#define Mos2_T_1_ADC_Pin GPIO_PIN_12
#define Mos2_T_1_ADC_GPIO_Port GPIOD
#define ECAP_T_1_ADC_Pin GPIO_PIN_13
#define ECAP_T_1_ADC_GPIO_Port GPIOD
#define S13V8_FB_ADC_Pin GPIO_PIN_14
#define S13V8_FB_ADC_GPIO_Port GPIOD
#define Front_sig_DO_Pin GPIO_PIN_15
#define Front_sig_DO_GPIO_Port GPIOD
#define Rear_sig_DO_Pin GPIO_PIN_6
#define Rear_sig_DO_GPIO_Port GPIOC
#define Boost_DI_Pin GPIO_PIN_7
#define Boost_DI_GPIO_Port GPIOC
#define Reverse_DI_Pin GPIO_PIN_8
#define Reverse_DI_GPIO_Port GPIOC
#define Brake_DI_Pin GPIO_PIN_9
#define Brake_DI_GPIO_Port GPIOC
#define I2C_SDA_1_Pin GPIO_PIN_8
#define I2C_SDA_1_GPIO_Port GPIOA
#define I2C_SCL_1_Pin GPIO_PIN_9
#define I2C_SCL_1_GPIO_Port GPIOA
#define I2C_nRST_1_Pin GPIO_PIN_10
#define I2C_nRST_1_GPIO_Port GPIOA
#define Kill_Switch_DI_Pin GPIO_PIN_11
#define Kill_Switch_DI_GPIO_Port GPIOA
#define PWM_HV_Pin GPIO_PIN_15
#define PWM_HV_GPIO_Port GPIOA
#define PWM_LV_Pin GPIO_PIN_10
#define PWM_LV_GPIO_Port GPIOC
#define PWM_HW_Pin GPIO_PIN_12
#define PWM_HW_GPIO_Port GPIOC
#define PWM_HU_Pin GPIO_PIN_0
#define PWM_HU_GPIO_Port GPIOD
#define PWM_LU_Pin GPIO_PIN_1
#define PWM_LU_GPIO_Port GPIOD
#define BUF_FB_DI_Pin GPIO_PIN_2
#define BUF_FB_DI_GPIO_Port GPIOD
#define BUF_ENA_DO_Pin GPIO_PIN_3
#define BUF_ENA_DO_GPIO_Port GPIOD
#define FLASH_CS_DO_Pin GPIO_PIN_7
#define FLASH_CS_DO_GPIO_Port GPIOD
#define FLASH_SPI_SCK_Pin GPIO_PIN_3
#define FLASH_SPI_SCK_GPIO_Port GPIOB
#define FALSH_SPI_MISO_Pin GPIO_PIN_4
#define FALSH_SPI_MISO_GPIO_Port GPIOB
#define FLASH_SPI_MOSI_Pin GPIO_PIN_5
#define FLASH_SPI_MOSI_GPIO_Port GPIOB
#define FLASH_HOLD_DO_Pin GPIO_PIN_6
#define FLASH_HOLD_DO_GPIO_Port GPIOB
#define boot0_DI_Pin GPIO_PIN_8
#define boot0_DI_GPIO_Port GPIOB
#define PWM_LW_Pin GPIO_PIN_9
#define PWM_LW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// If voltage control is PWR_REGULATOR_VOLTAGE_SCALE1_BOOST, then system clock frequency is 170 MHz.
#define SYSTEM_CLK_FREQ 170000000 		// unit in Hz
#define INITIAL_CURRENT_LOOP_FREQ 10000 // unit in Hz
#define INITIAL_REPET_COUNTER 1
#define INITIAL_PWM_PERIOD ( SYSTEM_CLK_FREQ / (INITIAL_CURRENT_LOOP_FREQ * ( INITIAL_REPET_COUNTER + 1 )) ) // unit in counter


enum E_TargetID {
	TARGET_ID_GLOBAL = 0,
	TARGET_ID_AXIS1,
	TARGET_ID_AXIS2
};


// #define SAFETY_SENSOR_SIGNAL_CONNECTED  GPIO_PIN_RESET
// #define SAFETY_SENSOR_SIGNAL_DISCONNECTED  GPIO_PIN_SET
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
