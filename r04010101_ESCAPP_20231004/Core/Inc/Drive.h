/*
 * Drive.h
 *
 *  Created on: Dec 19, 2019
 *      Author: MikeSFWen
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "UtilityBase.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "PWM_TRA.h"
#include "ADC_DRI.h"
#include "ExtranetCANStation.h"
#include "CANDrive.h"

//extern UART_Module UART_Module1;
extern AdcStation AdcStation1;
extern PwmStation PwmStation1;
extern ExtranetCANStation_t ExtranetCANStation;


extern FDCAN_HandleTypeDef hfdcan2;
extern TIM_HandleTypeDef htim6; // PLC loop
extern TIM_HandleTypeDef htim7; // 100Hz loop
extern DAC_HandleTypeDef hdac1;
extern CORDIC_HandleTypeDef hcordic;
extern TIM_HandleTypeDef htim3; // total time
extern TIM_HandleTypeDef htim2; // QEP
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart3;

extern void drive_Init(void);
extern void drive_DoCurrentLoop(void);
extern void drive_DoPLCLoop(void);


extern void DisableMcuModule( void );



#endif /* INC_DRIVE_H_ */
