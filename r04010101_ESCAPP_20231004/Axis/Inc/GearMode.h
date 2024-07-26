/*
 * GearMode.h
 *
 *  Created on: 2024年1月25日
 *      Author: Jeff.Changr
 *
 *      This module includes two features:
 *      1. Virtual Gear
 *      Input :
 *      ServoOn command from InverterStateMachine
 *      Reverse Btn
 *      ratAPP
 *      MotorSpeed
 *
 *      Process: switch the gear position according to the inputs
 *
 *      Output :
 *      Gear State (N, D, R)
 *
 *      2. Boost Mode
 *      Input:
 *      Boost Btn
 *
 *      Process:
 *
 */

#ifndef INC_GEARMODE_H_
#define INC_GEARMODE_H_

#include "stdint.h"
#include "Constant.h"
#include "E10App.h"
#include "ICANInterface.h"

#define GEAR_MODE_TIMEBASE 0.001f //PLC loop
#define BOOST_COOLDOWN_TIME_s 45.0f //s
#define BOOST_CONTINUE_TIME_s 10.0f //s
#define BOOST_COOLDOWN_TIME ( BOOST_COOLDOWN_TIME_s / GEAR_MODE_TIMEBASE )
#define BOOST_CONTINUE_TIME ( BOOST_CONTINUE_TIME_s / GEAR_MODE_TIMEBASE )

typedef enum       // Add BOOST_RUNNING in the next stage
{
	BOOST_READY = 0,
	BOOST_COOLDOWN,
	BOOST_MODE_ENABLE
}BoostState_e;

typedef struct
{
	BtnState_e   IsBoostBtnPressed;
	BtnState_e IsReverseBtnPressed ;
    uint16_t        BoostCnt;
    uint16_t        BoostCoolCnt;
    ENUM_VIRTUAL_GEAR   GearPositionState;
    BoostState_e    BoostState;
    uint8_t ServoOnCommand;
    uint8_t APPReleaseFlag;
    int16_t MotorRPM;
    int16_t MaxMotorRPMToEnRev;
}GearMode_Var_t;

#define GEAR_MODE_VAR_DEFALUT { \
		BTN_BOOST_RELEASE,  /*IsBoostBtnPressed;  */\
		BTN_REVERSE_RELEASE,/*IsReverseBtnPressed */\
		0,                   /*BoostCnt;      */\
		0,                   /*BoostCoolCnt;      */\
		VIRTUAL_GEAR_N,         /*GearPositionState;     */\
		BOOST_READY,          /*BoostState;         */\
		0,   /*ServoOnCommand*/\
		0,/*APPReleaseFlag*/\
		0,/*MotorRPM*/\
		0/*MaxMotorRPMToEnRev*/\
}


//extern void GearMode_Init ( GearMode_Var_t* v );
extern void GearMode_DoPLCLoop ( GearMode_Var_t* v );
extern void GearMode_Init ( GearMode_Var_t* v, int16_t MaxMotorRPMToEnRev );
extern void GearMode_EnableBoostMode (void);
extern void GearMode_DisableBoostMode (void);
extern void GearMode_EnableReverseMode (void);
extern void GearMode_DisableReverseMode (void);
#endif /* INC_GEARMODE_H_ */
