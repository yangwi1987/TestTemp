/*
 * GearMode.h
 *
 *  Created on: 2024年1月25日
 *      Author: UJeff.Changr
 */

#ifndef INC_GEARMODE_H_
#define INC_GEARMODE_H_

#include "stdint.h"
#include "Constant.h"

#define GEAR_MODE_TIMEBASE 0.001f //PLC loop
#define BOOST_COOLDOWN_TIME_s 45.0f //s
#define BOOST_CONTINUE_TIME_s 10.0f //s
#define BOOST_COOLDOWN_TIME ( BOOST_COOLDOWN_TIME_s / GEAR_MODE_TIMEBASE )
#define BOOST_CONTINUE_TIME ( BOOST_CONTINUE_TIME_s / GEAR_MODE_TIMEBASE )

typedef enum
{
	BOOST_BTN_RELEASED = 0,
	BOOST_BTN_PRESSED
}BoostBtnState;

typedef enum
{
	REVERSE_BTN_RELEASED = 0,
	REVERSE_BTN_PRESSED
}ReverseBtnState;

typedef enum
{
	NORMAL_MODE,
	BOOST_MODE,
	REVERSE_MODE
}GearModeSel_e;

typedef enum
{
	BOOST_READY = 0,
	BOOST_COOLDOWN
}BoostState_e;

typedef struct
{
	BoostBtnState   IsBoostBtnPressed;
	ReverseBtnState IsReverseBtnPressed ;
    uint16_t        BoostCnt;
    uint16_t        BoostCoolCnt;
    GearModeSel_e   GearModeSelect;
    BoostState_e    BoostState;

}GearMode_Var_t;

#define GEAR_MODE_VAR_DEFALUT { \
		BOOST_BTN_RELEASED,  /*IsBoostBtnPressed;  */\
		REVERSE_BTN_RELEASED,/*IsReverseBtnPressed */\
		0,                   /*BoostCnt;      */\
		0,                   /*BoostCoolCnt;      */\
		NORMAL_MODE,         /*GearModeSelect;     */\
		BOOST_READY          /*BoostState;         */\
}


//extern void GearMode_Init ( GearMode_Var_t* v );
extern void GearMode_DoPLCLoop ( GearMode_Var_t* v );
extern void GearMode_EnableBoostMode (void);
extern void GearMode_DisableBoostMode (void);

#endif /* INC_GEARMODE_H_ */
