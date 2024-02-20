/*
 * GearMode.h
 *
 *  Created on: 2024年1月25日
 *      Author: Jeff.Changr
 *
 *      Integrate with FourQuadControl In the next stage!!!
 */

#ifndef INC_GEARMODE_H_
#define INC_GEARMODE_H_

#include "stdint.h"
#include "Constant.h"
#include "E10App.h"

#define GEAR_MODE_TIMEBASE 0.001f //PLC loop
#define BOOST_COOLDOWN_TIME_s 45.0f //s
#define BOOST_CONTINUE_TIME_s 10.0f //s
#define BOOST_COOLDOWN_TIME ( BOOST_COOLDOWN_TIME_s / GEAR_MODE_TIMEBASE )
#define BOOST_CONTINUE_TIME ( BOOST_CONTINUE_TIME_s / GEAR_MODE_TIMEBASE )

typedef enum
{
	NORMAL_MODE,
	BOOST_MODE,
	REVERSE_MODE
}GearModeSel_e;

typedef enum       // Add BOOST_RUNNING in the next stage
{
	BOOST_READY = 0,
	BOOST_COOLDOWN
}BoostState_e;

typedef struct
{
	BtnState_e   IsBoostBtnPressed;
	BtnState_e IsReverseBtnPressed ;
    uint16_t        BoostCnt;
    uint16_t        BoostCoolCnt;
    GearModeSel_e   GearModeSelect;
    BoostState_e    BoostState;

}GearMode_Var_t;

#define GEAR_MODE_VAR_DEFALUT { \
		BTN_BOOST_RELEASE,  /*IsBoostBtnPressed;  */\
		BTN_REVERSE_RELEASE,/*IsReverseBtnPressed */\
		0,                   /*BoostCnt;      */\
		0,                   /*BoostCoolCnt;      */\
		NORMAL_MODE,         /*GearModeSelect;     */\
		BOOST_READY          /*BoostState;         */\
}


//extern void GearMode_Init ( GearMode_Var_t* v );
extern void GearMode_DoPLCLoop ( GearMode_Var_t* v );
extern void GearMode_EnableBoostMode (void);
extern void GearMode_DisableBoostMode (void);
extern void GearMode_EnableReverseMode (void);
extern void GearMode_DisableReverseMode (void);
#endif /* INC_GEARMODE_H_ */
