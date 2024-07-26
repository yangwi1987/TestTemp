/*
 * FourQuadControl.h
 *
 *  Created on: 2020年3月11日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_FOURQUADCONTROL_H_
#define INC_FOURQUADCONTROL_H_

#include "SystemTableLinker.h"
#include "ThrotMap.h"
#include "Constant.h"
#include "ICANInterface.h"
#include "Pid.h"
#include "SmoothCurveChange.h"
#include "filter.h"
#include "ParamMgr.h"
#include "GearMode.h"

#define ONE_PERCENT	1
#define PLC_LOOP_TS 0.001f
#define FOUR_QUAD_TIMEBASE	0.001f

enum FOUR_QUAD_STATE {
	FourQuadState_None = 0,
	FourQuadState_Driving_I = 1,
	FourQuadState_BackRoll_II = 2,
	FourQuadState_Reverse_III = 3,
	FourQuadState_Regen_IV = 4
};

typedef void ( *functypeFourQuadControl_Init )( void * );
typedef void ( *functypeFourQuadControl_Switch )( void * );
typedef void ( *functypeFourQuadControl_Calc )( void * , uint8_t );
typedef void ( *functypeFourQuadControl_Reset )( void *, float );
typedef float ( *functypeFourQuadControl_DCCurrLimitComparator )( void *, float, float, float );

typedef struct {
	uint16_t InitFailure;
	uint8_t Driving_TNIndex;		// Setting parameter
	uint8_t Driving_TNIndexPrevious;// Setting parameter
	uint8_t DriveTableSelect;
	float MotorRPM;
	float MaxDcLimitRecord;			// internal parameter
	ENUM_VIRTUAL_GEAR  GearPositionState;	    // input
	uint16_t FourQuadState;			// Output
	float DrivePropulsion;			// Output
	float TorqueCommandOut;			// Output
	float LimpTransitRamp;
	float DriveRisingRamp;
	float DriveFallingRamp;
	float DriveChangeTime;
	float DCDrainLimit;
	float DrainRisingRamp;
	float DrainFallingRamp;
	float DrivePowerCmd;
	float ratAPP;
	float facPnlty;
	float SpdPnlty;
	uint8_t spdPnltyOvrd;
	uint8_t BoostState;
	DRIVE_TABLE_TYPE DriveCurve[DRIVE_TABLE_LENGTH];
	DRIVE_TABLE_TYPE DriveCurveNow[DRIVE_TABLE_LENGTH_NOW];
	functypeFourQuadControl_Init Init;
	functypeFourQuadControl_Switch Switch;
	functypeFourQuadControl_Calc Calc;
	functypeFourQuadControl_Reset Reset;
	functypeFourQuadControl_DCCurrLimitComparator DCCurrLimitComparator;
} FourQuadControl;

void FourQuadControl_Init( FourQuadControl *v );
void FourQuadControl_Switch( FourQuadControl *v );
void FourQuadControl_Calc( FourQuadControl *v, uint8_t TriggerLimpHome );
void FourQuadControl_Reset( FourQuadControl *v, float DCDrainLimit );
float FourQuadControl_DCCurrLimitComparator( FourQuadControl *v, float DCDrainCurr, float VbusReal, float VbusUsed );

#define FOURQUADCONTROL_DEFAULT { \
    0,         /* InitFailure;                                      */\
    0,         /* Driving_TNIndex;		// Setting parameter        */\
    0,         /* Driving_TNIndexPrevious;// Setting parameter      */\
	0,         /* uint8_t DriveTableSelect;        */\
    0.0f,      /* MotorRPM;                                         */\
    0.0f,      /* MaxDcLimitRecord;			// internal parameter   */\
	VIRTUAL_GEAR_N, /* GearPositionState;	    // input                */\
    0,         /* FourQuadState;			// Output               */\
    0.0f,      /* DrivePropulsion;			// Output               */\
    0.0f,      /* TorqueCommandOut;			// Output               */\
    0.0f,      /* LimpTransitSec;                                   */\
    0.0f,      /* DriveRisingRamp;                                  */\
    0.0f,      /* DriveFallingRamp;                                 */\
    0.0f,      /* DriveChangeTime;                                  */\
    0.0f,      /* DCDrainLimit;                                     */\
    0.0f,      /* DrainRisingRamp;                                  */\
    0.0f,      /* DrainFallingRamp;                                 */\
    0.0f,      /* DrivePowerCmd;                                    */\
    0.0f,      /* ratAPP;                                         */\
	1.0f,      /*facPnlty*/\
	10000.0f,      /*SpdPnlty*/\
	0,           /*spdPnltyOvrd*/\
	0,             /*BoostState*/\
	{DRIVE_TABLE_DEFAULT}, \
	{DRIVE_TABLE_DEFAULT}, \
	(functypeFourQuadControl_Init)FourQuadControl_Init, \
	(functypeFourQuadControl_Switch)FourQuadControl_Switch, \
	(functypeFourQuadControl_Calc)FourQuadControl_Calc, \
	(functypeFourQuadControl_Reset)FourQuadControl_Reset, \
	(functypeFourQuadControl_DCCurrLimitComparator) FourQuadControl_DCCurrLimitComparator	\
}

#endif /* INC_FOURQUADCONTROL_H_ */
