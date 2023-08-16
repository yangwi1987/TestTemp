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

#define ONE_PERCENT	1
#define PLC_LOOP_TS 0.001f
#define FOUR_QUAD_TIMEBASE	0.001f

enum FOUR_QUAD_STATE {
	FourQuadState_None = 0,
	FourQuadState_Driving_I = 1,
	FourQuadState_BackRoll_II = 2,
};

typedef void ( *functypeFourQuadControl_Init )( void * );
typedef void ( *functypeFourQuadControl_Switch )( void * );
typedef void ( *functypeFourQuadControl_Calc )( void * );
typedef void ( *functypeFourQuadControl_Reset )( void *, float );
typedef float ( *functypeFourQuadControl_DCCurrLimitComparator )( void *, float, float, float );

typedef struct {
	uint16_t InitFailure;
	uint8_t Driving_TNIndex;		// Setting parameter
	uint8_t Driving_TNIndexPrevious;// Setting parameter
	float MotorSpeedRadps;			// input
	float MotorRPM;
	float TmaxSpeed; 				// internal parameter
	float MaxDcLimitRecord;			// internal parameter
	uint16_t FirstEntryFlg;         // internal parameter
	uint16_t ThrottleReleaseFlg;    // input
	int16_t  GearPositionState;	    // input
	int16_t  GearPositionCmd;	    // input
	uint16_t ServoCmdIn;            // input
	uint16_t ServoCmdOut;           // Output
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
	float DrivePowerLevelTarget;
	float DrivePowerLevelGain;
	float DrivePowerCmd;
	float Throttle;
	float DrivePowerLevelRampUp;
	float DrivePowerLevelRampDown;
	float DrivePowerLevelRampUpParamA[DRIVE_TABLE_LENGTH];
	float DrivePowerLevelRampUpParamB[DRIVE_TABLE_LENGTH];
	float DrivePowerLevelRampDownParamA[DRIVE_TABLE_LENGTH];
	float DrivePowerLevelRampDownParamB[DRIVE_TABLE_LENGTH];
	BACK_ROLL_TABLE_TYPE BackRollTable;
	DRIVE_TABLE_TYPE DriveCurve[DRIVE_TABLE_LENGTH];
	functypeFourQuadControl_Init Init;
	functypeFourQuadControl_Switch Switch;
	functypeFourQuadControl_Calc Calc;
	functypeFourQuadControl_Reset Reset;
	functypeFourQuadControl_DCCurrLimitComparator DCCurrLimitComparator;
} FourQuadControl;

void FourQuadControl_Init( FourQuadControl *v );
void FourQuadControl_Switch( FourQuadControl *v );
void FourQuadControl_Calc( FourQuadControl *v );
void FourQuadControl_Reset( FourQuadControl *v, float DCDrainLimit );
float FourQuadControl_DCCurrLimitComparator( FourQuadControl *v, float DCDrainCurr, float VbusReal, float VbusUsed );

#define FOURQUADCONTROL_DEFAULT { \
    0,         /* InitFailure;                                      */\
    0,         /* Driving_TNIndex;		// Setting parameter        */\
    0,         /* Driving_TNIndexPrevious;// Setting parameter      */\
    0.0f,      /* MotorSpeedRadps;			// input                */\
    0.0f,      /* MotorRPM;                                         */\
    0.0f,      /* TmaxSpeed; 				// internal parameter   */\
    0.0f,      /* MaxDcLimitRecord;			// internal parameter   */\
    1,         /* FirstEntryFlg;         // internal parameter      */\
    0,         /* ThrottleReleaseFlg;    // input                   */\
    PcuShiftP, /* GearPositionState;	    // input                */\
    PcuShiftP, /* GearPositionCmd;	    // input                    */\
    0,         /* ServoCmdIn;            // input                   */\
    0,         /* ServoCmdOut;           // Output                  */\
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
    0.0f,      /* DrivePowerLevelTarget;                            */\
    0.0f,      /* DrivePowerLevelGain;                              */\
    0.0f,      /* DrivePowerCmd;                                    */\
    0.0f,      /* Throttle;                                         */\
    0.0f,      /* DrivePowerLevelRampUp;                            */\
    0.0f,      /* DrivePowerLevelRampDown;                          */\
    {0.0f},    /* DrivePowerLevelRampUpParamA[DRIVE_TABLE_LENGTH];  */\
    {0.0f},    /* DrivePowerLevelRampUpParamB[DRIVE_TABLE_LENGTH];  */\
    {0.0f},    /* DrivePowerLevelRampDownParamA[DRIVE_TABLE_LENGTH];*/\
    {0.0f},    /* DrivePowerLevelRampDownParamB[DRIVE_TABLE_LENGTH];*/\
	BACK_ROLL_TABLE_DEFAULT, \
	{DRIVE_TABLE_DEFAULT}, \
	(functypeFourQuadControl_Init)FourQuadControl_Init, \
	(functypeFourQuadControl_Switch)FourQuadControl_Switch, \
	(functypeFourQuadControl_Calc)FourQuadControl_Calc, \
	(functypeFourQuadControl_Reset)FourQuadControl_Reset, \
	(functypeFourQuadControl_DCCurrLimitComparator) FourQuadControl_DCCurrLimitComparator	\
}

#endif /* INC_FOURQUADCONTROL_H_ */
