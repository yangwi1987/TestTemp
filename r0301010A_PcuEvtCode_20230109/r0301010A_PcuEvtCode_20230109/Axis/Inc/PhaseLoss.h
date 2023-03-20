/*
 * PhaseLoss.h
 *
 *  Created on: 2020年8月1日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_PHASELOSS_H_
#define INC_PHASELOSS_H_

#include "stdint.h"
#include "MotorControl.h"

enum PHASE_LOSS_INIT_ENUM
{
	PHASE_LOSS_INIT_OK = 0,
	PHASE_LOSS_INIT_ERROR_CURR_LEVEL = 0x8001,
	PHASE_LOSS_INIT_ERROR_CURR_CMD,
	PHASE_LOSS_INIT_ERROR_EXE_TIME,
};

enum PHASE_LOSS_ERROR_ENUM
{
	PHASE_LOSS_ERROR_NONE = 0,
	PHASE_LOSS_ERROR_HAPPEN
};

typedef void (*pfunPhaseLoss_RunTimeDetect)( void* );
typedef void (*pfunPhaseLoss_RunTimeClean)( void* );
typedef void (*pfunPhaseLoss_Clean)( void* );
typedef uint16_t (*pfunPhaseLoss_Init)( void*, float, float, float, float );
typedef void (*pfunPhaseLoss_Detection)( void*, void*, int16_t* );


typedef struct
{
	uint16_t Enable : 1;
	uint16_t Start : 1;
	uint16_t Error : 1;
	uint16_t Reserved : 13;
	uint16_t Timer;
	uint16_t ExeCnt;
	uint16_t State;
	uint16_t PhaseLossCnt;
	float CurrLevel;
	float CurrCmd;
	float IuAbsMax;
	float IvAbsMax;
	float IwAbsMax;
	float Avg1KhzCurrSqrU;
	float Avg1KhzCurrSqrV;
	float Avg1KhzCurrSqrW;
	float Acc10KhzCurrSqrU;
	float Acc10KhzCurrSqrV;
	float Acc10KhzCurrSqrW;
	float PLCLoopVcmdAmp;
	float PLCLoopElecSpeedAbs;
	float PLCLoopTorqueCmd;
	pfunPhaseLoss_RunTimeDetect RunTimeDetect;
	pfunPhaseLoss_Clean RunTimeClean;
	pfunPhaseLoss_Init Init;
	pfunPhaseLoss_Detection Detection;
} PHASE_LOSS_TYPE;

void PhaseLoss_RunTimeDetect( PHASE_LOSS_TYPE *p );
void PhaseLoss_RunTimeClean( PHASE_LOSS_TYPE *p );
void PhaseLoss_Clean( PHASE_LOSS_TYPE *p );
uint16_t PhaseLoss_Init( PHASE_LOSS_TYPE *p, float CurrLevel, float CurrCmd, float ExeTime, float Period );
void PhaseLoss_Detection( PHASE_LOSS_TYPE *p, MOTOR_CONTROL_TYPE *v, int16_t *CtrlMode );

#define PHASE_LOSS_DEFAULT	\
{					\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		/*PhaseLossCnt    */\
	0.0f,	/*CurrLevel       */\
	0.0f,	/*CurrCmd         */\
	0.0f,	/*IuAbsMax        */\
	0.0f,	/*IvAbsMax        */\
	0.0f,	/*IwAbsMax        */\
	0.0f,	/*Avg1KhzCurrSqrU */\
	0.0f,	/*Avg1KhzCurrSqrV */\
	0.0f,	/*Avg1KhzCurrSqrW */\
	0.0f,	/*Acc10KhzCurrSqrU*/\
	0.0f,	/*Acc10KhzCurrSqrV*/\
	0.0f,	/*Acc10KhzCurrSqrW*/\
	0.0f,	/*PLCLoopVcmdAmp*/\
	0.0f,	/*PLCLoopElecSpeedAbs*/\
	0.0f,	/*PLCLoopTorqueCmd*/\
	(pfunPhaseLoss_RunTimeDetect) PhaseLoss_RunTimeDetect,	\
	(pfunPhaseLoss_RunTimeClean) PhaseLoss_RunTimeClean,	\
	(pfunPhaseLoss_Init) PhaseLoss_Init,			\
	(pfunPhaseLoss_Detection) PhaseLoss_Detection,	\
}

#endif /* INC_PHASELOSS_H_ */
