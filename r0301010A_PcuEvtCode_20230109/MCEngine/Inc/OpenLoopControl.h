/*
 * OpenLoopControl.h
 *
 *  Created on: 2019年12月13日
 *      Author: Fernando
 */

#ifndef INC_OPENLOOPCONTROL_H_
#define INC_OPENLOOPCONTROL_H_

//#define OPEN_LOOP_CONTROL_INIT_OK 							0
//
//#define VIRTUAL_POSITION_INIT_ERROR_PERIOD_IS_NOT_POSITIVE 	0x8001
//#define VIRTUAL_POSITION_INIT_ERROR_ACCEL_IS_NOT_POSITIVE 	0x8002
//#define VIRTUAL_POSITION_INIT_ERROR_DECEL_IS_NOT_POSITIVE 	0x8003
//
//#define IF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE 			0x8004
//#define VF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE 			0x8005

enum OPEN_LOOP_CONTROL_INIT_STATUS_ENUM
{
	OPEN_LOOP_CONTROL_INIT_OK = 0,
	VIRTUAL_POSITION_INIT_ERROR_PERIOD_IS_NOT_POSITIVE = 0x8001,
	VIRTUAL_POSITION_INIT_ERROR_ACCEL_IS_NOT_POSITIVE,
	VIRTUAL_POSITION_INIT_ERROR_DECEL_IS_NOT_POSITIVE,
	IF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE,
	VF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE
};

typedef void (*pIfControlClean)( void* );
typedef uint16_t (*pIfControlInit)( void*, float, float, float, float, float );
typedef void (*pIfControlCalcCmd)( void*, float );
typedef void (*pVfControlClean)( void* );
typedef uint16_t (*pVfControlInit)( void*, float, float, float, float, float );
typedef void (*pVfControlCalcCmd)( void*, float );


typedef struct
{
	float Period;
	float RpmToEleSpeedGain;
	float VirtualEleSpeed;
	float VirtualEleAngle;
	float RpmAccel;
	float RpmDecel;
	float Rpm;
}	VIRTUAL_POSITION_TYPE;

typedef struct
{
	VIRTUAL_POSITION_TYPE Position;
	float Gain;
	float CurrAmp;
	float CurrLimit;
	pIfControlClean Clean;
	pIfControlInit Init;
	pIfControlCalcCmd Calc;
}	IF_CONTROL_TYPE;

typedef struct
{
	VIRTUAL_POSITION_TYPE Position;
	float Gain;
	float VoltAmp;
	pVfControlClean Clean;
	pVfControlInit Init;
	pVfControlCalcCmd Calc;
}	VF_CONTROL_TYPE;

void OpenLoopControl_IfControlClean( IF_CONTROL_TYPE *p );
uint16_t OpenLoopControl_IfControlInit( IF_CONTROL_TYPE *p, float Period, float Gain, float RpmAccel, float RpmDecel, float Polepair );
void OpenLoopControl_IfControlCalcCmd( IF_CONTROL_TYPE *p, float RpmTarget );
void OpenLoopControl_VfControlClean( VF_CONTROL_TYPE *p );
uint16_t OpenLoopControl_VfControlInit( VF_CONTROL_TYPE *p, float Period, float Gain, float RpmAccel, float RpmDecel, float Polepair );
void OpenLoopControl_VfControlCalcCmd( VF_CONTROL_TYPE *p, float RpmTarget );

#define VIRTUAL_POSITION_DEFAULT	\
{									\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	0.0f							\
}

#define IF_CONTROL_DEFAULT				\
{										\
	VIRTUAL_POSITION_DEFAULT,			\
	0.0f,								\
	0.0f,								\
	300.0f,								\
	(pIfControlClean)OpenLoopControl_IfControlClean,		\
	(pIfControlInit)OpenLoopControl_IfControlInit,		\
	(pIfControlCalcCmd)OpenLoopControl_IfControlCalcCmd	\
}

#define VF_CONTROL_DEFAULT				\
{										\
	VIRTUAL_POSITION_DEFAULT,			\
	0.0f,								\
	0.0f,								\
	(pVfControlClean)OpenLoopControl_VfControlClean,		\
	(pVfControlInit)OpenLoopControl_VfControlInit,		\
	(pVfControlCalcCmd)OpenLoopControl_VfControlCalcCmd	\
}


#endif /* INC_OPENLOOPCONTROL_H_ */
