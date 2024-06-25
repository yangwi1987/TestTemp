/*
 * AngleObserver.h
 *
 *  Created on: 2022年1月10日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_ANGLEOBSERVER_H_
#define INC_ANGLEOBSERVER_H_
#include "constant.h"

enum ANGLE_OBSERVER_INIT_STATUS
{
	ANGLE_OBSERVER_INIT_OK = 0,
	ANGLE_OBSERVER_ERROR_J,
	ANGLE_OBSERVER_ERROR_SPEED_LIMIT,
	ANGLE_OBSERVER_ERROR_ACEL_LIMIT,
};

typedef void (*pfunAngleObserver_Clean)( void*, float, float, float );
typedef uint16_t (*pfunAngleObserver_Init)( void*, void* );
typedef void (*pfunAngleObserver_Calc)( void*, float );

typedef struct
{
	float Period;
	float J;
	float L1;
	float L2;
	float L3;
	float SpeedUpperLimit;
	float SpeedLowerLimit;
	float AcelUpperLimit;
	float AcelLowerLimit;
} AngleObserverInitParm_t;

typedef struct
{
	uint16_t Error : 8;
	uint16_t LowerLevelError : 8;
	int16_t Quotient;
	float Period;
	float J;
	float DivideJ;
	float L1;
	float L2;
	float L3;
	float L1Result;
	float L2Result;
	float L3Result;
	float Te;
	float AcelTmp;
	float Acel;
	float AcelUpperLimit;
	float AcelLowerLimit;
	float SpeedTmp;
	float Speed;
	float SpeedUpperLimit;
	float SpeedLowerLimit;
	float Angle;
	float AngleError;
	float CorrespondingAngleError;
	pfunAngleObserver_Clean Clean;
	pfunAngleObserver_Init Init;
	pfunAngleObserver_Calc Calc;
} AngleObserver_t;

void AngleObserver_Clean( AngleObserver_t *p, float AngleNow, float SpeedNow, float AcelNow );
uint16_t AngleObserver_Init( AngleObserver_t *p, AngleObserverInitParm_t *pSetting );
void AngleObserver_Calc( AngleObserver_t *p, float AngleIn );

#define ANGLE_OBSERVER_INIT_PARAM_DEFAULT \
{				\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
	0.0f,		\
}

#define ANGLE_OBSERVER_DEFAULT \
{			\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	(pfunAngleObserver_Clean)AngleObserver_Clean,	\
	(pfunAngleObserver_Init)AngleObserver_Init,		\
	(pfunAngleObserver_Calc)AngleObserver_Calc,		\
}

#define ANGLE_OBSERVER_CLEAN_MACRO( p, AcelNow, SpeedNow, AngleNow ) \
{											\
	p->L1Result = 0.0f;						\
	p->L2Result = 0.0f;						\
	p->L3Result = 0.0f;						\
	p->AcelTmp = AcelNow;					\
	p->Acel = AcelNow;						\
	p->SpeedTmp = SpeedNow;					\
	p->Speed = SpeedNow;					\
	p->Angle = AngleNow;					\
	p->AngleError = 0.0f;					\
	p->CorrespondingAngleError = 0.0f;		\
}

#define ANGLE_OBSERVER_CALC_MACRO( p, AngleIn )													\
{																								\
	p->AngleError = ( AngleIn - p->Angle );														\
	p->Quotient = (int16_t)( p->AngleError * INV_2PI);											\
	p->Quotient = ( p->Quotient >= 0 ) ? p->Quotient : p->Quotient - 1;							\
	p->CorrespondingAngleError = p->AngleError - ((float)p->Quotient) * _2PI;					\
	p->CorrespondingAngleError = ( p->CorrespondingAngleError > _PI ) ? ( p->CorrespondingAngleError - _2PI ) : p->CorrespondingAngleError;	\
	p->CorrespondingAngleError = ( p->CorrespondingAngleError < -_PI ) ? ( p->CorrespondingAngleError + _2PI ) : p->CorrespondingAngleError;	\
	p->L1Result = p->CorrespondingAngleError * p->L1;											\
	p->L2Result = p->CorrespondingAngleError * p->L2;											\
	p->L3Result = p->CorrespondingAngleError * p->L3;											\
	p->AcelTmp += ( p->L3Result * p->Period );													\
	p->AcelTmp = ( p->AcelTmp > p->AcelUpperLimit ) ? p->AcelUpperLimit : p->AcelTmp;			\
	p->AcelTmp = ( p->AcelTmp < -p->AcelUpperLimit ) ? -p->AcelUpperLimit : p->AcelTmp;			\
	p->Acel = p->L2Result + p->AcelTmp;															\
	p->Acel = ( p->Acel > p->AcelUpperLimit ) ? p->AcelUpperLimit : p->Acel;					\
	p->Acel = ( p->Acel < -p->AcelUpperLimit ) ? -p->AcelUpperLimit : p->Acel;					\
	p->SpeedTmp += ( p->Acel * p->Period );														\
	p->SpeedTmp = ( p->SpeedTmp > p->SpeedUpperLimit ) ? p->SpeedUpperLimit : p->SpeedTmp;		\
	p->SpeedTmp = ( p->SpeedTmp < -p->SpeedUpperLimit ) ? -p->SpeedUpperLimit : p->SpeedTmp;	\
	p->Speed = p->L1Result + p->SpeedTmp;														\
	p->Speed = ( p->Speed > p->SpeedUpperLimit ) ? p->SpeedUpperLimit : p->Speed;				\
	p->Speed = ( p->Speed < -p->SpeedUpperLimit ) ? -p->SpeedUpperLimit : p->Speed;				\
	p->Angle += ( p->Speed * p->Period );														\
	p->Angle = ( p->Angle >= _2PI ) ? p->Angle - _2PI : p->Angle;								\
	p->Angle = ( p->Angle < 0 ) ? p->Angle + _2PI : p->Angle;									\
}

#endif /* INC_ANGLEOBSERVER_H_ */
