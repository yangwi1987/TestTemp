/*
 * ThermoStrategy.h
 *
 *  Created on: 2020年8月14日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_THERMOSTRATEGY_H_
#define INC_THERMOSTRATEGY_H_

#include "stdio.h"
#include "ADC_DRI.h"
#include "MathFunctionInclude.h"

#define		MIN_AC_PHASE_CURRENT				(0.0f)
#define		MAX_AC_PHASE_CURRENT				(531.0f)
#define		MOS_DERATING						0x01
#define		CAP_DERATING						0x02
#define		MOTOR_DERATING						0x04

typedef void ( *functypeThermoStrategy_Init )( void *, const void*, const void*, const void*, void* );
typedef void ( *functypeThermoStrategy_Calc )( void * );


typedef struct {
	float MaxCurrPeak;
	float WindingLimit;
	float MOSACLimit;
	float CapLimit;
	float ACCurrentLimit;
	float ACCurrentLimitOut;
	float *TempNow[NTC_NONE];
	uint8_t ThermoDeratingSrc; // thermo derating source
	LUT_INT16_1DIM_TYPE WindingDerating;
	LUT_INT16_1DIM_TYPE MosDerating;
	LUT_INT16_1DIM_TYPE CapDerating;
	functypeThermoStrategy_Init Init;
	functypeThermoStrategy_Calc Calc;
} ThermoStrategy_t;

void ThermoStrategy_Init( ThermoStrategy_t *v, const LUT_INIT_PARA_INT16_1DIM_TYPE *pWindingInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pMosInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pCapInit, AdcStation *pAdcStation );
void ThermoStrategy_Calc( ThermoStrategy_t *v );

#define THERMOSTRATEGY_DEFAULT \
{	MAX_AC_PHASE_CURRENT, /* MaxCurrPeak */ \
	MAX_AC_PHASE_CURRENT, /* WindingLimit */ \
	MAX_AC_PHASE_CURRENT, /* MOSACLimit */ \
	MAX_AC_PHASE_CURRENT, /* CapLimit */ \
	MAX_AC_PHASE_CURRENT, /* ACCurrentLimit */ \
	MAX_AC_PHASE_CURRENT, /* ACCurrentLimitOut */ \
	{0}, /* *TempNow[NTC_NONE] */ \
	0, /* ThermoDeratingSrc */ \
	LUT_INT16_1DIM_DEFAULT,	\
	LUT_INT16_1DIM_DEFAULT,	\
	LUT_INT16_1DIM_DEFAULT,	\
	(functypeThermoStrategy_Init)ThermoStrategy_Init, \
	(functypeThermoStrategy_Calc)ThermoStrategy_Calc, \
}

#endif /* INC_THERMOSTRATEGY_H_ */
