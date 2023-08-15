/*
 * ThermoStrategy.c
 *
 *  Created on: 2020年8月14日
 *      Author: Mike.Wen.SFW
 */

#include "ThermoStrategy.h"

#define ABS(x)  ((x) >= 0 ? (x) : -(x))
#define MIN2(a,b)  ( ((a) <= (b)) ? (a) : (b) )
#define MIN3(a,b,c)  (((( a > b ) ? b : a) > c ) ? c : ( a > b ? b : a ))

void ThermoStrategy_Init( ThermoStrategy_t *v, const LUT_INIT_PARA_INT16_1DIM_TYPE *pWindingInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pMosInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pCapInit, AdcStation *pAdcStation )
{
	// Init Derating parameters
	v->WindingLimit = v->WindingDerating.Init( &v->WindingDerating, pWindingInit );
	v->MosDerating.Init( &v->MosDerating, pMosInit );
	v->CapDerating.Init( &v->CapDerating, pCapInit );

	// Put Default settings
	v->MaxCurrPeak = MAX_AC_PHASE_CURRENT;
	v->ACCurrentLimitOut = v->MaxCurrPeak;
	v->ACCurrentLimit = v->MaxCurrPeak;

	// Link the pointer to AdcStation
	v->TempNow[MOS_NTC_CENTER] = &(pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_CENTER]);
	v->TempNow[MOS_NTC_SIDE] = &(pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_SIDE]);
	v->TempNow[CAP_NTC] = &(pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC]);
	v->TempNow[MOTOR_NTC_0_A0] = &(pAdcStation->AdcTraOut.MOTOR_NTC);

}

// 100ms
void ThermoStrategy_Calc( ThermoStrategy_t *v )
{
	float ACCurrLimitTarget = v->MaxCurrPeak;
	float MaxMosTemp = 0; // max MOS temperature between side an center.

	if( *(v->TempNow[MOS_NTC_CENTER]) > *(v->TempNow[MOS_NTC_SIDE]) )
	{
		MaxMosTemp = *(v->TempNow[MOS_NTC_CENTER]);
	}
	else
	{
		MaxMosTemp = *(v->TempNow[MOS_NTC_SIDE]);
	}

	v->WindingLimit = v->WindingDerating.Calc(&v->WindingDerating,*(v->TempNow[MOTOR_NTC_0_A0]));
	v->MOSACLimit = v->MosDerating.Calc(&v->MosDerating, MaxMosTemp);
	//v->CapLimit = v->CapDerating.Calc( &v->CapDerating, *(v->TempNow[PCU_NTC_2]) );
	v->CapLimit = v->MaxCurrPeak;
	ACCurrLimitTarget = MIN3(v->WindingLimit, v->MOSACLimit, v->CapLimit);

	if( ACCurrLimitTarget > v->MaxCurrPeak )
	{
		ACCurrLimitTarget = v->MaxCurrPeak;
	}
	else if( ACCurrLimitTarget < 0.0f )
	{
		ACCurrLimitTarget = 0.0f;
	}
	else
	{
		// do nothing
	}

	v->ACCurrentLimit = ACCurrLimitTarget;

	// Output Clamper
	if( v->ACCurrentLimit > v->MaxCurrPeak )
	{
		v->ACCurrentLimit = v->MaxCurrPeak;
	}
	else if( v->ACCurrentLimit < 0.0f )
	{
		v->ACCurrentLimit = 0.0f;
	}
	else
	{
		// do nothing
	}

	// To avoid multi-thread
	v->ACCurrentLimitOut = v->ACCurrentLimit;
}
