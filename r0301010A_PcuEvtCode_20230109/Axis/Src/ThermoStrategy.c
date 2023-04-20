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
	v->TempNow[PCU_NTC_0] = &(pAdcStation->AdcTraOut.PCU_NTC[0]);
	v->TempNow[PCU_NTC_1] = &(pAdcStation->AdcTraOut.PCU_NTC[1]);
	v->TempNow[PCU_NTC_2] = &(pAdcStation->AdcTraOut.PCU_NTC[2]);
	v->TempNow[MOTOR_NTC_0_A0] = &(pAdcStation->AdcTraOut.MOTOR_NTC);

}

// 100ms
void ThermoStrategy_Calc( ThermoStrategy_t *v )
{
	float ACCurrLimitTarget = v->MaxCurrPeak;
	float MaxMosTemp = 0; // max MOS temperature between side an center.

	if( *(v->TempNow[PCU_NTC_0]) > *(v->TempNow[PCU_NTC_1]) )
	{
		MaxMosTemp = *(v->TempNow[PCU_NTC_0]);
	}
	else
	{
		MaxMosTemp = *(v->TempNow[PCU_NTC_1]);
	}

	// MaxMosTemp = MAX(*(v->TempNow[PCU_NTC_0]), *(v->TempNow[PCU_NTC_1]));

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
