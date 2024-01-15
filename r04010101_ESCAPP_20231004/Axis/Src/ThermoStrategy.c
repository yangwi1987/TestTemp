/*
 * ThermoStrategy.c
 *
 *  Created on: 2020年8月14日
 *      Author: Mike.Wen.SFW
 */

#include "ThermoStrategy.h"

#define MIN2(a,b)  ( ((a) <= (b)) ? (a) : (b) )
#define MIN3(a,b,c)  (((( a > b ) ? b : a) > c ) ? c : ( a > b ? b : a ))
#define MOS_DERATING_TABLE_NUN 12
#define WINDING_DERATING_TABLE_NUN 8

static int16_t ModifiedWindingDeratingTable[WINDING_DERATING_TABLE_NUN] = {0};
static int16_t ModifiedMosDeratingTable[MOS_DERATING_TABLE_NUN] = {0};

void ThermoStrategy_Init( ThermoStrategy_t *v, const LUT_INIT_PARA_INT16_1DIM_TYPE *pWindingInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pMosInit, const LUT_INIT_PARA_INT16_1DIM_TYPE *pCapInit, AdcStation *pAdcStation )
{
	uint16_t i = 0;
	int16_t TempFirstY = 0; // first Y value in the original table
	int16_t XOffset = 0; // offset value from parameter
	float YScale = 0.0f; // scale from parameter

	// Init Derating parameters
	v->WindingLimit = v->WindingDerating.Init( &v->WindingDerating, pWindingInit );
	v->MosDerating.Init( &v->MosDerating, pMosInit );
	v->CapDerating.Init( &v->CapDerating, pCapInit );

	// Modify Winding derating table by parameters
	XOffset = DriveParams.SystemParams.MotorDeratingTempOffset - 32768;
	YScale = DriveParams.SystemParams.MotorDeratingScale * 0.01f;
	TempFirstY = v->WindingDerating.pTableStart[0];
	v->WindingDerating.X.InputMin = v->WindingDerating.X.InputMin + XOffset;
	for(i = 0; i<WINDING_DERATING_TABLE_NUN; i++)
	{
		ModifiedWindingDeratingTable[i] = (int16_t) (roundf( (float)TempFirstY + (float)(v->WindingDerating.pTableStart[i] - TempFirstY) * YScale ));
	}
 	v->WindingDerating.pTableStart = &ModifiedWindingDeratingTable[0];

	// Modify MOS derating table by parameters
	XOffset = DriveParams.SystemParams.MosDeratingTempOffset - 32768;
	YScale = DriveParams.SystemParams.MOSDeratingScale * 0.01f;
	TempFirstY = v->MosDerating.pTableStart[0];
	v->MosDerating.X.InputMin = v->MosDerating.X.InputMin + XOffset;
	for(i = 0; i<MOS_DERATING_TABLE_NUN; i++)
	{
		ModifiedMosDeratingTable[i] = (int16_t) (roundf( (float)TempFirstY + (float)(v->MosDerating.pTableStart[i] - TempFirstY) * YScale ));
	}
	v->MosDerating.pTableStart = &ModifiedMosDeratingTable[0];

	// Put Default settings
	v->MaxCurrPeak = MAX_AC_PHASE_CURRENT;
	v->ACCurrentLimitOut = v->MaxCurrPeak;
	v->ACCurrentLimit = v->MaxCurrPeak;

	// Link the pointer to AdcStation
	v->TempNow[MOS_NTC_1] = &(pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_1]);
	v->TempNow[MOS_NTC_2] = &(pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_2]);
	v->TempNow[CAP_NTC] = &(pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC]);
	v->TempNow[MOTOR_NTC_0_A0] = &(pAdcStation->AdcTraOut.MOTOR_NTC);

}

// 100ms
void ThermoStrategy_Calc( ThermoStrategy_t *v )
{
	float ACCurrLimitTarget = v->MaxCurrPeak;
	float MaxMosTemp = 0; // max MOS temperature between side an center.

	if( *(v->TempNow[MOS_NTC_1]) > *(v->TempNow[MOS_NTC_2]) )
	{
		MaxMosTemp = *(v->TempNow[MOS_NTC_1]);
	}
	else
	{
		MaxMosTemp = *(v->TempNow[MOS_NTC_2]);
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
