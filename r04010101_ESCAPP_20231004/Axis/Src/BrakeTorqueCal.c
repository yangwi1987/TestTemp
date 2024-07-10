/*
 * BrakeTorqueCal.c
 *
 *  Created on: 2024年7月9日
 *      Author: Jeff C
 */

#include "BrakeTorqueCal.h"

void BrakeTorqueCalc_Init( BrakeTorqueCalc_t *v, DriveParams_t *a )
{
	v->Enable = a->SystemParams.BrkAccCncl;
	v->IsUseAnalogInput = a->SystemParams.IsUseAnalogBrk;
	v->AnalogThreshold = (float)(a->SystemParams.AnalogBrkThreshold) * 0.1f;

}

int8_t BrakeTorqueCalc_PickSignal ( BrakeTorqueCalc_t *v, float AnalotIn, int8_t DigitalIn )
{
	int8_t result = 0;

	if ( v->Enable == 0)
	{
		result = DEFAULT_OUTPUT_WHEN_FUNCITON_DISABLED;
	}
	else if ( v->IsUseAnalogInput == 0)
	{
		result = ( DigitalIn == 0 ) ? BREAK_PEDAL_PRESSED : BREAK_PEDAL_RELEASED;
	}
	else
	{
		result = ( AnalotIn < v->AnalogThreshold ) ? BREAK_PEDAL_PRESSED : BREAK_PEDAL_RELEASED;
	}
	return result;
}
void BrakeTorqueCalc_ThrottleCalc( BrakeTorqueCalc_t *v, float *ThrottlePercentageTarget, int8_t BrakePedal )
{
	v->OutputFactor = ( BrakePedal == BREAK_PEDAL_PRESSED ) ? 0.0f : 1.0f;
	*ThrottlePercentageTarget = *ThrottlePercentageTarget * v->OutputFactor;
}
