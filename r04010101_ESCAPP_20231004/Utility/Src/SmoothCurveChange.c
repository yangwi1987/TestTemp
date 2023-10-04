/*
 * SmoothCurveChange.c
 *
 *  Created on: 2021年4月18日
 *      Author: Fernando.Wang.HHW
 */

#include "SmoothCurveChange.h"

float SmoothCurveChange( float TargetNow, float ResultPrevious, uint8_t *CurveNow, uint8_t *CurvePrevious, float Rising, float Falling )
{
	float Out = 0.0f;
	if( *CurvePrevious == *CurveNow )
	{
		Out = TargetNow;
	}
	else
	{
		if( TargetNow > ResultPrevious )
		{
			Out = Ramp( ResultPrevious, TargetNow, Rising );
			if( Out == TargetNow )
			{
				*CurvePrevious = *CurveNow;
			}
			else
			{
				//do nothing
			}
		}
		else if( TargetNow < ResultPrevious )
		{
			Out = Ramp( ResultPrevious, TargetNow, Falling );
			if( Out == TargetNow )
			{
				*CurvePrevious = *CurveNow;
			}
			else
			{
				//do nothing
			}
		}
		else
		{
			Out = TargetNow;
			*CurvePrevious = *CurveNow;
		}
	}
	return Out;
}

