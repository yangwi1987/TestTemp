/*
 * ThrotMap.c
 *
 *  Created on: 2021年1月6日
 *      Author: Hank.Chen.CHC
 */

#include "ThrotMap.h"

void ThrottleMapping_Init( ThrottleMapping_t *v, DriveParams_t *a )
{
	int i;
	for( i = 0; i < MAX_TN_CNT; i++ )
	{
		if( ( ( float )( a->SystemParams.ThrottleEmptyPt[i] *0.01f ) < 1.0f ) && ( ( float )( a->SystemParams.ThrottleEmptyPt[i] *0.01f ) > 0.0f ) )
		{
			v->InitError = 0;
		}
		else
		{
			v->InitError |= 0x1;
		}
		if( ( ( float )( a->SystemParams.ThrottleHalfPt[i] *0.01f ) < 1.0f ) && ( ( float )( a->SystemParams.ThrottleHalfPt[i] *0.01f ) > 0.0f ) )
		{
			v->InitError = 0;
		}
		else
		{
			v->InitError |= 0x2;
		}
		if( ( ( float )( a->SystemParams.ThrottleFullPt[i] *0.01f ) < 1.0f ) && ( ( float )( a->SystemParams.ThrottleFullPt[i] *0.01f ) > 0.0f ) )
		{
			v->InitError = 0;
		}
		else
		{
			v->InitError |= 0x4;
		}
		if( ( ( float )( a->SystemParams.ThrottleRiseRamp[i] *0.001f ) < 10.1f ) && ( ( float )( a->SystemParams.ThrottleRiseRamp[i] *0.001f ) > 0.0f ) )
		{
			v->InitError = 0;
		}
		else
		{
			v->InitError |= 0x8;
		}
	}
	if( ( ( float )( a->SystemParams.ThrottleFallRamp *0.001f ) < 10.1f ) && ( ( float )( a->SystemParams.ThrottleFallRamp *0.001f ) > 0.0f ) )
	{
		v->InitError = 0;
	}
	else
	{
		v->InitError |= 0x8;
	}
	if( v->InitError == 0x0 )
	{
		for( i = 0; i < MAX_TN_CNT; i++ )
		{
			v->ThrottleTnTab[i].EmptyThrottle = ( float )( a->SystemParams.ThrottleEmptyPt[i] *0.01f  );
			v->ThrottleTnTab[i].HalfThrottle  = ( float )( a->SystemParams.ThrottleHalfPt[i]  *0.01f  );
			v->ThrottleTnTab[i].FullThrottle  = ( float )( a->SystemParams.ThrottleFullPt[i]  *0.01f  );
			v->ThrottleTnTab[i].ThrotRiseRamp 	  = ( float )( a->SystemParams.ThrottleRiseRamp[i]    *0.001f );
			v->ThrottleTnTab[i].ThrotFallRamp 	  = ( float )( a->SystemParams.ThrottleFallRamp    *0.001f );
		}
	}

	v->PercentageOut = 0.0f;
	v->PercentageTarget = 0.0f;
}

void ThrottleMapping_SmoothChange( ThrottleMapping_t *v )
{
	float EmptyThrottleTmp = v->ThrottleTnTab[ v->TnSelectDelay ].EmptyThrottle;
	float HalfThrottleTmp = v->ThrottleTnTab[ v->TnSelectDelay ].HalfThrottle;
	float FullThrottleTmp = v->ThrottleTnTab[ v->TnSelectDelay ].FullThrottle;
	if ( v->ChangeTime < THROTTLE_HANDLE_TIMEBASE )
	{
		v->ChangeTime = THROTTLE_HANDLE_TIMEBASE;
	}
	v->ChangeRamp = 1.0f / v->ChangeTime * THROTTLE_HANDLE_TIMEBASE;
	v->EmptyThrottleRamp = ( EmptyThrottleTmp - v->EmptyThrottle ) * v->ChangeRamp;
	v->EmptyThrottleRamp = ( v->EmptyThrottleRamp > 0.0f ) ? v->EmptyThrottleRamp : -v->EmptyThrottleRamp;
	v->HalfThrottleRamp = ( HalfThrottleTmp - v->HalfThrottle ) * v->ChangeRamp;
	v->HalfThrottleRamp = ( v->HalfThrottleRamp > 0.0f ) ? v->HalfThrottleRamp : -v->HalfThrottleRamp;
	v->FullThrottleRamp = ( FullThrottleTmp - v->FullThrottle ) * v->ChangeRamp;
	v->FullThrottleRamp = ( v->FullThrottleRamp > 0.0f ) ? v->FullThrottleRamp : -v->FullThrottleRamp;
	v->EmptyThrottle = SmoothCurveChange( EmptyThrottleTmp, v->EmptyThrottle, &(v->TnSelectDelay) , &(v->TnSelectEmpty), v->EmptyThrottleRamp, v->EmptyThrottleRamp );
	v->HalfThrottle = SmoothCurveChange( HalfThrottleTmp, v->HalfThrottle, &(v->TnSelectDelay) , &(v->TnSelectHalf), v->HalfThrottleRamp, v->HalfThrottleRamp );
	v->FullThrottle = SmoothCurveChange( FullThrottleTmp, v->FullThrottle, &(v->TnSelectDelay) , &(v->TnSelectFull), v->FullThrottleRamp, v->FullThrottleRamp );
}

void ThrottleMapping_Calc( ThrottleMapping_t *v )
{
	v->ThrottleReleaseFlag = 0;
	if( v->ThrottleRawIn > THROT_MAXIMUM )
	{
		v->PercentageOut = 1.0f;
		return;
	}
	else if( v->ThrottleRawIn < THROT_MINIMUM )
	{
		v->PercentageOut = 0.0f;
		v->ThrottleReleaseFlag = 1;
		return;
	}
	else if( v->ThrottleDI == 0 )
	{
		v->PercentageOut = 0.0f;
		v->ThrottleReleaseFlag = 1;
		return;
	}
	ThrottleMapping_SmoothChange( v );
	if( v->ThrottleRawIn <= v->EmptyThrottle )
	{
		v->PercentageTarget = 0.0f;
		v->ThrottleReleaseFlag = 1;
	}
	else if( ( v->ThrottleRawIn > v->EmptyThrottle ) &&
			 ( v->ThrottleRawIn <= ( ( v->EmptyThrottle + v->FullThrottle )* 0.5f ) ) )
	{
		v->PercentageTarget = ( v->ThrottleRawIn - v->EmptyThrottle ) * v->HalfThrottle /
				           ( v->FullThrottle - v->EmptyThrottle )* 2.0f;
	}
	else if( ( v->ThrottleRawIn > ( ( v->EmptyThrottle + v->FullThrottle )* 0.5f ) ) &&
			 ( v->ThrottleRawIn <= v->FullThrottle ) )
	{
		v->PercentageTarget = ( v->ThrottleRawIn - ( v->EmptyThrottle + v->FullThrottle )* 0.5f )*
				           ( 1.0f - v->HalfThrottle )/ ( v->FullThrottle - v->EmptyThrottle ) * 2.0f +
						   v->HalfThrottle;
	}
	else
	{
		v->PercentageTarget = 1.0f;
	}
}

void ThrottleMapping_Ramp( ThrottleMapping_t *v )
{
	if( v->PercentageTarget > v->PercentageOut )
	{
		v->PercentageOut = v->PercentageOut + ( v->ThrottleTnTab[ v->TnSelect ].ThrotRiseRamp * 0.1f);
		if( v->PercentageTarget < v->PercentageOut )
		{
			v->PercentageOut = v->PercentageTarget;
		}
	}
	else
	{
//		v->PercentageOut = v->PercentageTarget;
		v->PercentageOut = v->PercentageOut - ( v->ThrottleTnTab[ v->TnSelect ].ThrotFallRamp * 0.1f);
		if( v->PercentageTarget > v->PercentageOut )
		{
			v->PercentageOut = v->PercentageTarget;
		}
	}

	if( v->PercentageOut > 1.0f )
	{
		v->PercentageOut = 1.0f;
	}
	else if( v->PercentageOut < 0.0f )
	{
		v->PercentageOut = 0.0f;
	}
}

void ThrottleMapping_Reset( ThrottleMapping_t *v )
{
	v->PercentageOut = 0.0;
	v->ThrottleIn = 0.0;
	v->ThrottleDI = 1;
	v->ThrottleRawIn = 0.0;
}
