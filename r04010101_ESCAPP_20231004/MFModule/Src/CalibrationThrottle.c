/*
 * CalibThrottle.c
 *
 *  Created on: 2020年12月15日
 *      Author: Hank.Chen.CHC
 */

#include "CalibrationThrottle.h"

void ReadModeAndAuthority( THROT_CALIB_STRUCT_DEFINE *v, uint16_t Mode, uint16_t Auth, uint16_t ServoOnSate )
{
	v->Authority = Auth;
	if( ServoOnSate == MF_SERVO_ON )
	{
		v->CtrlMode = MF_CLEAR;
	}
	else
	{
		v->CtrlMode = Mode;
	}
}

void ReadAdcValueOfThrottle( THROT_CALIB_STRUCT_DEFINE *v, AdcStation *a, uint32_t *Enable )
{
	uint16_t SelectBuf = *Enable;
	if( ( v->Authority < Mfsa_LscMf ) && ( v->CtrlMode != MF_CALIB_THROT_MODE ) )
		return;

	if( *Enable > 0 )
	{
		v->AccumulatorCnt++;
		if( v->AccumulatorCnt == ACC_NUM )
		{
			v->ThrottleAdValue[ SelectBuf - 1 ] = ( uint16_t )( v->AccumulatorADC >> SHIFT_BIT );
			v->AccumulatorADC = MF_CLEAR;
			v->AccumulatorCnt = MF_CLEAR;
			*Enable = MF_DISABLE;
		} else;
	} else;
}

void CalcThrottleGain( THROT_CALIB_STRUCT_DEFINE *v, uint32_t *Enable )
{
//	if( ( v->Authority < Mfsa_LscMf ) && ( v->CtrlMode != MF_CALIB_THROT_MODE ) )
//		return;
//	if( *Enable == MF_ENABLE )
//	{
//		v->CalibrationGain.FDta = ( float )( 1.0f /( float )( v->ThrottleAdValue[ MF_HIGH ] - v->ThrottleAdValue[MF_LOW] ) );
//
//		DriveParams.SystemParams.ThrottleMinAdc = v->ThrottleAdValue[ MF_LOW ];
//		DriveParams.SystemParams.ThrottleMaxAdc = v->ThrottleAdValue[ MF_HIGH ];
//		DriveParams.SystemParams.ThrottleScale[ MF_LOW ] = v->CalibrationGain.Uint16Type[ MF_LOW ];
//		DriveParams.SystemParams.ThrottleScale[ MF_HIGH ] = v->CalibrationGain.Uint16Type[ MF_HIGH ];
//		*Enable = MF_DISABLE;
//	}else;
}
