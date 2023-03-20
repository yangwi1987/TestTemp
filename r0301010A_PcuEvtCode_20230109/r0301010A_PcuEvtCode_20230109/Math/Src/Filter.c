/*
 * Filter.c
 *
 *  Created on: Mar 11, 2020
 *      Author: Fernando
 */
#include "MathFunctionInclude.h"
#include "Constant.h"

void Filter_Bilinear1OrderClean(FILTER_BILINEAR_1ORDER_TYPE *p)
{
	p->PreviousX = 0.0f;
	p->PreviousY = 0.0f;
	p->Times = FILTER_TIMES_FIRST;
}

uint16_t Filter_Bilinear1OrderInit(FILTER_BILINEAR_1ORDER_TYPE *p, FILTER_INIT_BILINEAR_1ORDER_TYPE *pSetting)
{
	float w=0.0f;
	float a=0.0f;

	if ( pSetting->Period <= 0.0f )
	{
		return FILTER_INIT_ERROR_PERIOD_IS_NOT_POSITIVE;
	}
	if ( pSetting->BandwithHz <= 0.0f )
	{
		return FILTER_INIT_ERROR_BANDWITH_IS_NOT_POSITIVE;
	}
	if (( pSetting->Type != FILTER_TYPE_LPF ) && ( pSetting->Type != FILTER_TYPE_HPF ))
	{
		return FILTER_INIT_ERROR_TYPE_IS_NOT_DEFINED;
	}

	Filter_Bilinear1OrderClean(p);
	p->Type = pSetting->Type;
	w = _2PI * pSetting->BandwithHz;
	a = 2 / pSetting->Period;
	p->Gain = w / ( a + w );
	return 0;
}

float Filter_Bilinear1OrderCalc(FILTER_BILINEAR_1ORDER_TYPE *p, float X)
{
	float Y=0.0f;
	if ( p->Times == FILTER_TIMES_FIRST)
	{
		Y = 0.0f;
		p->Times = FILTER_TIMES_ABOVE_TWICE;
	}
	else
	{
		Y = p->Gain * ( X + p->PreviousX ) + ( 1 - 2 * p->Gain ) * p->PreviousY;
	}
	p->PreviousY = Y;
	p->PreviousX = X;
	if ( p->Type == FILTER_TYPE_LPF )
	{

	}
	else	//p->Type == TYPE_HPF
	{
		Y = X - Y;
	}
	return Y;
}


void ZohBandPass_OnLineGain(FILTER_ZOH_BANDPASS_TYPE *p,float Rad)
{
	float Angle = 0.0f;
	float SinValue=0.0f;
	float CosValue=1.0f;
	float TmpSqrt=0.0f;
	int16_t AngleInt;
	TmpSqrt = ( Rad < p->GainConst1 ) ? p->GainConst1 : sqrtf( Rad * Rad - p->GainConst1 * p->GainConst1 );
	TmpSqrt = (TmpSqrt<=1.0f) ? 1.0f : TmpSqrt;

	Angle = TmpSqrt * p->Period;
//	CordicMath_GetSinCosValue_Macro(Angle,SinValue,CosValue);
	AngleInt = (int16_t)(Angle * DIVIDE_2PI);
	Angle -= ((float)AngleInt) * _2PI;
	SinValue = sinf(Angle);
	CosValue = cosf(Angle);

	p->Gain3 = 2.0f * CosValue * p->GainConst2;
	p->Gain2 = -2.0f * p->GainConst1 * p->GainConst2 * SinValue / TmpSqrt;
	p->Gain1 = -p->Gain2;

}

void ZohBandPass_InitGain(FILTER_ZOH_BANDPASS_TYPE *p,float Period, float Fc, float F0)
{
	p->Period = Period;
	p->GainConst1 = Fc*_2PI;
	p->GainConst2 = expf(-p->Period * p->GainConst1);
	p->Gain4 = -expf(-p->Period * p->GainConst1 * 2.0f);
	ZohBandPass_OnLineGain(p,F0*_2PI);
}

float ZohBandPass_Calc(FILTER_ZOH_BANDPASS_TYPE *p,float Input)
{
	float Output = 0.0f;
	Output = p->Gain1 * p->XPrevious + p->Gain2 * p->XPrevious2 + p->Gain3 * p->YPrevious + p->Gain4 * p->YPrevious2;
	p->XPrevious2=p->XPrevious;
	p->YPrevious2=p->YPrevious;
	p->XPrevious=Input;
	p->YPrevious=Output;
	return Output;
}

void MovingAverage_Clean( MovingAverage_t *v )
{
	int i = 0;
	for(i = 0; i < MAX_COUNT_N; i++)
	{
		v->LastInput[i] = 0;
	}
	v->InputSum = 0;
	v->Index = 0;
	v->OutFilteredData = 0;
}

void MovingAverage_Init( MovingAverage_t *v, uint16_t Number )
{
	MovingAverage_Clean( v );
	v->MovAvgNumber = Number;

	if( v->MovAvgNumber >= MAX_COUNT_N )
	{
		v->MovAvgNumber = MAX_COUNT_N - 1; // max legal number is Max count number - 1.
	}
}

void MovingAverage_Calc( MovingAverage_t *v, float Input)
{
	// disable filter
	if( v->MovAvgNumber <= 1 )
	{
		v->OutFilteredData = Input;
	}
	// normal setting
	else
	{
		v->LastInput[v->Index] = Input;
		if( v->Index == v->MovAvgNumber )
		{
			v->InputSum = v->InputSum - v->LastInput[0] + v->LastInput[v->Index];
			v->Index = 0;
		}
		else
		{
			v->InputSum = v->InputSum - v->LastInput[(v->Index + 1)] + v->LastInput[v->Index];
			v->Index++;
		}
		v->OutFilteredData = v->InputSum / v->MovAvgNumber;
	}
}

void Filter_BilinearBPFClean( FilterBilinearBPF_t *p )
{
	p->XPre1 = 0.0f;
	p->XPre2 = 0.0f;
	p->YPre1 = 0.0f;
	p->YPre2 = 0.0f;
	p->Times = 0;
}

uint16_t Filter_BilinearBPFInit( FilterBilinearBPF_t *p, FilterBilinearBPFSetting_t *pSetting )
{
	float w=0.0f;
	float a=0.0f;
	float GainD0=0;
	float GainD1=0;
	float GainD2=0;
	float GainN0=0;
	float GainN2=0;


	if( pSetting->Period <= 0.0f )
	{
		return FILTER_INIT_ERROR_PERIOD_IS_NOT_POSITIVE;
	}
	if( pSetting->BandwithHz <= 0.0f )
	{
		return FILTER_INIT_ERROR_BANDWITH_IS_NOT_POSITIVE;
	}
	if( pSetting->Q <= 0.0f )
	{
		return FILTER_INIT_ERROR_Q_IS_NOT_POSITIVE;
	}

	Filter_BilinearBPFClean(p);
	w = _2PI * pSetting->BandwithHz;
	a = w /tanf( w * pSetting->Period / 2);
	GainD0 = ( a * a  - w / pSetting->Q * a + w * w );
	GainD1 = ( -2 * a * a + 2 * w * w );
	GainD2 = ( a * a  + w / pSetting->Q * a + w * w );
	GainN0 = ( - w / pSetting->Q * a );
	GainN2 = ( w / pSetting->Q * a );

	p->GainYPre1 = -GainD1 / GainD2;
	p->GainYPre2 = -GainD0 / GainD2;
	p->GainX = GainN2 / GainD2;
	p->GainXPre2 = GainN0 / GainD2;

	return 0;
}

float Filter_BilinearBPFCalc( FilterBilinearBPF_t *p, float X )
{
	if ( p->Times < 2 )
	{
		p->Out = X;
		p->Times++;
	}
	else
	{
		p->Out = p->GainYPre1 * p->YPre1 + p->GainYPre2 * p->YPre2 + p->GainX * X + p->GainXPre2 * p->XPre2;
	}
	p->XPre2 = p->XPre1;
	p->XPre1 = X;
	p->YPre2 = p->YPre1;
	p->YPre1 = p->Out;
	return p->Out;
}
