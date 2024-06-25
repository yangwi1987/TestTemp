/*
 * FunctionGen.c
 *
 *  Created on: 2024年6月25日
 *      Author: Jeff C
 */


#include "FunctionGen.h"


void FunctionGenerator_Init(FG_t* v, float Start_Freq, float End_Freq, float duration, float ExePeriod, float Amplitude, float offset)
{

	v->Freq_Step = (( End_Freq - Start_Freq ) / duration ) * ExePeriod;
	v->Total_Step = (uint32_t)( duration / ExePeriod );
	v->Offset = offset;
	v->Amplitude = Amplitude;
	v->ExePeriod = ExePeriod;
	v->Direction = End_Freq >= Start_Freq ? 1 : 0;
	v->StartFreq = Start_Freq;
	v->EndFreq = End_Freq;
	v->Freq_Now = v->StartFreq;
	switch ( v->Signal_Select )
	{
	    case FG_SIGNAL_SELECT_SINE:
	    {
	    	v->Calc = (functypeFunctionGenerator_Calc)&FunctionGenerator_Sin_Calc;
	    	break;
	    }
	    case FG_SIGNAL_SELECT_SQUARE:
	    {
	    	v->Calc = (functypeFunctionGenerator_Calc)&FunctionGenerator_Square_Calc;
	    	break;
	    }
	    case FG_SIGNAL_SELECT_TRIANGLE:
	    {
	    	v->Calc = (functypeFunctionGenerator_Calc)&FunctionGenerator_Triangle_Calc;
	    	break;
	    }
	    default:
	    {
	    	v->Calc = (functypeFunctionGenerator_Calc)&FunctionGenerator_None_Calc;
	    	break;
	    }
	}

}

float FunctionGenerator_Sin_Calc( FG_t* v )
{
	float result = 0.0f;
	if ( v->Start == 1 )
	{
		float temp_pos = 0.0f;
		float t = 0.0f;
		uint16_t quotient = 0;
		t = (float)v->Step_Now * v->ExePeriod;
		quotient = (uint16_t)(v->Freq_Now * t);//2PI * INV_2PI
		temp_pos = ( _2PI * v->Freq_Now * t ) - (float)( quotient * _2PI );
	    v->Signal_Now = sin_LT[(uint16_t)(temp_pos * 4096.0f * INV_2PI + 0.5f)];
		v->Step_Now++;
		if ( v->Direction == 1 )
		{
		    v->Freq_Now = v->Freq_Now + v->Freq_Step;

		}
		else
		{
			v->Freq_Now = v->Freq_Now - v->Freq_Step;
		}

		if ( v->Step_Now >= v->Total_Step )
		{
			switch ( v->Pattern )
			{
			    case FG_SIGNAL_PATTERN_ONCE:
			    {
			    	v->Start = 0;
			    	v->Freq_Now = v->StartFreq;
			    	break;
			    }
			    case FG_SIGNAL_PATTERN_REPEAT:
			    {
			    	v->Step_Now = 0;
			    	v->Freq_Now = v->StartFreq;
			    	break;
			    }
			    case FG_SIGNAL_PATTERN_UPDOWN:
			    {
			    	v->Step_Now = 0;
			    	v->Freq_Now = v->EndFreq;
			    	v->Direction = !v->Direction;
			    	break;
			    }
			    default:
			    {
			    	break;
			    }
			}

		}

	}
	else
	{
		v->Freq_Now = v->StartFreq;
		v->Step_Now = 0;
	}

	result = v->Offset + v->Signal_Now * v->Amplitude;
    return result;
}

float FunctionGenerator_Square_Calc( FG_t* v )
{
return 0;
}
float FunctionGenerator_Triangle_Calc( FG_t* v )
{
	return 0;
}
float FunctionGenerator_None_Calc( FG_t* v )
{
//nothing happen
	return 0;
}
