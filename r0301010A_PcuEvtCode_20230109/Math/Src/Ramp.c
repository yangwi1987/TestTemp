/*
 * Ramp.c
 *
 *  Created on: 2019年12月18日
 *      Author: Fernando
 */

#include "MathFunctionInclude.h"

float Ramp( float Value, float Target, float Slope)
{
	if ( Target > Value )
	{
		Value += Slope;
		Value = ( Value >= Target ) ? Target : Value;
	}
	else if ( Target < Value )
	{
		Value -= Slope;
		Value = ( Value <= Target ) ? Target : Value;
	}
	else
	{
		//do nothing
	}
	return Value;
}
