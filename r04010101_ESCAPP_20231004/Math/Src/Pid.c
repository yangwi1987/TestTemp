/*
 * Pid.c
 *
 *  Created on: 2019年12月4日
 *      Author: Fernando
 */
#include <MathFunctionInclude.h>

uint16_t Pid_PiInit( float Period, float Kp, float Ki, float UpperLimit, float LowerLimit, PI_TYPE* p )
{
	Pid_PiClean( p );
	if ( Period <= 0.0f )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->UpperLimit = 0.0f;
		p->LowerLimit = 0.0f;
		return PID_INIT_ERROR_PERIOD_IS_NOT_NEGATIVE;
	}
	if ( LowerLimit > UpperLimit )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->UpperLimit = 0.0f;
		p->LowerLimit = 0.0f;
		return 	PID_INIT_ERROR_LOWER_LIMIT_IS_LARGER_UPPER_LIMIT;
	}
	if ( Kp < 0.0f )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->UpperLimit = 0.0f;
		p->LowerLimit = 0.0f;
		return 	PID_INIT_ERROR_KP_IS_NAGATIVE;
	}
	if ( Ki < 0.0f )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->UpperLimit = 0.0f;
		p->LowerLimit = 0.0f;
		return 	PID_INIT_ERROR_KI_IS_NAGATIVE;
	}

	p->Period = Period;
	p->Kp = Kp;
	p->Ki = Ki;
	p->UpperLimit = UpperLimit;
	p->LowerLimit = LowerLimit;

	return PID_INIT_OK;
}

uint16_t Pid_PiComplexVectorInit( float Period, float Kp, float Ki, float Kij, uint8_t UseAntiWindup, PI_COMPLEX_VECTOR_TYPE* p )
{
	Pid_PiComplexVectorClean( p );
	if ( Period <= 0.0f )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->Kij = 0.0f;
		return PID_INIT_ERROR_PERIOD_IS_NOT_NEGATIVE;
	}

	if ( Kp < 0.0f )
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->Kij = 0.0f;
		return 	PID_INIT_ERROR_KP_IS_NAGATIVE;
	}
	if (( Ki < 0.0f ) || ( Kij < 0.0f ))
	{
		p->Period = 0.0f;
		p->Kp = 0.0f;
		p->Ki = 0.0f;
		p->Kij = 0.0f;
		return 	PID_INIT_ERROR_KI_IS_NAGATIVE;
	}

	p->Period = Period;
	p->Kp = Kp;
	p->Ki = Ki;
    p->Kij = Kij;
    p->UseAntiWindup = UseAntiWindup;

	return PID_INIT_OK;
}

void Pid_PiClean( PI_TYPE* p )
{
	p->Up = 0.0f;
	p->Ui = 0.0f;
	p->Output = 0.0f;
	p->Error = 0.0f;
}

void Pid_PiComplexVectorClean( PI_COMPLEX_VECTOR_TYPE* p )
{
	p->Up = 0.0f;
	p->Uij = 0.0f;
	p->Output_Raw = 0.0f;
	p->Output = 0.0f;
	p->Error = 0.0f;
}

void Pid_PiControl( float Command, float Feedback, PI_TYPE* p )
{
	p->Error = Command - Feedback;

	p->Up = p->Kp * p->Error;

	p->Ui += ( p->Ki * p->Error * p->Period );
	p->Ui = ( p->Ui > p->UpperLimit ) ? p->UpperLimit : p->Ui;
	p->Ui = ( p->Ui < p->LowerLimit ) ? p->LowerLimit : p->Ui;

	p->Output = p->Up + p->Ui;
}

void Pid_PiControlNoLimit( float Command, float Feedback, PI_TYPE* p )
{
	p->Error = Command - Feedback;

	p->Up = p->Kp * p->Error;

	p->Ui += ( p->Ki * p->Error * p->Period );

	p->Output = p->Up + p->Ui;
}
