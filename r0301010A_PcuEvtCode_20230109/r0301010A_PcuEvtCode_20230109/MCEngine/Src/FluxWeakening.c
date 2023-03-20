/*
 * FluxWeakening.c
 *
 *  Created on: 2019年12月13日
 *      Author: Fernando
 */
#include "MotorControl.h"

uint16_t MotorControl_FluxWeakeningForCurrentControlInit( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p, float Ilimit, float IdMin, float Period, float Kp, float Ki)
{
	uint16_t Status = 0;
	float IdMinTmp = 0.0f;
	p->Ilimit = Ilimit;
	if ( p->Ilimit <= 0.0f )
	{
		Status |= FLUX_WEAKENING_INIT_ERROR_ILIMIT_IS_NOT_POSITIVE;
		p->Ilimit = 0.0f;
	}
	else
	{
		Status |= FLUX_WEAKENING_INIT_OK;
	}
	IdMinTmp = IdMin;
	IdMinTmp = ( IdMinTmp < -p->Ilimit ) ? -p->Ilimit : IdMinTmp;
	p->FluxWeakeningRegulator.Init( Period, Kp, Ki, 0.0f, IdMinTmp, &(p->FluxWeakeningRegulator) );
	MotorControl_FluxWeakeningForCurrentControlClean( p );
	return Status;
}

void MotorControl_FluxWeakeningForCurrentControlClean( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p )
{
	p->IdCmd = 0.0f;
	p->IqCmd = 0.0f;
	p->IdComp = 0.0f;
	p->FluxWeakeningRegulator.Clean( &(p->FluxWeakeningRegulator) );
}

void MotorControl_FluxWeakeningForCurrentControlCalc( CURRENT_CONTROL_FLUX_WEAKENING_TYPE *p, float VcmdAmp, float VbusLimit, float OrgIdCmd, float OrgIqCmd)
{
	//It's replaced by MOTOR_CONTROL_FLUX_WEAKENING_FOR_CURRENT_CONTROL
}
