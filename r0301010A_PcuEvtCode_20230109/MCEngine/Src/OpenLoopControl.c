/*
 * OpenLoopControl.c
 *
 *  Created on: 2019年12月13日
 *      Author: Fernando
 */

#include "MotorControl.h"


/********************************************************************************************************************************
   	   	   	   	   	   	   	   	   	   	   	   	   Virtual Position Start
*********************************************************************************************************************************/
void OpenLoopControl_OpenLoopControlCleanVirtualPosition( VIRTUAL_POSITION_TYPE *p )
{
	p->VirtualEleAngle = 0.0f;
	p->VirtualEleSpeed = 0.0f;
	p->Rpm = 0.0f;
}

uint16_t OpenLoopControl_OpenLoopControlInitVirtualPosition( VIRTUAL_POSITION_TYPE *p, float Period, float RpmAccel, float RpmDecel, float Polepair )
{
	p->RpmToEleSpeedGain = RPM_TO_SPEED * Polepair;
	OpenLoopControl_OpenLoopControlCleanVirtualPosition( p );

	p->Period = 0.0f;
	p->RpmAccel = 0.0f;
	p->RpmDecel = 0.0f;
	if ( Period <= 0.0f)
	{
		return VIRTUAL_POSITION_INIT_ERROR_PERIOD_IS_NOT_POSITIVE;
	}
	else
	{
		p->Period = Period;
	}
	if ( RpmAccel <= 0.0f )
	{
		return VIRTUAL_POSITION_INIT_ERROR_ACCEL_IS_NOT_POSITIVE;
	}
	else
	{
		p->RpmAccel = RpmAccel;
	}
	if ( RpmDecel <= 0.0f )
	{
		return VIRTUAL_POSITION_INIT_ERROR_DECEL_IS_NOT_POSITIVE;
	}
	else
	{
		p->RpmDecel = RpmDecel;
	}

	return OPEN_LOOP_CONTROL_INIT_OK;
}

void OpenLoopControl_OpenLoopControlCalcVirtualPosition( VIRTUAL_POSITION_TYPE *p, float RpmTarget )
{
	if ( RpmTarget >= 0 )
	{
		if ( p->Rpm >= 0 )
		{
			if ( RpmTarget > p->Rpm )
			{
				(p->Rpm) = Ramp( p->Rpm, RpmTarget, ( p->RpmAccel * p->Period ) );
			}
			else
			{
				(p->Rpm) = Ramp( p->Rpm, RpmTarget, ( p->RpmDecel * p->Period ) );
			}
		}
		else
		{
			(p->Rpm) = Ramp( p->Rpm, 0.0f, ( p->RpmDecel * p->Period ) );
		}
	}
	else
	{
		if (p->Rpm <= 0)
		{
			if ( RpmTarget < p->Rpm )
			{
				(p->Rpm) = Ramp( p->Rpm, RpmTarget, ( p->RpmAccel * p->Period ) );
			}
			else
			{
				(p->Rpm) = Ramp( p->Rpm, RpmTarget, ( p->RpmDecel * p->Period ) );
			}
		}
		else
		{
			(p->Rpm) = Ramp( p->Rpm, 0.0f, ( p->RpmDecel * p->Period ) );
		}
	}

	p->VirtualEleSpeed = p->Rpm * p->RpmToEleSpeedGain;
	p->VirtualEleAngle += ( p->VirtualEleSpeed * p->Period);

	//Get the corresponding angle
	p->VirtualEleAngle = ( p->VirtualEleAngle > _2PI) ? (p->VirtualEleAngle - _2PI) : p->VirtualEleAngle;
	p->VirtualEleAngle = ( p->VirtualEleAngle < 0) ? (p->VirtualEleAngle + _2PI) : p->VirtualEleAngle;

}
/********************************************************************************************************************************
*********************************************************************************************************************************
*******************************************************Virtual PositionEnd*******************************************************
*********************************************************************************************************************************
*********************************************************************************************************************************/


/********************************************************************************************************************************
   	   	   	   	   	   	   	   	   	   	   	   	   IF Control Start
*********************************************************************************************************************************/
void OpenLoopControl_IfControlClean( IF_CONTROL_TYPE *p )
{
	OpenLoopControl_OpenLoopControlCleanVirtualPosition( &(p->Position) );
	p->CurrAmp = 0.0f;
}

uint16_t OpenLoopControl_IfControlInit( IF_CONTROL_TYPE *p, float Period, float Gain, float RpmAccel, float RpmDecel, float Polepair )
{
	uint16_t Status;
	OpenLoopControl_IfControlClean( p );
	p->Gain = 0.0f;
	Status = OpenLoopControl_OpenLoopControlInitVirtualPosition( &(p->Position), Period, RpmAccel, RpmDecel, Polepair );
	if( Status != 0)
	{
		return Status;
	}
	else
	{
		if ( Gain <= 0.0f )
		{
			return IF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE;
		}
		else
		{
			p->Gain = Gain;
		}
	}
	return OPEN_LOOP_CONTROL_INIT_OK;
}

void OpenLoopControl_IfControlCalcCmd( IF_CONTROL_TYPE *p, float RpmTarget )
{
	OpenLoopControl_OpenLoopControlCalcVirtualPosition( &(p->Position), RpmTarget );
	p->CurrAmp = p->Gain * p->Position.Rpm;
	p->CurrAmp = ( p->CurrAmp > p->CurrLimit ) ? p->CurrLimit : p->CurrAmp;
	p->CurrAmp = ( p->CurrAmp < -(p->CurrLimit) ) ? -(p->CurrLimit) : p->CurrAmp;
}
/********************************************************************************************************************************
*********************************************************************************************************************************
*******************************************************IF Control End************************************************************
*********************************************************************************************************************************
*********************************************************************************************************************************/

/********************************************************************************************************************************
   	   	   	   	   	   	   	   	   	   	   	   	   IF Control Start
*********************************************************************************************************************************/
void OpenLoopControl_VfControlClean( VF_CONTROL_TYPE *p )
{
	OpenLoopControl_OpenLoopControlCleanVirtualPosition( &(p->Position) );
	p->VoltAmp = 0.0f;
}

uint16_t OpenLoopControl_VfControlInit( VF_CONTROL_TYPE *p, float Period, float Gain, float RpmAccel, float RpmDecel, float Polepair )
{
	uint16_t Status;
	OpenLoopControl_VfControlClean( p );
	p->Gain = 0.0f;
	Status = OpenLoopControl_OpenLoopControlInitVirtualPosition( &(p->Position), Period, RpmAccel, RpmDecel, Polepair );
	if( Status != 0)
	{
		return Status;
	}
	else
	{
		if ( Gain <= 0.0f )
		{
			return VF_CONTROL_INIT_ERROR_GAIN_IS_NOT_POSITIVE;
		}
		else
		{
			p->Gain = Gain;
		}
	}
	return OPEN_LOOP_CONTROL_INIT_OK;
}

void OpenLoopControl_VfControlCalcCmd( VF_CONTROL_TYPE *p, float RpmTarget )
{
	OpenLoopControl_OpenLoopControlCalcVirtualPosition( &(p->Position), RpmTarget );
	p->VoltAmp = p->Gain * p->Position.Rpm;
}
/********************************************************************************************************************************
*********************************************************************************************************************************
*******************************************************IF Control End************************************************************
*********************************************************************************************************************************
*********************************************************************************************************************************/
