/*
 * MotorStall.c
 *
 *  Created on: 2020年8月17日
 *      Author: Mike.Wen.SFW
 */

#include "MotorStall.h"

void MotorStall_Init( MotorStall_t *v )
{
	v->LastAccHeatEnergy = 0;
	v->AccHeatEnergy = 0;
}

void MotorStall_Calc( MotorStall_t *v, float ACCurrent, float MotorRPM )
{
	float HeatEnergy = 0.0;

	if( MotorRPM >= MOTORSTALL_BEGIN_RPM )
	{
		v->AccHeatEnergy *= 0.631f; // decay to 1 % after 100 ms
	}
	else
	{
		HeatEnergy = (ACCurrent * ACCurrent) * 0.07344; // max phase current x phase resistance x update period(ms)
		v->AccHeatEnergy = v->LastAccHeatEnergy + HeatEnergy;
		v->AccHeatEnergy *= 0.994f; // if phase current is 69 Apeak, then no motor stall alarm will occur.
	}

	if( v->AccHeatEnergy >= 59117 ) // 59117 = (259 Apeak)^2 * (0.00734 ohm) * (120 ms) todo
	{
		v->IsMotorStall = 1;
	}

	v->LastAccHeatEnergy = v->AccHeatEnergy;
}

void MotorStall_Reset( MotorStall_t *v )
{
	v->IsMotorStall = 0;
	v->LastAccHeatEnergy = 0;
	v->AccHeatEnergy = 0;
}
