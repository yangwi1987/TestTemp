/*
 * CoordinateTransfer.c
 *
 *  Created on: 2019年12月2日
 *      Author: Fernando
 */

#include <MotorControl.h>

void CoordinateTransfer_PhaseToStatorClean( PHASE_TO_STATOR_TYPE* p)
{
	p->Alpha=0.0f;
	p->Beta=0.0f;
}

void CoordinateTransfer_StatorToRotorClean( STATOR_TO_ROTOR_TYPE* p)
{
	p->D=0.0f;
	p->Q=0.0f;
}

void CoordinateTransfer_RotorToStatorClean( ROTOR_TO_STATOR_TYPE* p)
{
	p->Alpha=0.0f;
	p->Beta=0.0f;
}

void CoordinateTransfer_StatorToPhaseClean( STATOR_TO_PHASE_TYPE* p)
{
	p->U=0.0f;
	p->V=0.0f;
	p->W=0.0f;
}
