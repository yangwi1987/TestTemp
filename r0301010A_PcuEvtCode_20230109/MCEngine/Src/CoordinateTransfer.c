/*
 * CoordinateTransfer.c
 *
 *  Created on: 2019年12月2日
 *      Author: Fernando
 */

#include <MotorControl.h>

void CoordinateTransfer_PhaseToStator( float U, float V, float W, PHASE_TO_STATOR_TYPE* p)
{
	// It's replaced by COORDINATE_TRANSFER_PHASE_TO_STATOR_MACRO
}

void CoordinateTransfer_StatorToRotor( float Alpha, float Beta, float Angle, STATOR_TO_ROTOR_TYPE* p)
{
	// It's replaced by COORDINATE_TRANSFER_STATOR_TO_ROTOR_MACRO
}

void CoordinateTransfer_RotorToStator( float D, float Q, float Angle, ROTOR_TO_STATOR_TYPE* p)
{
	// It's replaced by COORDINATE_TRANSFER_ROTOR_TO_STATOR_MACRO
}

void CoordinateTransfer_StatorToPhase( float Alpha, float Beta, STATOR_TO_PHASE_TYPE* p)
{
	// It's replaced by COORDINATE_TRANSFER_STATOR_TO_PHASE_MACRO
}

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
