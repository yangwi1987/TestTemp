/*
 * TorqueCommandGenerator.c
 *
 *  Created on: Dec 4, 2019
 *      Author: MikeSFWen
 */

#include <TorqCommandGenerator.h>
#define MIN3(a,b,c)  (((( a > b ) ? b : a) > c ) ? c : ( a > b ? b : a ))

float TorqCommandGenerator_DCLimitation( TorqCommandGenerator_t *v )
{
	float DcCurrLimit = 0;
	float Torque = 0.0f;
	DcCurrLimit = ( v->DcCurrLimit >= 0.0f ) ? v->DcCurrLimit : 0.0f;
	Torque = v->DcCurrLimitLut.Calc( &(v->DcCurrLimitLut), DcCurrLimit, v->AllowFluxRec );
	return Torque;
}

float TorqCommandGenerator_ACLimitation( TorqCommandGenerator_t *v )
{
	float AcCurrLimitAbs = 0;
	float AcCurrLimitSign = 1.0f;
	float Torque = 0.0f;
	AcCurrLimitSign = ( v->AcCurrLimit >= 0.0f ) ? 1.0f : -1.0f;
	AcCurrLimitAbs = v->AcCurrLimit * AcCurrLimitSign;
	Torque = v->AcCurrLimitLut.Calc( &(v->AcCurrLimitLut), AcCurrLimitAbs, v->AllowFluxRec);
	Torque *= AcCurrLimitSign;
	return Torque;
}

void TorqCommandGenerator_Calc( TorqCommandGenerator_t *v, FourQuadControl *p, float ThrottleCmd )
{
	float TorqueOut = 0.0f;
	float FourQuadTorqueCommandAbs = 0.0f;
	float DCLimitedTorqueCommandAbs = 0.0f;
	float ACLimitedTorqueCommandAbs = 0.0f;

	// step 1: calc AC and DC limitation Torque command
	v->DCLimitedTorqueCommand = TorqCommandGenerator_DCLimitation(v);
	v->ACLimitedTorqueCommand = TorqCommandGenerator_ACLimitation(v);

	// step 2: find minimum torque command
	FourQuadTorqueCommandAbs = p->TorqueCommandOut;
	DCLimitedTorqueCommandAbs = v->DCLimitedTorqueCommand;
	ACLimitedTorqueCommandAbs = v->ACLimitedTorqueCommand;
	TorqueOut = MIN3(FourQuadTorqueCommandAbs, DCLimitedTorqueCommandAbs, ACLimitedTorqueCommandAbs);

	// Get TNTorqueAbs、ACToruqeAbs、DCTorqueAbs
	v->TNTorqueAbs = ( FourQuadTorqueCommandAbs > v->TorqueCommandMax ) ? v->TorqueCommandMax : FourQuadTorqueCommandAbs;
	v->ACTorqueAbs = ACLimitedTorqueCommandAbs;
	v->DCTorqueAbs = DCLimitedTorqueCommandAbs;

	// step 3: ThrottleCmdPercentage
	TorqueOut = TorqueOut * ThrottleCmd;


	// step 4: Clamp Torque Command
	if( TorqueOut > v->TorqueCommandMax )
	{
		TorqueOut = v->TorqueCommandMax;
	}
	else if( TorqueOut < v->TorqueCommandmin )
	{
		TorqueOut = v->TorqueCommandmin;
	}

	// step 5: TorqueOut
	v->Out = TorqueOut;
}
