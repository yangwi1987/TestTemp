/*
 * TorqueToIdq.c
 *
 *  Created on: 2020年4月9日
 *      Author: Fernando.Wang.HHW
 */
#include "MotorControl.h"
#include "UtilityBase.h"
#define EXTENSION_LINE_RES 0.008f

void TorqueToIdq_GetIdqCmd( TORQUE_TO_IDQ_TYPE *p, float TorqueTarget, float AllowFluxRec )
{
	uint16_t Way = 0;
	Way = p->Way;
	if ( Way == GET_IDQ_BY_LUT)
	{
		float IdTmp = 0.0f;
		float IqTmp = 0.0f;
		float MaxTorqueTmp = 0.0f;
		float TorqueTargetSign = 1.0f;
		float TorqueTargetAbs = 1.0f;

		TorqueTargetSign = ( TorqueTarget >= 0.0f) ? 1.0f : -1.0f;
		TorqueTargetAbs = TorqueTargetSign * TorqueTarget;
		MaxTorqueTmp = p->MaxTorqueLut.Calc(&(p->MaxTorqueLut), AllowFluxRec);
		if ( TorqueTargetAbs > MaxTorqueTmp ) TorqueTargetAbs = MaxTorqueTmp;
		if ( TorqueTargetAbs < 0.0f ) TorqueTargetAbs = 0.0f;
		p->OutputTqRatio = ( MaxTorqueTmp > 0.1f )? ( TorqueTargetAbs / MaxTorqueTmp * 100.0f ) : 100.0f ;

		IdTmp = p->IdCmdLut.Calc(&(p->IdCmdLut),TorqueTargetAbs, AllowFluxRec);
		IqTmp = p->IqCmdLut.Calc(&(p->IqCmdLut),TorqueTargetAbs, AllowFluxRec);
		p->IdCmd = IdTmp;
		p->IqCmd = IqTmp * TorqueTargetSign;
	}
	else
	{
		//do nothing
	}
}

void TorqueToIdq_GetAllowFluxRec( TORQUE_TO_IDQ_TYPE *p, float EleSpeedInput, float Vbus, float MaxDuty, float *AllowFluxRec1, float *AllowFluxRec2 )
{
	float AllowFluxRecTmp = 0.0f;
	float EleSpeed = 0.0f;
	float VphaseUsed = 0.0f;
	float VbusUsedTmp = 0.0f;
	EleSpeed = EleSpeedInput;
	p->VbusReal = Vbus;
	VbusUsedTmp = p->VbusReal;

	if ( VbusUsedTmp <= GUARANTEED_VBUS_FOR_PERFORMANCE )
	{
		p->VbusUsed = VbusUsedTmp * p->VbusGain;
	}
	else
	{
		p->VbusUsed = GUARANTEED_VBUS_FOR_PERFORMANCE * p->VbusGain;
	}
	VphaseUsed = p->VbusUsed * DIVIDE_SQRT3;

	AllowFluxRecTmp = EleSpeed / VphaseUsed;



	AllowFluxRecTmp = ( AllowFluxRecTmp >= 0.0f ) ? AllowFluxRecTmp : -AllowFluxRecTmp;
	(*AllowFluxRec1) = AllowFluxRecTmp;
	(*AllowFluxRec2) = AllowFluxRecTmp;

}

