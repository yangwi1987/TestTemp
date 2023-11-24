/*
 * MotorControlParamDefault.c
 *
 *  Created on: 2020年2月25日
 *      Author: MikeSFWen
 */

#include "MotorControl.h"
#include "Motor_Init_Table.h"
#include "UtilityBase.h"
#include "ConstantParamAndUseFunction.h"

extern const MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE MotorDefault;

const MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE MotorDefault =
{
#if MOTOR_STAGE==MOTOR_P1_3
	.PmMotorFlux = 0.0077f,
	.PmMotorLd = 0.0000193f,		//0.00002090405251
	.PmMotorLq = 0.0000281f,		//0.00003069411535
	.PmMotorRes = 0.0117f,			//0.008733
	.PmMotorPolepair = 4.0f,
	.PmMotorJ = 0.000365063f,
	.MosfetDriverDeadTime = 0.000002f,
	.MosfetDriverLowerBridgeMinTime = 0.0000016f,
	.MosfetDriverPwmPeriodCnt = (float)(INITIAL_CURRENT_LOOP_FREQ),
	.MosfetDriverShuntRisingTime = 0.0f,
	.MosfetDriverMinTimeChangeEleSpeedAbs = 1256.637f, //Mech RPM = 3000->Ele Speed = 3000/60*4*2*pi
	.PwmPeriod = 0.0001f,
	.IdHz = 100.0f,
	.IqHz = 100.0f,
	.Ilimit = 260.0f,
	.IfGain = 0.056f,
	.IfAecel = 500.0f,
	.IfDecel = 500.0f,

	.VfGain = 0.012f,
	.VfAecel = 100.0f,
	.VfDecel = 100.0f,
	.PwmPeriod = 0.0001f,
	.pMotorTableHeader = &MotorTable,
#elif MOTOR_STAGE==MOTOR_P1
	.PmMotorFlux = 0.00728f,
	.PmMotorLd = 0.0000192f,
	.PmMotorLq = 0.0000266f,
	.PmMotorRes = 0.0117f,
	.PmMotorPolepair = 4.0f,
	.PmMotorJ = 0.000365063f,
	.MosfetDriverDeadTime = 0.000002f,
	.MosfetDriverLowerBridgeMinTime = 0.0000016f,
	.MosfetDriverPwmPeriodCnt = (float)(INITIAL_CURRENT_LOOP_FREQ),
	.MosfetDriverShuntRisingTime = 0.0f,
	.MosfetDriverMinTimeChangeEleSpeedAbs = 1256.637f, //Mech RPM = 3000->Ele Speed = 3000/60*4*2*pi
	.PwmPeriod = 0.0001f,
	.IdHz = 100.0f,
	.IqHz = 100.0f,
	.Ilimit = 260.0f,
	.IfGain = 0.056f,
	.IfAecel = 500.0f,
	.IfDecel = 500.0f,

	.VfGain = 0.012f,
	.VfAecel = 100.0f,
	.VfDecel = 100.0f,
	.PwmPeriod = 0.0001f,
	.pMotorTableHeader = &MotorTable,
#endif
};

