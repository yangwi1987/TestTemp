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
#if MOTOR_STAGE==MOTOR_P0
	.PmMotorFlux = 0.0051f,     // When 120 degree C
	.PmMotorLd = 0.000096f,		// 96 uh
	.PmMotorLq = 0.000136f,    // 136 uh
	.PmMotorRes = 0.05f,			//50mohm
	.PmMotorPolepair = 5.0f,
	.PmMotorJ = 0.00697189f,
	.MosfetDriverDeadTime = 0.000002f, /*todo check with EE*/
	.MosfetDriverLowerBridgeMinTime = 0.0000025f, /*todo check with EE*/
	.MosfetDriverPwmPeriodCnt = (float)(INITIAL_CURRENT_LOOP_FREQ),
	.MosfetDriverShuntRisingTime = 0.0f,
	.MosfetDriverMinTimeChangeEleSpeedAbs = 6283.1853f, //Mech RPM = 12000->Ele Speed = 12000/60*5*2*pi
	.PwmPeriod = 0.0001f,
	.IdHz = 200.0f,
	.IqHz = 200.0f,
	.Ilimit = 531.0f,
	.IfGain = 0.056f,
	.IfAecel = 500.0f,
	.IfDecel = 500.0f,

	.VfGain = 0.012f,
	.VfAecel = 100.0f,
	.VfDecel = 100.0f,
	.pMotorTableHeader = &MotorTable,
#endif
};

