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
	.PmMotorFlux = 0.0051f,     // Wbㄝ120 degree C
	.PmMotorLd = 0.0000096f,		// 9.6 uh
	.PmMotorLq = 0.0000136f,    // 13.6 uh
	.PmMotorRes = 0.001f,			//1mohm
	.PmMotorPolepair = 5.0f,
	.PmMotorJ = 0.00697189f,
	.MosfetDriverDeadTime = 0.000002f,
	.MosfetDriverLowerBridgeMinTime = 0.0000025f,
	.MosfetDriverPwmPeriodCnt = (float)(INITIAL_CURRENT_LOOP_FREQ),
	.MosfetDriverShuntRisingTime = 0.0f,
	.MosfetDriverMinTimeChangeEleSpeedAbs = 3141.59f, //Mech RPM = 6000->Ele Speed = 6000/60*5*2*pi
	.PwmPeriod = 0.0001f,
	.IdHz = 100.0f,
	.IqHz = 100.0f,
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

