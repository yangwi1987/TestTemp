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
	.PmMotorFlux = 0.00614f,     // When 120 degree C
	.PmMotorLd = 0.000009712f,		// 9.6 uh
	.PmMotorLq = 0.00001473f,    // 13.6 uh
	.PmMotorRes = 0.000913f,			//1mohm
	.PmMotorPolepair = 5.0f,
	.PmMotorJ = 0.00697189f,
	.MosfetDriverDeadTime = 0.000002f, /*todo check with EE*/
	.MosfetDriverLowerBridgeMinTime = 0.0000025f, /*todo check with EE*/
	.MosfetDriverPwmPeriodCnt = (float)(INITIAL_CURRENT_LOOP_FREQ),
	.MosfetDriverShuntRisingTime = 0.0f,
	.MosfetDriverMinTimeChangeEleSpeedAbs = 523.5987756f, //Mech RPM = 1000->Ele Speed = 1000/60*5*2*pi
	.PwmPeriod = 1.0f / (float)(INITIAL_CURRENT_LOOP_FREQ),
	.IdHz = 500.0f,
	.IqHz = 500.0f,
	.IdDCPHz = 100.0f,
	.IqDCPHz = 100.0f,
	.VbusHz = 60000.0f,
	.FWHz = 20.0f,
	.Ilimit = 531.0f,

	.DeadtimeDutyP = 0.0125f,
	.DeadtimeSlopeP = 5.0f,
	.DeadtimeDutyN = 0.0125f,
	.DeadtimeSlopeN = 5.0f,

	.IfGain = 0.056f,
	.IfAecel = 500.0f,
	.IfDecel = 500.0f,

	.VfGain = 0.012f,
	.VfAecel = 100.0f,
	.VfDecel = 100.0f,
	.pMotorTableHeader = &MotorTable,
#endif
};

