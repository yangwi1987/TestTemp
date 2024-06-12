/*
 * MotorControl.h
 *
 *  Created on: 2019年12月3日
 *      Author: Fernando
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#define PHASE_U	0
#define PHASE_V	1
#define PHASE_W	2

#define MOTOR_TYPE_IS_PM		0

#define DRIVER_TYPE_IS_MOSFET	0


#include "stdint.h"
#include "Constant.h"
#include "ConstantParamAndUseFunction.h"

#include "Math.h"
#include "MathFunctionInclude.h"
#include "Motor_Init_Table.h"

#include "PWM_TRA.h"

#include "CoordinateTransfer.h"
#include "GeneratePwmDuty.h"
#include "FluxWeakening.h"
#include "OpenLoopControl.h"
#include "TorqueToIdq.h"
#include "SixWave.h"
#include "AngleObserver.h"
#include "Sensorless.h"

#define MOTOR_PHASE 3


enum CONTROL_INIT_STATUS_ENUM
{
	CONTROL_INIT_OK = 0,
	CONTROL_INIT_STATUS_NUMBER_PWM_DUTY	= 0x8001,
	CONTROL_INIT_STATUS_NUMBER_ID_REGULATOR,
	CONTROL_INIT_STATUS_NUMBER_IQ_REGULATOR,
	CONTROL_INIT_STATUS_NUMBER_FLUX_WEAKENING_REGULATOR,
	CONTROL_INIT_STATUS_NUMBER_IF_CONTROL,
	CONTROL_INIT_STATUS_NUMBER_VF_CONTROL,
	CONTROL_INIT_STATUS_NUMBER_ID_CMD_TABLE,
	CONTROL_INIT_STATUS_NUMBER_IQ_CMD_TABLE,
	CONTROL_INIT_STATUS_NUMBER_AC_lIMIT_TABLE,
	CONTROL_INIT_STATUS_NUMBER_DC_lIMIT_TABLE,
	CONTROL_INIT_STATUS_NUMBER_MAX_TORQUE_TABLE,
	CONTROL_INIT_STATUS_NUMBER_SIX_WAVE,
};

enum FUNCTION_MODE_ENUM
{
	FUNCTION_MODE_NORMAL_CURRENT_CONTROL = 0x0001,
	FUNCTION_MODE_VF_CONTROL,
	FUNCTION_MODE_IF_CONTROL,
	FUNCTION_MODE_TORQUE_CONTROL,
	FUNCTION_MODE_BOOTSTRAP,
	FUNCTION_MODE_SIX_WAVE_120_OPEN_LOOP,
	FUNCTION_MODE_SIX_WAVE_120_CLOSE_LOOP,
	FUNCTION_MODE_EEMF,
	FUNCTION_MODE_HFI_SIN,
	FUNCTION_MODE_PWM,
};

enum POS_FB_ENUM
{
	POS_FB_NONE = 0xff,
	POS_FB_ENCODER_SENSOR = 0x0001,
	POS_FB_IF_VIRTUAL,
	POS_FB_VF_VIRTUAL,
	POS_FB_HALL,
	POS_FB_EEMF,
	POS_FB_HFI_SIN,
};

enum CMD_FROM_ENUM
{
	CMD_FROM_NONE = 0xff,
	CMD_FROM_FOC_CTRL = 0x0001,
	CMD_FROM_IF_CTRL,
	CMD_FROM_VF_CTRL,
	CMD_FROM_SIX_WAVE_120_CLOSE_LOOP,
};

enum CTRL_MODE_ENUM
{
	CTRL_MODE_NONE = 0xff,
	CTRL_MODE_CURRENT_CONTROL = 0x0001,
	CTRL_MODE_VF_CONTROL,
	CTRL_MODE_SIX_WAVE_120_OPEN_LOOP,
	CTRL_MODE_SIX_WAVE_120_CLOSE_LOOP,
};

enum
{
	PWM_MODE_SVPWM = 0x0001,
	PWM_MODE_BOOTSTRAP,
	PWM_MODE_SIX_WAVE_120
};

typedef void (*pfunMotorControl_Algorithm) ( void*, uint16_t);
typedef void (*pfunMotorControl_CleanParameter) ( void* );
typedef uint16_t (*pfunMotorControl_InitParameter)( void*, void*);


typedef struct
{
/******************************Motor*************************************/
	float PmMotorFlux;
	float PmMotorLd;
	float PmMotorLq;
	float PmMotorRes;
	float PmMotorPolepair;
	float PmMotorJ;
	const MOTOR_TABLE_TYPE *pMotorTableHeader;
/******************************Driver*************************************/
	float MosfetDriverDeadTime;
	float MosfetDriverLowerBridgeMinTime;
	float MosfetDriverPwmPeriodCnt;
	float MosfetDriverShuntRisingTime;
	float MosfetDriverMinTimeChangeEleSpeedAbs;
/******************************Control*************************************/
	float PwmPeriod;
	float IdHz;
	float IqHz;
	float IdDCPHz;
	float IqDCPHz;
	float Ilimit;
	float IfGain;
	float IfAecel;
	float IfDecel;
	float VfGain;
	float VfAecel;
	float VfDecel;
} MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE;

typedef struct
{
//	PM motor electrical characteristic
	float Ld;
	float Lq;
	float Res;
	float Flux;
	float Polepair;
	float J;
} PM_MOTOR_PARAMETER_TYPE;

typedef struct
{
	uint16_t Type;	// PM / Induction / ...
	PM_MOTOR_PARAMETER_TYPE PM;
} MOTOR_PARAMETER_TYPE;

typedef struct
{
	float DeadTime;
	float LowerBridgeMinTime;
	float MinTimeEleSpeedAbs;
} MOSFET_DRIVER_PARAMETER_TYPE;

typedef struct
{
	unsigned int Type;
	MOSFET_DRIVER_PARAMETER_TYPE Mosfet;
} DRIVER_PARAMETER_TYPE;

typedef struct
{
	float Iu;
	float Iv;
	float Iw;
	float Vbus;
	float EleAngle;
	float EleSpeed;
	float AllowFluxRec;
	uint16_t HallSignal;
	uint16_t TestHallSignal;
} SENSOR_FEEDBACK_TYPE;

typedef struct
{
	float IdCmd;
	float IqCmd;
	float IfRpmTarget;
	float VfRpmTarget;
	float BoostrapDuty;
	float TorqueCmd;
	float SixWaveVoltCmd;
	float SixWaveCurrCmd;
} COMMAND_TYPE;

typedef struct
{
	float VdCmd;
	float VqCmd;
	float VcmdAmp;
	float EleCompAngle;
	ROTOR_TO_STATOR_TYPE StatorVoltCmd;
} VOLT_COMMAND_TYPE;

typedef struct
{
	PI_TYPE PIDWayId;
	PI_TYPE PIDWayIq;
} FOC_DECOUPLING_TYPE;

typedef struct
{
	uint16_t EnableDirectIdqCmd:1;
	uint16_t EnableFunctionReserved2:1;
	uint16_t EnableFunctionReserved3:1;
	uint16_t EnableFunctionReserved4:1;
	uint16_t EnableFunctionReserved5:1;
	uint16_t EnableFunctionReserved6:1;
	uint16_t EnableFunctionReserved7:1;
	uint16_t EnableFunctionReserved8:1;
	uint16_t EnableFunctionReserved9:1;
	uint16_t EnableFunctionReserved10:1;
	uint16_t EnableFunctionReserved11:1;
	uint16_t EnableFunctionReserved12:1;
	uint16_t EnableFunctionReserved13:1;
	uint16_t EnableFunctionReserved14:1;
	uint16_t EnableFunctionReserved15:1;
	uint16_t EnableFunctionReserved16:1;
	float IdCmd;
	float IqCmd;
	float EleAngle;
	float EleSpeed;
	float PwmPeriod;
	float PwmHz;
	PHASE_TO_STATOR_TYPE StatorCurrFb;
	STATOR_TO_ROTOR_TYPE RotorCurrFb;
	PI_TYPE	IdRegulator;
	PI_TYPE	IqRegulator;
	FOC_DECOUPLING_TYPE Decoupling;
	CURRENT_CONTROL_FLUX_WEAKENING_TYPE FluxWeakening;
} CURRENT_CONTROL_TYPE;

typedef struct
{
	uint16_t ControlInitStatus;
	uint16_t HallSelect : 1;
	uint16_t Reserved2 : 1;
	uint16_t Reserved3 : 1;
	uint16_t Reserved4 : 1;
	uint16_t Reserved5 : 1;
	uint16_t Reserved6 : 1;
	uint16_t Reserved7 : 1;
	uint16_t Reserved8 : 1;
	uint16_t Reserved9 : 1;
	uint16_t Reserved10 : 1;
	uint16_t Reserved11 : 1;
	uint16_t Reserved12 : 1;
	uint16_t Reserved13 : 1;
	uint16_t Reserved14 : 1;
	uint16_t Reserved15 : 1;
	uint16_t Reserved16 : 1;
	uint16_t StartUpWay;
	//parameter
	MOTOR_PARAMETER_TYPE MotorPara;
	DRIVER_PARAMETER_TYPE DriverPara;
	//Sensor feedback and command
	SENSOR_FEEDBACK_TYPE SensorFb;
	COMMAND_TYPE Cmd;
	TORQUE_TO_IDQ_TYPE TorqueToIdq;
	SIX_WAVE_HALL_TYPE TestHall;
	Sensorless_t Sensorless;

	//control algorithm
	IF_CONTROL_TYPE IfControl;
	VF_CONTROL_TYPE	VfControl;
	CURRENT_CONTROL_TYPE CurrentControl;
	SIX_WAVE_120_CURRENT_CONTROL_TYPE SixWave120CurrentControl;

	//Generate PWM
	VOLT_COMMAND_TYPE VoltCmd;
	SVPWM_TYPE Svpwm;
	SIX_WAVE_120_DUTY_TYPE	SixWave120Duty;
	DUTY_COMMAND_TYPE PwmDutyCmd;
	COMPENSATION_DEADTIME_TYPE CompDuty;
	pfunMotorControl_Algorithm Process;
	pfunMotorControl_CleanParameter Clean;
	pfunMotorControl_InitParameter Init;
} MOTOR_CONTROL_TYPE;


void MotorControl_Algorithm( MOTOR_CONTROL_TYPE *p, uint16_t FunctionMode);
void MotorControl_CleanParameter( MOTOR_CONTROL_TYPE *p );
uint16_t MotorControl_InitParameter( MOTOR_CONTROL_TYPE *p, MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE *pSetting);


#define MOTOR_CONTROL_PARAMETER_DEFAULT_DEFAULT		\
{													\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	2.0f,											\
	0.0f,											\
	0,												\
	0.000001f,										\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0001f,										\
	10.0f,											\
	10.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f,											\
	0.0f											\
}

#define PM_MOTOR_PARAMETER_DEAFULAT \
{									\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	0.0f,							\
	2.0f,							\
	0.0f,							\
}

#define MOTOR_PARAMETER_DEFAULT		\
{									\
	MOTOR_TYPE_IS_PM,				\
	PM_MOTOR_PARAMETER_DEAFULAT 	\
}

#define MOSFET_DRIVER_PARAMETER_DEFAULT \
{										\
    0.000001f,							\
	0.0f,								\
	10000.0f,							\
}

#define DRIVER_PARAMETER_DEFAULT		\
{										\
	DRIVER_TYPE_IS_MOSFET,				\
	MOSFET_DRIVER_PARAMETER_DEFAULT		\
}

#define SENSOR_FEEDBACK_DEFAULT			\
{										\
	0.0f,								\
	0.0f,								\
	0.0f,								\
	1.0f,								\
	0.0f,								\
	0.0f,								\
	0.0f,								\
	0,									\
	0,									\
}

#define COMMAND_DEFAULT	\
{						\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
}

#define VOLT_COMMAND_DEFAULT	\
{								\
	0.0f,						\
	0.0f,						\
	0.0f,						\
	0.0f,						\
	ROTOR_TO_STATOR_DEFAULT, 	\
}

#define FOC_DECOUPLING_DEFAULT	\
{				\
	PI_DEFAULT,	\
	PI_DEFAULT,	\
}

#define CURRENT_CONTROL_DEFAULT				\
{											\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	FUNCTION_DISABLE, 						\
	0.0f,									\
	0.0f,									\
	0.0f,									\
	0.0f,									\
	0.0001f,								\
	10000.0f,								\
	PHASE_TO_STATOR_DEFAULT,				\
	STATOR_TO_ROTOR_DEFAULT,				\
	PI_DEFAULT,								\
	PI_DEFAULT,								\
	FOC_DECOUPLING_DEFAULT,					\
	CURRENT_CONTROL_FLUX_WEAKENING_DEFAULT	\
}


#define MOTOR_CONTROL_DEFAULT			\
{										\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	0,									\
	MOTOR_PARAMETER_DEFAULT,			\
	DRIVER_PARAMETER_DEFAULT,			\
	SENSOR_FEEDBACK_DEFAULT,			\
	COMMAND_DEFAULT,					\
	TORQUE_TO_IDQ_DEFAULT,				\
	SIX_WAVE_HALL_DEFAULT,				\
	SENSORLESS_DEFAULT,					\
	IF_CONTROL_DEFAULT,					\
	VF_CONTROL_DEFAULT,					\
	CURRENT_CONTROL_DEFAULT,			\
	SIX_WAVE_CURRENT_CONTROL_DEFAULT,	\
	VOLT_COMMAND_DEFAULT,				\
	SVPWM_DEFAULT,						\
	SIX_WAVE_120_DUTY_DEFAULT,			\
	DUTY_COMMAND_DEFAULT,				\
	COMPENSATION_DEADTIME_DEFAULT,		\
	(pfunMotorControl_Algorithm)MotorControl_Algorithm,					\
	(pfunMotorControl_CleanParameter)MotorControl_CleanParameter,		\
	(pfunMotorControl_InitParameter)MotorControl_InitParameter,			\
}

#endif /* INC_MOTORCONTROL_H_ */
