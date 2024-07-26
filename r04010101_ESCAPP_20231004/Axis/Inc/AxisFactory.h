/*
 * AxisFactory.h
 *
 *  Created on: 2019年12月11日
 *      Author: MikeSFWen
 */

#ifndef INC_AXISFACTORY_H_
#define INC_AXISFACTORY_H_

#define AXIS_INDEX_TO_AXIS_ID( x ) ( (x) + 1 ) // Index means index of axis array: 0, 1, ...
#define AXIS_ID_TO_AXIS_INDEX( x ) ( (x) - 1 ) // ID means order of axis: 1, 2, ...

#include "AlarmMgr.h"
#include "TorqCommandGenerator.h"
#include "MotorControl.h"
#include "Drive.h"
#include "PWM_TRA.h"
#include "AlarmDetect.h"
#include "FourQuadControl.h"
#include "ExtranetCANStation.h"
#include "MotorStall.h"
#include "ThermoStrategy.h"
#include "PhaseLoss.h"
#include "PWM_RC.h"
#include "SpeedInfo.h"
#include "ConstantParamAndUseFunction.h"
#include "RcUartComm.h"
#if BME
#include "BMEApp.h"
#endif
#if E10
#include "E10App.h"
#include "HiResoMotorTable.h"
#include "GearMode.h"
#endif

typedef void (*functypeAxis_Init)(void*,uint16_t);
typedef void (*functypeAxis_DoCurrentLoop)(void*);
typedef void (*functypeAxis_DoPLCLoop)(void*);
typedef void (*functypeAxis_Do100HzLoop)(void*);
typedef void (*functypeAxis_Do10HzLoop)(void*);
typedef void (*functypeAxis_OnParamValueChanged)(void*, uint16_t no);

#define BRAKE_ON GPIO_PIN_RESET

typedef enum
{
	MOTOR_STATE_OFF,
	MOTOR_STATE_ON,
	MOTOR_STATE_WAIT_BOOT,
	MOTOR_STATE_SHUTDOWN_START
}MotorStateMachine_e;

typedef enum
{
	Drive_Stop_Flag,
	Drive_Start_Flag
}DriveStateFlag_e;

enum UI_FUNCTION_MODE {
	UI_FUNCTION_DISABLE = 0,
	UI_FUNCTION_RD_ENABLE,
	UI_FUNCTION_MF_ENABLE
};

enum RD_FUNCTION_MODE {
	RD_FUNCTION_DISABLE = 0,
	RD_FUNCTION_FOC,
	RD_FUNCTION_VF,
	RD_FUNCTION_IF,
};

enum PID_CHECK_STATE {
	PID_CHECK_PASS = 0,
	PID_CHECK_PASS_BUT_FULL,
	PID_CHECK_FAIL_EMPTY,
	PID_CHECK_FAIL_ABNORMAL,
	PID_CHECK_FAIL_AND_FULL
};

enum GAIN_STATE{
	GAIN_STATE_EMPTY = 0,
	GAIN_STATE_ABNORMAL,
	GAIN_STATE_NORMAL,
};

enum DRIVE_MODE{
	DRIVE_NONE,
	DRIVE_PADDLE,
	DRIVE_SURF,
	DRIVE_FOIL
};

enum MAST_MODE{
	MAST_PADDLE,
	MAST_SURF,
	MAST_FOIL
};

typedef struct
{
	uint16_t FOIL_DI2:1;
	uint16_t FOIL_DI3:1;
	uint16_t Reserved:14;
}FoilInfor_t;



typedef struct
{
	float MaxSurf;
	float MinSurf;
	float MaxFoil;
	float MinFoil;
}AnalogFoilInfo_t;

typedef struct
{
	uint8_t IsUseDriveLockFn;
	DriveStateFlag_e DriveStateFlag;
	uint32_t TimeToStopDriving_InPLCLoop;
	uint32_t TimeToStopDriving_cnt;
	uint16_t RpmToStartCntDriveLock;
}DriveLockInfo_t;

typedef union
{
	uint16_t All;
	FoilInfor_t Bit;
}FoilInfor_u;

#if BME
typedef struct {
	int16_t AxisID;
	uint16_t PcuID;
	uint16_t PIDCheckResult;
	uint16_t ESCOperationState;
	int16_t ServoOn;
	int16_t ServoOnOffState;
	uint16_t HasWarning; // 0: no warning, 1: warning exist
	uint16_t HasNonCriAlarm; // 0: no non-critical alarm, 1: non-critical alarm exist
	uint16_t HasCriAlarm; // 0: no critical alarm, 1: critical alarm exist
	int16_t BootstrapCounter;
	int16_t BootstrapMaxCounter;
	int16_t BoostrapTimeMs;
	int16_t MotorCtrlMode;
	int16_t	CtrlUiEnable;
	int16_t VCUServoOnCommand;
	float ThrottleGain;
	uint16_t ThrottleGainState;
	uint16_t DcBusGainState;
	int16_t PcuPowerState;
	FoilInfor_u FoilState;
	uint16_t MfOrRDFunctionDisable;
	uint16_t TriggerLimpHome;
	MOTOR_CONTROL_TYPE MotorControl;
	PWM_RC_TYPE	PwmRc;
	TorqCommandGenerator_t TorqCommandGenerator;
	AlarmStack_t* pAlarmStack;
	DriveParams_t *pDriveParams;
	AdcStation  *pAdcStation;
	PwmStation	*pPwmStation;
	ExtranetCANStation_t *pCANStaion;
	ParamMgr_t *pParamMgr;
	AlarmDetect_t AlarmDetect;
	ThrottleMapping_t ThrotMapping;
	FourQuadControl FourQuadCtrl;
	MotorStall_t MotorStall;
	ThermoStrategy_t ThermoStrategy;
	STRUCT_CANTxInterface	*pCANTxInterface;
	STRUCT_CANRxInterface	*pCANRxInterface;
	PHASE_LOSS_TYPE	PhaseLoss;
	SpeedInfo_t SpeedInfo;
	AnalogFoilInfo_t AnalogFoilInfo;
	DriveLockInfo_t DriveLockInfo;
	functypeAxis_Init Init;
	functypeAxis_DoCurrentLoop DoCurrentLoop;
	functypeAxis_DoPLCLoop DoPLCLoop;
	functypeAxis_Do100HzLoop Do100HzLoop;
	functypeAxis_Do10HzLoop Do10HzLoop;
	functypeAxis_OnParamValueChanged OnParamValueChanged;
} Axis_t;

#define AXIS_DEFAULT { \
	0,      /*AxisID;             */ \
	0,      /* PcuID;             */ \
	0,      /* PIDCheckResult;    */ \
	0,      /*ESCOperationState   */ \
	0,      /*ServoOn;            */ \
	0,      /*ServoOnOffState;    */ \
	0,      /* HasWarning;        */ \
	0,      /* HasNonCriAlarm;          */ \
	0,      /* HasCriAlarm;          */ \
	0,      /*BootstrapCounter;   */ \
	100,	/*BootstrapMaxCounter;*/ \
	10,     /*BoostrapTimeMs;     */ \
	1,      /*MotorCtrlMode;      */ \
	0,      /*CtrlUiEnable;       */ \
	0,      /*VCUServoOnCommand;  */ \
	0.0f,   /*ThrottleGain*/ \
	0,      /*ThrottleGainState;*/ \
	0,      /*DcBusGainState;   */ \
	PWR_SM_INITIAL,    /*PcuPowerState;       */\
	{0},                   /*    FoilState;       */\
	1,	                   /* MfOrRDFunctionDisable;    */\
	0,	                   /* TriggerLimpHome;    */\
	MOTOR_CONTROL_DEFAULT, /*MotorControl         */\
	PWM_RC_TYPE_DEFAULT,   /*   	PwmRc;        */\
	TORQ_COMMAND_GENERTATOR_DEFAULT, /*TorqCommandGenerator*/\
	0, /* pAlarmStack; */\
	0, /* *pDriveParams*/\
	0, /*pAdcStation;  */\
	0, /*pPwmStation;  */\
	0, /*pCANStaion*/\
	0, /*pParamMgr*/\
	ALARM_DETECT_DEFAULT, \
	THROTTLE_CALIB_DEFAULT, \
	FOURQUADCONTROL_DEFAULT, \
	MOTORSTALL_DEFAULT, \
	THERMOSTRATEGY_DEFAULT, \
	0,                             /*pCANTxInterface*/\
	0,                             /*pCANRxInterface*/\
	PHASE_LOSS_DEFAULT,	\
	SPEED_INFO_DEFAULT,	\
	ANALOG_FOIL_INFO_DEFAULT, \
	DRIVE_LOCK_INFO_DERAULT, \
	(functypeAxis_Init)AxisFactory_Init, \
	(functypeAxis_DoCurrentLoop)AxisFactory_DoCurrentLoop, \
	(functypeAxis_DoPLCLoop)AxisFactory_DoPLCLoop, \
	(functypeAxis_Do100HzLoop)AxisFactory_Do100HzLoop, \
	(functypeAxis_Do10HzLoop)AxisFactory_Do10HzLoop, \
	(functypeAxis_OnParamValueChanged)AxisFactory_OnParamValueChanged \
}
#endif /* BME */

#if E10
typedef struct {
	int16_t AxisID;
	uint16_t PcuID;
	uint16_t PIDCheckResult;
	int16_t ServoOn;
	int16_t ServoOnOffState;
	uint16_t HasWarning; // 0: no warning, 1: warning exist
	uint16_t HasNonCriAlarm; // 0: no non-critical alarm, 1: non-critical alarm exist
	uint16_t HasCriAlarm; // 0: no critical alarm, 1: critical alarm exist
	int16_t BootstrapCounter;
	int16_t BootstrapMaxCounter;
	int16_t BoostrapTimeMs;
	int16_t MotorCtrlMode;
	int16_t	CtrlUiEnable;
	uint8_t ServoOnCommand;
	float ThrottleGain;
	uint16_t ThrottleGainState;
	uint16_t DcBusGainState;
	int16_t PcuPowerState;
	uint16_t MfOrRDFunctionDisable;
	uint16_t TriggerLimpHome;
	MOTOR_CONTROL_TYPE MotorControl;
	PWM_RC_TYPE	PwmRc;
	TorqCommandGenerator_t TorqCommandGenerator;
	AlarmStack_t* pAlarmStack;
	DriveParams_t *pDriveParams;
	AdcStation  *pAdcStation;
	PwmStation	*pPwmStation;
	ExtranetCANStation_t *pCANStaion;
	ParamMgr_t *pParamMgr;
	AlarmDetect_t AlarmDetect;
	ThrottleMapping_t ThrotMapping;
	FourQuadControl FourQuadCtrl;
	MotorStall_t MotorStall;
	ThermoStrategy_t ThermoStrategy;
	STRUCT_CANTxInterface	*pCANTxInterface;
	STRUCT_CANRxInterface	*pCANRxInterface;
	PHASE_LOSS_TYPE	PhaseLoss;
	SpeedInfo_t SpeedInfo;
	GearMode_Var_t GearModeVar;
	functypeAxis_Init Init;
	functypeAxis_DoCurrentLoop DoCurrentLoop;
	functypeAxis_DoPLCLoop DoPLCLoop;
	functypeAxis_Do100HzLoop Do100HzLoop;
	functypeAxis_Do10HzLoop Do10HzLoop;
	functypeAxis_OnParamValueChanged OnParamValueChanged;
} Axis_t;

#define AXIS_DEFAULT { \
	0,      /*AxisID;             */ \
	0,      /* PcuID;             */ \
	0,      /* PIDCheckResult;    */ \
	0,      /*ServoOn;            */ \
	0,      /*ServoOnOffState;    */ \
	0,      /* HasWarning;        */ \
	0,      /* HasNonCriAlarm;          */ \
	0,      /* HasCriAlarm;          */ \
	0,      /*BootstrapCounter;   */ \
	100,	/*BootstrapMaxCounter;*/ \
	10,     /*BoostrapTimeMs;     */ \
	1,      /*MotorCtrlMode;      */ \
	0,      /*CtrlUiEnable;       */ \
	0,      /*ServoOnCommand;  */ \
	0.0f,   /*ThrottleGain*/ \
	0,      /*ThrottleGainState;*/ \
	0,      /*DcBusGainState;   */ \
	PWR_SM_INITIAL,    /*PcuPowerState;       */\
	1,	                   /* MfOrRDFunctionDisable;    */\
	0,	                   /* TriggerLimpHome;    */\
	MOTOR_CONTROL_DEFAULT, /*MotorControl         */\
	PWM_RC_TYPE_DEFAULT,   /*   	PwmRc;        */\
	TORQ_COMMAND_GENERTATOR_DEFAULT, /*TorqCommandGenerator*/\
	0, /* pAlarmStack; */\
	0, /* *pDriveParams*/\
	0, /*pAdcStation;  */\
	0, /*pPwmStation;  */\
	0, /*pCANStaion*/\
	0, /*pParamMgr*/\
	ALARM_DETECT_DEFAULT, \
	THROTTLE_CALIB_DEFAULT, \
	FOURQUADCONTROL_DEFAULT, \
	MOTORSTALL_DEFAULT, \
	THERMOSTRATEGY_DEFAULT, \
	0,                             /*pCANTxInterface*/\
	0,                             /*pCANRxInterface*/\
	PHASE_LOSS_DEFAULT,	\
	SPEED_INFO_DEFAULT,	\
	GEAR_MODE_VAR_DEFALUT, \
	(functypeAxis_Init)AxisFactory_Init, \
	(functypeAxis_DoCurrentLoop)AxisFactory_DoCurrentLoop, \
	(functypeAxis_DoPLCLoop)AxisFactory_DoPLCLoop, \
	(functypeAxis_Do100HzLoop)AxisFactory_Do100HzLoop, \
	(functypeAxis_Do10HzLoop)AxisFactory_Do10HzLoop, \
	(functypeAxis_OnParamValueChanged)AxisFactory_OnParamValueChanged \
}
#endif /* E10 */

void AxisFactory_Init( Axis_t *v, uint16_t AxisIndex );
void AxisFactory_DoCurrentLoop( Axis_t *v );
void AxisFactory_DoPLCLoop( Axis_t *v );
void AxisFactory_Do100HzLoop( Axis_t *v );
void AxisFactory_Do10HzLoop( Axis_t *v );
void AxisFactory_OnParamValueChanged( Axis_t *v, uint16_t ParamNumber );

extern MOTOR_CONTROL_PARAMETER_DEFAULT_TYPE MotorDefault;

#endif /* INC_AXISFACTORY_H_ */
