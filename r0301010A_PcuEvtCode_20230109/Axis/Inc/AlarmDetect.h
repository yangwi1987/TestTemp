/*
 * AlarmDetect.h
 *
 *  Created on: 2020年2月25日
 *      Author: MikeSFWen
 */

#ifndef INC_ALARMDETECT_H_
#define INC_ALARMDETECT_H_

#include "SystemTableLinker.h"
#include "ADC_DRI.h"
#include "PWM_TRA.h"
#include "MotorControl.h"
#include "PhaseLoss.h"
#include "SpeedInfo.h"
#include "PWM_RC.h"

enum SigState{
	SIGNAL_LOW,
	SIGNAL_HIGH
};
enum BufEnaState{
	PULL_LOW,
	PULL_HIGH
};
enum BufControlEnable{
	BUF_FLG_DISABLE,
	BUF_FLG_ENABLE
};

/* =======================================
 * ======== protect polling object =======
 * ======================================= */
typedef struct
{
	AlarmTableInfo_t AlarmInfo;
	uint16_t Counter;
} PROTECT_POLLING_TYPE;

#define PROTECT_POLLING_TYPE_DEFAULT { \
		ALARMTABLEINFO_DEFAULT, \
	0 \
	}
/* ======================================= */

/* =======================================
 * ====== Axial alarm detect object ======
 * ======================================= */
typedef void (*functypeAlarmDetect_Init)(void*, uint16_t, void*,
					void*, void*,
					void *, void *, void *);
typedef void (*functypeAlarmDetect_RegisterAxisAlarm)(void*, uint16_t, uint16_t );
typedef void (*functypeAlarmDetect_DoCurrentLoop)(void*);
typedef void (*functypeAlarmDetect_DoPLCLoop)(void*);
typedef void (*functypeAlarmDetect_Do100HzLoop)(void*);

// add SetAlarmThreshold to change threshold from external flash
typedef struct {
	int AxisID;
	AdcStation *pAdcStation;
	PwmStation *pPwmStation;
	MOTOR_CONTROL_TYPE *pMotorControl;
	SpeedInfo_t *pSpeedInfo;
	PHASE_LOSS_TYPE *pPhaseLoss;
	PWM_RC_TYPE *pPwmRcStattion;
	uint16_t BufICEnable;
	PROTECT_POLLING_TYPE UVP_Bus;
	PROTECT_POLLING_TYPE OVP_Bus;
	PROTECT_POLLING_TYPE OCP_Iu;
	PROTECT_POLLING_TYPE OCP_Iv;
	PROTECT_POLLING_TYPE OCP_Iw;
	PROTECT_POLLING_TYPE OSP;
	PROTECT_POLLING_TYPE OTP_PCU_0;
	PROTECT_POLLING_TYPE OTP_PCU_1;
	PROTECT_POLLING_TYPE OTP_PCU_2;
	PROTECT_POLLING_TYPE OTP_Motor_0;
	PROTECT_POLLING_TYPE OTP_PCU_0_WARNING;
	PROTECT_POLLING_TYPE OTP_PCU_1_WARNING;
	PROTECT_POLLING_TYPE OTP_PCU_2_WARNING;
	PROTECT_POLLING_TYPE OTP_Motor_0_WARNING;
	PROTECT_POLLING_TYPE BUF_IC_FB;
	PROTECT_POLLING_TYPE UVP_13V;
	PROTECT_POLLING_TYPE BREAK_NTC_PCU_0;
	PROTECT_POLLING_TYPE BREAK_NTC_PCU_1;
	PROTECT_POLLING_TYPE BREAK_NTC_PCU_2;
	PROTECT_POLLING_TYPE BREAK_NTC_Motor_0;
	PROTECT_POLLING_TYPE SHORT_NTC_PCU_0;
	PROTECT_POLLING_TYPE SHORT_NTC_PCU_1;
	PROTECT_POLLING_TYPE SHORT_NTC_PCU_2;
	PROTECT_POLLING_TYPE SHORT_NTC_Motor_0;
	PROTECT_POLLING_TYPE POWER_TRANSISTOR_OC;
	PROTECT_POLLING_TYPE CAN0Timeout;
	PROTECT_POLLING_TYPE CAN1Timeout;
	PROTECT_POLLING_TYPE THROT_ERROR_SHORT;
	PROTECT_POLLING_TYPE THROT_ERROR_BREAK;
	PROTECT_POLLING_TYPE RC_INVALID;
	PROTECT_POLLING_TYPE FOIL_SENSOR_BREAK;
	PROTECT_POLLING_TYPE FOIL_SENSOR_SHORT;
	functypeAlarmDetect_Init Init;
	functypeAlarmDetect_RegisterAxisAlarm RegisterAxisAlarm;
	functypeAlarmDetect_DoCurrentLoop DoCurrentLoop;
	functypeAlarmDetect_DoPLCLoop DoPLCLoop;
	functypeAlarmDetect_Do100HzLoop Do100HzLoop;
} AlarmDetect_t;

void AlarmDetect_Init( AlarmDetect_t *v, uint16_t AxisID, AdcStation *pAdcStation,
		PwmStation *pPwmStation, PHASE_LOSS_TYPE *pPhaseLoss,
		MOTOR_CONTROL_TYPE *pMotorControl, SpeedInfo_t *pSpeedInfo, PWM_RC_TYPE *pPwmRc );
void AlarmDetect_DoCurrentLoop( AlarmDetect_t *v );
void AlarmDetect_DoPLCLoop( AlarmDetect_t *v );
void AlarmDetect_Do100HzLoop( AlarmDetect_t *v );

#define ALARM_DETECT_DEFAULT { \
	0, /* AxisID */ \
	0, /* pAdcStation */\
	0, /* pPwmStation */ \
	0, /* pMotorControl */\
	0, /* pEncoderModule */\
	0, /* pPhaseLoss */ \
	0, /* pPwmRcStattion */ \
	0, /* BufICEnable */\
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	PROTECT_POLLING_TYPE_DEFAULT, \
	(functypeAlarmDetect_Init)AlarmDetect_Init, \
	0, /* RegisterAxisAlarm */ \
	(functypeAlarmDetect_DoCurrentLoop)AlarmDetect_DoCurrentLoop, \
	(functypeAlarmDetect_DoPLCLoop)AlarmDetect_DoPLCLoop, \
	(functypeAlarmDetect_Do100HzLoop)AlarmDetect_Do100HzLoop}

#endif /* INC_ALARMDETECT_H_ */
