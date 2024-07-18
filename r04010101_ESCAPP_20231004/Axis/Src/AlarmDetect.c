/*
 * AlarmDetect.c
 *
 *  Created on: 2020年2月25日
 *      Author: MikeSFWen
 */

#include "AlarmDetect.h"
#include "RcUartComm.h"
extern uint16_t IsUseDigitalFoilSensor;

static uint16_t AlarmDetect_Accumulation( AlarmDetect_t *v, PROTECT_POLLING_TYPE *p, int Signal )
{
	uint16_t Trigger = 0;
	uint16_t Abnormal = ALARM_TYPE_NONE;

	if( p->AlarmInfo.AlarmEnable == ALARM_ENABLE )
	{
		if( p->AlarmInfo.AlarmMode == TRIG_MODE_LOW )
		{
			Trigger = Signal < p->AlarmInfo.AlarmThreshold;
		}
		else
		{
			Trigger = Signal > p->AlarmInfo.AlarmThreshold;
		}

		if( Trigger )
		{
			if( p->Counter < ALARM_COUNTER_MAX )
			{
				p->Counter++;
			}
			// critical debounce
			if( p->Counter > p->AlarmInfo.CriAlarmCounter )
			{
				v->RegisterAxisAlarm( v, p->AlarmInfo.AlarmID, ALARM_TYPE_CRITICAL );
				Abnormal = ALARM_TYPE_CRITICAL;
			}

			// non critical debounce
			else if( p->Counter > p->AlarmInfo.NonCriAlarmCounter )
			{
				v->RegisterAxisAlarm( v, p->AlarmInfo.AlarmID, ALARM_TYPE_NONCRITICAL );
				Abnormal = ALARM_TYPE_NONCRITICAL;
			}
		}
		else if( p->Counter > 0 )
		{
			p->Counter--;
		}
	}
	return Abnormal;
}

static void AlarmDetect_BufferIcFb( AlarmDetect_t *v, PROTECT_POLLING_TYPE *p, uint16_t BufFB, uint16_t HwocpState, int TargetID )
{
	/* Function : Buffer IC control (detect the HWOCP and Buffer Feedback signal
	 * Author : Hank Chen
	 * Data source : HW - Uzi Wang
	 * Turth table
	 * |Buffer		|HW			|Buffer		|Alarm		|Mcu		|Gate
	 * |Feedback	|OCP		|Enable		|			|Pwm		|Driver
	 * --------------------------------------------------------------------
	 * |L			|L			|H			|Alarm		|Off		|Off
	 * --------------------------------------------------------------------
	 * |L			|H			|H			|No			|Off		|Off
	 * --------------------------------------------------------------------
	 * |H			|L			|L			|No			|On			|On
	 * --------------------------------------------------------------------
	 * |H			|H			|H			|Alarm		|Off		|Off
	 * --------------------------------------------------------------------
	 * H = 1 ( High )
	 * L = 0 ( Low )
	 * X = Don't care -> Classifies it to H
	 */
	if( BufFB == SIGNAL_HIGH && HwocpState == SIGNAL_LOW)
	{
		//Buffer IC Normal
		v->BufICEnable = PULL_HIGH;		//Buffer IC Disable, But its HWOCP triggered State
	}
	else if( BufFB == SIGNAL_LOW && HwocpState == SIGNAL_HIGH )
	{
		//Buffer IC Normal
		v->BufICEnable = PULL_LOW;		//Buffer IC Enable
	}
	else
	{
		v->BufICEnable = PULL_HIGH;		//Buffer IC Disable
		//Buffer IC Abnormal
		v->RegisterAxisAlarm( v, p->AlarmInfo.AlarmID, p->AlarmInfo.AlarmType );
	}
}

static void SetAlarmThreshold(PROTECT_POLLING_TYPE *v, uint16_t index)
{
	// set threshold from external flash (P3-00~P3-xx)
	// This function sholud be executed after ParamMgr1.Init
	v->AlarmInfo.AlarmThreshold = *(&DriveParams.PCUParams.Reserved300 + index);
}

void AlarmDetect_Init( AlarmDetect_t *v, uint16_t AxisID, AdcStation *pAdcStation,
		PwmStation *pPwmStation,
		PHASE_LOSS_TYPE *pPhaseLoss, MOTOR_CONTROL_TYPE *pMotorControl, SpeedInfo_t *pSpeedInfo, PWM_RC_TYPE *pPwmRc  )
{
	// init pointers
	v->AxisID = AxisID;
	v->pAdcStation = pAdcStation;
	v->pPwmStation = pPwmStation;
	v->pMotorControl = pMotorControl;
	v->pSpeedInfo = pSpeedInfo;
	v->pPhaseLoss = pPhaseLoss;
	v->pPwmRcStattion = pPwmRc;
	// Init Alarm Infomation from table
	v->UVP_Bus.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_UNDER_VOLTAGE_BUS];
	v->UVP_Bus.Counter = 0;
	v->OVP_Bus.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OVER_VOLTAGE_BUS];
	v->OVP_Bus.Counter = 0;
	v->OCP_Iu.AlarmInfo  = SystemTable.AlarmTableInfo[ALARMID_IU_OCP];
	v->OCP_Iu.Counter = 0;
	v->OCP_Iv.AlarmInfo  = SystemTable.AlarmTableInfo[ALARMID_IV_OCP];
	v->OCP_Iv.Counter = 0;
	v->OCP_Iw.AlarmInfo  = SystemTable.AlarmTableInfo[ALARMID_IW_OCP];
	v->OCP_Iw.Counter = 0;
	v->OSP.AlarmInfo 	 = SystemTable.AlarmTableInfo[ALARMID_MOTOR_OVER_SPEED];
	v->OSP.Counter = 0;
	v->OTP_PCU_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_0];
	v->OTP_PCU_0.Counter = 0;
	v->OTP_PCU_1.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_1];
	v->OTP_PCU_1.Counter = 0;
	v->OTP_PCU_2.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_2];
	v->OTP_PCU_2.Counter = 0;
	v->OTP_Motor_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_MOTOR_0];
	v->OTP_Motor_0.Counter = 0;
	v->OTP_PCU_0_WARNING.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_0_WARNING];
	v->OTP_PCU_0_WARNING.Counter = 0;
	v->OTP_PCU_1_WARNING.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_1_WARNING];
	v->OTP_PCU_1_WARNING.Counter = 0;
	v->OTP_PCU_2_WARNING.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_PCU_2_WARNING];
	v->OTP_PCU_2_WARNING.Counter = 0;
	v->OTP_Motor_0_WARNING.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_OT_MOTOR_0_WARNING];
	v->OTP_Motor_0_WARNING.Counter = 0;
	v->BREAK_NTC_PCU_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_BREAK_NTC_PCU_0];
	v->BREAK_NTC_PCU_0.Counter = 0;
	v->BREAK_NTC_PCU_1.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_BREAK_NTC_PCU_1];
	v->BREAK_NTC_PCU_1.Counter = 0;
	v->BREAK_NTC_PCU_2.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_BREAK_NTC_PCU_2];
	v->BREAK_NTC_PCU_2.Counter = 0;
	v->BREAK_NTC_Motor_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_BREAK_NTC_MOTOR_0];
	v->BREAK_NTC_Motor_0.Counter = 0;
	v->SHORT_NTC_PCU_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_SHORT_NTC_PCU_0];
	v->SHORT_NTC_PCU_0.Counter = 0;
	v->SHORT_NTC_PCU_1.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_SHORT_NTC_PCU_1];
	v->SHORT_NTC_PCU_1.Counter = 0;
	v->SHORT_NTC_PCU_2.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_SHORT_NTC_PCU_2];
	v->SHORT_NTC_PCU_2.Counter = 0;
	v->SHORT_NTC_Motor_0.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_SHORT_NTC_MOTOR_0];
	v->SHORT_NTC_Motor_0.Counter = 0;
	v->BUF_IC_FB.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_BUFFER_IC_ERROR];
	v->BUF_IC_FB.Counter = 0;
	v->UVP_13V.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_UNDER_VOLTAGE_13V];
	v->UVP_13V.Counter = 0;
	v->UVP_E5V.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_UNDER_VOLTAGE_E5V];
	v->UVP_E5V.Counter = 0;
	v->UVP_ES5V.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_UNDER_VOLTAGE_ES5V];
	v->UVP_ES5V.Counter = 0;
	v->UVP_EA5V.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_UNDER_VOLTAGE_EA5V];
	v->UVP_EA5V.Counter = 0;
	v->POWER_TRANSISTOR_OC.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_POWER_TRANSISTOR_OC];
	v->POWER_TRANSISTOR_OC.Counter = 0;
	v->CAN0Timeout.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_CAN0_TIMEOUT];
	v->CAN0Timeout.Counter = 0;
	v->POWER_TRANSISTOR_OC.Counter = 0;
	v->CAN1Timeout.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_CAN1_TIMEOUT];
	v->CAN1Timeout.Counter = 0;
	v->ACC_PEDAL_SENSOR_BREAK.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_ACC_PEDAL_BREAK];
	v->ACC_PEDAL_SENSOR_BREAK.Counter = 0;
	v->ACC_PEDAL_SENSOR_SHORT.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_ACC_PEDAL_SHORT];
	v->ACC_PEDAL_SENSOR_SHORT.Counter = 0;
	v->KILL_SWITCH_INVALID.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_KILL_SWITCH_INVALID];
	v->KILL_SWITCH_INVALID.Counter = 0;

	// set threshold from external flash (P3-00~P3-99) for alarm detected by AlarmDetect_Accumulation
	// This function sholud be executed after ParamMgr1.Init, P3-00 is index 0. Max P3-99 is index 0x63
	SetAlarmThreshold(&v->CAN1Timeout, ALARMID_CAN1_TIMEOUT);
	SetAlarmThreshold(&v->POWER_TRANSISTOR_OC, ALARMID_POWER_TRANSISTOR_OC);
	SetAlarmThreshold(&v->BUF_IC_FB, ALARMID_BUFFER_IC_ERROR);
	SetAlarmThreshold(&v->OSP, ALARMID_MOTOR_OVER_SPEED);
	SetAlarmThreshold(&v->OVP_Bus, ALARMID_OVER_VOLTAGE_BUS);
	SetAlarmThreshold(&v->UVP_Bus, ALARMID_UNDER_VOLTAGE_BUS);
	SetAlarmThreshold(&v->UVP_13V, ALARMID_UNDER_VOLTAGE_13V);
	SetAlarmThreshold(&v->UVP_E5V, ALARMID_UNDER_VOLTAGE_E5V);
	SetAlarmThreshold(&v->UVP_ES5V, ALARMID_UNDER_VOLTAGE_ES5V);
	SetAlarmThreshold(&v->UVP_EA5V, ALARMID_UNDER_VOLTAGE_EA5V);
	//SetAlarmThreshold(&v->OCP_Iu, ALARMID_IU_OCP);
	//SetAlarmThreshold(&v->OCP_Iv, ALARMID_IV_OCP);
	//SetAlarmThreshold(&v->OCP_Iw, ALARMID_IW_OCP);
	SetAlarmThreshold(&v->ACC_PEDAL_SENSOR_BREAK, ALARMID_ACC_PEDAL_BREAK);
	SetAlarmThreshold(&v->ACC_PEDAL_SENSOR_SHORT, ALARMID_ACC_PEDAL_SHORT);
	SetAlarmThreshold(&v->OTP_PCU_0, ALARMID_OT_PCU_0);
	SetAlarmThreshold(&v->OTP_PCU_1, ALARMID_OT_PCU_1);
	SetAlarmThreshold(&v->OTP_PCU_2, ALARMID_OT_PCU_2);
	SetAlarmThreshold(&v->OTP_Motor_0, ALARMID_OT_MOTOR_0);
	SetAlarmThreshold(&v->OTP_PCU_0_WARNING, ALARMID_OT_PCU_0_WARNING);
	SetAlarmThreshold(&v->OTP_PCU_1_WARNING, ALARMID_OT_PCU_1_WARNING);
	SetAlarmThreshold(&v->OTP_PCU_2_WARNING, ALARMID_OT_PCU_2_WARNING);
	SetAlarmThreshold(&v->OTP_Motor_0_WARNING, ALARMID_OT_MOTOR_0_WARNING);
	SetAlarmThreshold(&v->BREAK_NTC_PCU_0, ALARMID_BREAK_NTC_PCU_0);
	SetAlarmThreshold(&v->BREAK_NTC_PCU_1, ALARMID_BREAK_NTC_PCU_1);
	SetAlarmThreshold(&v->BREAK_NTC_PCU_2, ALARMID_BREAK_NTC_PCU_2);
	SetAlarmThreshold(&v->BREAK_NTC_Motor_0, ALARMID_BREAK_NTC_MOTOR_0);
	SetAlarmThreshold(&v->SHORT_NTC_PCU_0, ALARMID_SHORT_NTC_PCU_0);
	SetAlarmThreshold(&v->SHORT_NTC_PCU_1, ALARMID_SHORT_NTC_PCU_1);
	SetAlarmThreshold(&v->SHORT_NTC_PCU_2, ALARMID_SHORT_NTC_PCU_2);
	SetAlarmThreshold(&v->SHORT_NTC_Motor_0, ALARMID_SHORT_NTC_MOTOR_0);

	v->Do100HzLoop = (functypeAlarmDetect_Do100HzLoop)AlarmDetect_Do100HzLoop;
}

void AlarmDetect_DoCurrentLoop( AlarmDetect_t *v )
{
	// System alarm detect while 1st motor
	if( v->AxisID == 1 )
	{
		AlarmDetect_Accumulation( v, &v->OVP_Bus, (int16_t)v->pAdcStation->AdcTraOut.BatVdc );
	}

	// Register Phass loss alarm
	if( SystemTable.AlarmTableInfo[ALARMID_PHASE_LOSS].AlarmEnable == ALARM_ENABLE )
	{
		if( v->pPhaseLoss->Error == PHASE_LOSS_ERROR_HAPPEN )
		{
			v->RegisterAxisAlarm( v, ALARMID_PHASE_LOSS, SystemTable.AlarmTableInfo[ALARMID_PHASE_LOSS].AlarmType );
		}
	}
}

void AlarmDetect_DoPLCLoop( AlarmDetect_t *v )
{
	uint16_t Abnormal = ALARM_TYPE_NONE;
	// System alarm detect while 1st motor
	// TODO the following contain axis alarms and system alarms. System alarms should be detected in Drive.c
	// Replace "AlarmDetect_Accumulation" with "GlobalAlarmDetect_Accumulation" in the future, when there are multi-axis in single drive.
	if( v->AxisID == 1 )
	{
		AlarmDetect_Accumulation( v, &v->UVP_Bus, (int16_t)v->pAdcStation->AdcTraOut.BatVdc );
		AlarmDetect_Accumulation( v, &v->UVP_13V, (int16_t)v->pAdcStation->AdcTraOut.S13V8 );

		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOS_NTC_1].AdcGroupIndex][v->pAdcStation->ThermoCh[MOS_NTC_1].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOS_NTC_1);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_1, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOS_NTC_2].AdcGroupIndex][v->pAdcStation->ThermoCh[MOS_NTC_2].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOS_NTC_2);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_2, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[CAP_NTC].AdcGroupIndex][v->pAdcStation->ThermoCh[CAP_NTC].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<CAP_NTC);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_Motor_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcGroupIndex][v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOTOR_NTC_0_A0);

		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOS_NTC_1].AdcGroupIndex][v->pAdcStation->ThermoCh[MOS_NTC_1].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOS_NTC_1);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_1, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOS_NTC_2].AdcGroupIndex][v->pAdcStation->ThermoCh[MOS_NTC_2].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOS_NTC_2);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_2, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[CAP_NTC].AdcGroupIndex][v->pAdcStation->ThermoCh[CAP_NTC].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<CAP_NTC);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_Motor_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcGroupIndex][v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOTOR_NTC_0_A0);

	}

	// Axis alarm detect

	if( v->BUF_IC_FB.AlarmInfo.AlarmEnable == ALARM_ENABLE )
	{
		AlarmDetect_BufferIcFb( v, &v->BUF_IC_FB, HAL_GPIO_ReadPin( BUF_FB_DI_GPIO_Port, BUF_FB_DI_Pin ), HAL_GPIO_ReadPin( HWOCP_BKIN_GPIO_Port, HWOCP_BKIN_Pin ), v->AxisID );
	}

	int16_t TempACC0p001V; // in 0.001V
	TempACC0p001V = (int16_t)(v->pAdcStation->AdcTraOut.Pedal_V1 * 1000.0f);

	AlarmDetect_Accumulation( v, &v->ACC_PEDAL_SENSOR_BREAK, TempACC0p001V );
	AlarmDetect_Accumulation( v, &v->ACC_PEDAL_SENSOR_SHORT, TempACC0p001V );
	AlarmDetect_Accumulation( v, &v->UVP_E5V, (int16_t)(v->pAdcStation->AdcTraOut.E5V * 10.0f));
	AlarmDetect_Accumulation( v, &v->UVP_ES5V, (int16_t)(v->pAdcStation->AdcTraOut.ES5V * 10.0f));
	AlarmDetect_Accumulation( v, &v->UVP_EA5V, (int16_t)(v->pAdcStation->AdcTraOut.EA5V * 10.0f));

}

void AlarmDetect_Do100HzLoop( AlarmDetect_t *v )
{
	// System alarm detect while 1st motor
	if( v->AxisID == 1 )
	{
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<MOS_NTC_1) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_0, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_1] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_0_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_1] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<MOS_NTC_2) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_1, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_2] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_1_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[MOS_NTC_2] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<CAP_NTC) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_2, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_2_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[CAP_NTC] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<MOTOR_NTC_0_A0) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_Motor_0, (int16_t)v->pAdcStation->AdcTraOut.MOTOR_NTC_0 );
			AlarmDetect_Accumulation( v, &v->OTP_Motor_0_WARNING, (int16_t)v->pAdcStation->AdcTraOut.MOTOR_NTC_0 );
		}
		AlarmDetect_Accumulation( v, &v->CAN1Timeout, 1 );
	}
	// Axis alarm detect

	AlarmDetect_Accumulation( v, &v->OSP, (int16_t) v->pSpeedInfo->MotorMechSpeedRPM );
	AlarmDetect_Accumulation( v, &v->KILL_SWITCH_INVALID, v->KillSwitchStatus );

}

