/*
 * AlarmDetect.c
 *
 *  Created on: 2020年2月25日
 *      Author: MikeSFWen
 */

#include "AlarmDetect.h"
#include "RcUartComm.h"
#define ABS(x) 	( (x) > 0 ? (x) : -(x) )
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
			// error debounce
			if( p->Counter > p->AlarmInfo.ErrorCounter )
			{
				v->RegisterAxisAlarm( v, p->AlarmInfo.AlarmID, ALARM_TYPE_ERROR );
				Abnormal = ALARM_TYPE_ERROR;
			}

			// alarm debounce
			else if( p->Counter > p->AlarmInfo.WarningCounter )
			{
				v->RegisterAxisAlarm( v, p->AlarmInfo.AlarmID, ALARM_TYPE_WARNING );
				Abnormal = ALARM_TYPE_WARNING;
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
	if (v->AlarmInfo.AlarmEnable == ALARM_ENABLE)
	{
		v->AlarmInfo.AlarmThreshold = *(&DriveParams.PCUParams.Reserved300 + index);
	}
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
	v->POWER_TRANSISTOR_OC.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_POWER_TRANSISTOR_OC];
	v->POWER_TRANSISTOR_OC.Counter = 0;
	v->CAN0Timeout.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_CAN0_TIMEOUT];
	v->CAN0Timeout.Counter = 0;
	v->POWER_TRANSISTOR_OC.Counter = 0;
	v->CAN1Timeout.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_CAN1_TIMEOUT];
	v->CAN1Timeout.Counter = 0;
	v->RC_INVALID.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_RC_INVALID];
	v->RC_INVALID.Counter = 0;
#if USE_FOIL_ABNORMAL_DETECT
	v->FOIL_SENSOR_BREAK.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_FOIL_BREAK];
	v->FOIL_SENSOR_BREAK.Counter = 0;
	v->FOIL_SENSOR_SHORT.AlarmInfo = SystemTable.AlarmTableInfo[ALARMID_FOIL_SHORT];
	v->FOIL_SENSOR_SHORT.Counter = 0;
	if(IsUseDigitalFoilSensor==1)
	{
		v->FOIL_SENSOR_BREAK.AlarmInfo.AlarmEnable = ALARM_DISABLE;
		v->FOIL_SENSOR_SHORT.AlarmInfo.AlarmEnable = ALARM_DISABLE;
	}
#endif

	// set threshold from external flash (P3-00~P3-99) for alarm detected by AlarmDetect_Accumulation
	// This function sholud be executed after ParamMgr1.Init, P3-00 is index 0. Max P3-99 is index 0x63
	//SetAlarmThreshold(&v->CAN0Timeout, ALARMID_CAN1_TIMEOUT);
	SetAlarmThreshold(&v->CAN1Timeout, ALARMID_CAN1_TIMEOUT);
	SetAlarmThreshold(&v->POWER_TRANSISTOR_OC, ALARMID_POWER_TRANSISTOR_OC);
	SetAlarmThreshold(&v->BUF_IC_FB, ALARMID_BUFFER_IC_ERROR);
	SetAlarmThreshold(&v->OSP, ALARMID_MOTOR_OVER_SPEED);
	SetAlarmThreshold(&v->OVP_Bus, ALARMID_OVER_VOLTAGE_BUS);
	SetAlarmThreshold(&v->UVP_Bus, ALARMID_UNDER_VOLTAGE_BUS);
	SetAlarmThreshold(&v->UVP_13V, ALARMID_UNDER_VOLTAGE_13V);
	//SetAlarmThreshold(&v->OCP_Iu, ALARMID_IU_OCP);
	//SetAlarmThreshold(&v->OCP_Iv, ALARMID_IV_OCP);
	//SetAlarmThreshold(&v->OCP_Iw, ALARMID_IW_OCP);
	SetAlarmThreshold(&v->FOIL_SENSOR_BREAK, ALARMID_FOIL_BREAK);
	SetAlarmThreshold(&v->FOIL_SENSOR_SHORT, ALARMID_FOIL_SHORT);
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
	//SetAlarmThreshold(&v->THROT_ERROR_SHORT, ALARMID_CAN1_TIMEOUT);
	//SetAlarmThreshold(&v->THROT_ERROR_BREAK, ALARMID_CAN1_TIMEOUT);
	SetAlarmThreshold(&v->RC_INVALID, ALARMID_RC_INVALID);
	//

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
		AlarmDetect_Accumulation( v, &v->UVP_13V, (int16_t)v->pAdcStation->AdcTraOut.V13 );

		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_0].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_0);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_1, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_1].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_1].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_1);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_PCU_2, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_2].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_2].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_2);
		Abnormal = AlarmDetect_Accumulation( v, &v->BREAK_NTC_Motor_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcGroupIndex][v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOTOR_NTC_0_A0);

		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_0].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_0);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_1, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_1].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_1].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_1);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_PCU_2, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[PCU_NTC_2].AdcGroupIndex][v->pAdcStation->ThermoCh[PCU_NTC_2].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<PCU_NTC_2);
		Abnormal = AlarmDetect_Accumulation( v, &v->SHORT_NTC_Motor_0, (int16_t)v->pAdcStation->AdcDmaData[v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcGroupIndex][v->pAdcStation->ThermoCh[MOTOR_NTC_0_A0].AdcRankIndex] );
		if ( Abnormal!= ALARM_TYPE_NONE ) v->pAdcStation->NTCIsAbnormal |= (((uint16_t)1)<<MOTOR_NTC_0_A0);

	}

	// Axis alarm detect

	if( v->BUF_IC_FB.AlarmInfo.AlarmEnable == ALARM_ENABLE )
	{
		AlarmDetect_BufferIcFb( v, &v->BUF_IC_FB, HAL_GPIO_ReadPin( BUF_FB_GPIO_Port, BUF_FB_Pin ), HAL_GPIO_ReadPin( HWOCP_GPIO_Port, HWOCP_Pin ), v->AxisID );
	}

#if USE_FOIL_ABNORMAL_DETECT
	// analog foil sensor should in 5~0V, and the input signal is between 5000~0000 (0.001V)
	int16_t TempAnalogFoil0p001V; // in 0.001V
	if(v->pAdcStation->AdcTraOut.Foil < 0.00001f)
	{
		TempAnalogFoil0p001V = 0; // 0.00000 * 1000 = 0
	}
	else if(v->pAdcStation->AdcTraOut.Foil > 4.99999f)
	{
		TempAnalogFoil0p001V = 5000; // 5.00000 * 1000 = 5000
	}
	else
	{
		TempAnalogFoil0p001V = v->pAdcStation->AdcTraOut.Foil * 1000.0f;
	}
	AlarmDetect_Accumulation( v, &v->FOIL_SENSOR_BREAK, TempAnalogFoil0p001V );
	AlarmDetect_Accumulation( v, &v->FOIL_SENSOR_SHORT, TempAnalogFoil0p001V );
#endif

}

void AlarmDetect_Do100HzLoop( AlarmDetect_t *v )
{
	// System alarm detect while 1st motor
	if( v->AxisID == 1 )
	{
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<PCU_NTC_0) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_0, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_0] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_0_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_0] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<PCU_NTC_1) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_1, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_1] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_1_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_1] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<PCU_NTC_2) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_PCU_2, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_2] );
			AlarmDetect_Accumulation( v, &v->OTP_PCU_2_WARNING, (int16_t)v->pAdcStation->AdcTraOut.PCU_NTC[PCU_NTC_2] );
		}
		if( ( v->pAdcStation->NTCIsAbnormal & (((uint16_t)1)<<MOTOR_NTC_0_A0) ) == 0 )
		{
			AlarmDetect_Accumulation( v, &v->OTP_Motor_0, (int16_t)v->pAdcStation->AdcTraOut.MOTOR_NTC );
			AlarmDetect_Accumulation( v, &v->OTP_Motor_0_WARNING, (int16_t)v->pAdcStation->AdcTraOut.MOTOR_NTC );
		}
		AlarmDetect_Accumulation( v, &v->CAN1Timeout, 1 );
	}
	// Axis alarm detect
	AlarmDetect_Accumulation( v, &v->OSP, (int16_t)ABS( v->pSpeedInfo->MotorMechSpeedRPM ) );

	// PWM RC SIGNAL ABNORMAL, check if RC have connected to RF after ESC power on.
	if( RCCommCtrl.RcHaveConnectedFlag == 1){
		AlarmDetect_Accumulation( v, &v->RC_INVALID, RCCommCtrl.TimeoutCnt );
	}
}

