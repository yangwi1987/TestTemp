/*
 * SystemTableLinker.c
 *
 *  Created on: 2020年5月5日
 *      Author: Mike.Wen.SFW
 */
#if BME
#include "SystemTableLinker.h"
#include "ConstantParamAndUseFunction.h"

__attribute__((__section__(".SystemBin"),used)) const System_Table_t_Linker SystemTable =
{
		.Version = {01, 0x02, 1, 2}, /* Version numbers are undefined now. If version number are all zero, const table cannot work in bootloader. */
		.NumOfHeader = 0,
		.CheckSum = 0,

		// When use AlarmDetect_Accumulation, Alarm type depending on CriAlarmCounter or NonCriAlarmCounter.
		// AlarmTableInfo_t:
		//		AlarmID,						AlarmEnable,   AlarmType,		AlarmMode,		AlarmThreshold,		  CriAlarmCounter,	 NonCriAlarmCounter
		.AlarmTableInfo = \
		{		// 0x00 ~ 0x0F
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_CAN0_COMM_ERROR,		ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_CAN0_TIMEOUT, 			ALARM_DISABLE, ALARM_TYPE_NONE, 	TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_CAN1_COMM_ERROR,		ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_CAN1_TIMEOUT, 			ALARM_ENABLE, ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  0, 	ALARM_COUNTER_MAX, 100},
#if IS_MF_CODE_BIN_FILE
				{ALARMID_PSB1_COMM_FAIL, 		ALARM_DISABLE, 	ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_PSB_WRONG_CHECKSUM,	ALARM_DISABLE, 	ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#elif BME
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#endif
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				// 0x10 ~ 0x1F
				{ALARMID_POWER_TRANSISTOR_OC, 	ALARM_ENABLE, ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH, ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_BUFFER_IC_ERROR, 	  	ALARM_ENABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#if IS_MF_CODE_BIN_FILE
				{ALARMID_PHASE_LOSS, 	 	  	ALARM_DISABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#elif BME&EVT
				{ALARMID_PHASE_LOSS, 	 	  	ALARM_ENABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#endif
				{ALARMID_MOTOR_OVER_SPEED, 	  ALARM_ENABLE, ALARM_TYPE_CRITICAL, 	 TRIG_MODE_HIGH, 8000, 100, ALARM_COUNTER_MAX},
				{ALARMID_OVER_VOLTAGE_BUS, 	  ALARM_ENABLE, ALARM_TYPE_CRITICAL, 	 TRIG_MODE_HIGH, 62,   10, ALARM_COUNTER_MAX},
				{ALARMID_UNDER_VOLTAGE_BUS,   	ALARM_DISABLE, 	ALARM_TYPE_CRITICAL,   TRIG_MODE_LOW,   36,    10, ALARM_COUNTER_MAX},
				{ALARMID_UNDER_VOLTAGE_13V,   	ALARM_ENABLE, 	ALARM_TYPE_CRITICAL, 	TRIG_MODE_LOW,   10,    10, ALARM_COUNTER_MAX},
				{ALARMID_IU_OCP, 			  	ALARM_DISABLE,	ALARM_TYPE_CRITICAL, 	TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_IV_OCP, 			  	ALARM_DISABLE, 	ALARM_TYPE_CRITICAL, 	TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_IW_OCP, 		  	  	ALARM_DISABLE, 	ALARM_TYPE_CRITICAL, 	TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_FLASH_UNINITIALIZED, ALARM_ENABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, 0,		 0, ALARM_COUNTER_MAX},
				{ALARMID_FLASH_READ_FAILED,   ALARM_ENABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, 0,		 0, ALARM_COUNTER_MAX},
				{ALARMID_FLASH_DAMAGED,   	  ALARM_ENABLE, ALARM_TYPE_CRITICAL,    TRIG_MODE_HIGH, 0,		 0, ALARM_COUNTER_MAX},
				{ALARMID_MOTOR_REVERSE, 		ALARM_ENABLE, ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  100, ALARM_COUNTER_MAX, 100},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, 				ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				// 0x20 ~ 0x2F
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#if USE_ANALOG_FOIL_SENSOR_FUNC
				{ALARMID_FOIL_BREAK, ALARM_ENABLE, ALARM_TYPE_NONCRITICAL, TRIG_MODE_LOW,  125, ALARM_COUNTER_MAX, 10},
				{ALARMID_FOIL_SHORT, ALARM_ENABLE, ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  4752, ALARM_COUNTER_MAX, 10},
#else
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
#endif
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				// 0x30 ~ 0x3F
				{ALARMID_OT_PCU_0, 		 	ALARM_ENABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  120,   5,  	ALARM_COUNTER_MAX},
				{ALARMID_BREAK_NTC_PCU_0, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  3900, ALARM_COUNTER_MAX, 	10},
				{ALARMID_SHORT_NTC_PCU_0, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_LOW,   120,  ALARM_COUNTER_MAX, 	10},
				{ALARMID_OT_PCU_1, 		 	ALARM_ENABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  120,   5, 	ALARM_COUNTER_MAX},
				{ALARMID_BREAK_NTC_PCU_1, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  3900, ALARM_COUNTER_MAX, 	10},
				{ALARMID_SHORT_NTC_PCU_1, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_LOW,   120,  ALARM_COUNTER_MAX, 	10},
				{ALARMID_OT_PCU_2, 			ALARM_ENABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  115,  5,  	ALARM_COUNTER_MAX},
				{ALARMID_BREAK_NTC_PCU_2, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  4065, ALARM_COUNTER_MAX, 	10},
				{ALARMID_SHORT_NTC_PCU_2, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_LOW,   100,  ALARM_COUNTER_MAX, 	10},
#if IS_MF_CODE_BIN_FILE
				{ALARMID_OT_MOTOR_0,	 	ALARM_DISABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  180,  5,  	ALARM_COUNTER_MAX},
				{ALARMID_BREAK_NTC_MOTOR_0, ALARM_DISABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  4075, 10, 	ALARM_COUNTER_MAX},
				{ALARMID_SHORT_NTC_MOTOR_0, ALARM_DISABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_LOW,   200,  10, 	ALARM_COUNTER_MAX},
#else
				{ALARMID_OT_MOTOR_0,	 	ALARM_ENABLE,  ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  175,  5, 	 	ALARM_COUNTER_MAX},
				{ALARMID_BREAK_NTC_MOTOR_0, ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_HIGH,  4075, ALARM_COUNTER_MAX, 	10},
				{ALARMID_SHORT_NTC_MOTOR_0, ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, TRIG_MODE_LOW,   200,  ALARM_COUNTER_MAX, 	10},
#endif
				{ALARMID_OT_PCU_0_WARNING, 	ALARM_ENABLE, ALARM_TYPE_NONCRITICAL,  TRIG_MODE_HIGH,  115, ALARM_COUNTER_MAX, 10},
				{ALARMID_OT_PCU_1_WARNING, 	ALARM_ENABLE, ALARM_TYPE_NONCRITICAL,  TRIG_MODE_HIGH,  115, ALARM_COUNTER_MAX, 10},
				{ALARMID_OT_PCU_2_WARNING, 	ALARM_ENABLE, ALARM_TYPE_NONCRITICAL,  TRIG_MODE_HIGH,  110, ALARM_COUNTER_MAX, 10},
				{ALARMID_OT_MOTOR_0_WARNING,ALARM_ENABLE, ALARM_TYPE_NONCRITICAL,  TRIG_MODE_HIGH,  170, ALARM_COUNTER_MAX, 10},
				// 0x40 ~ 0x4F
				{ALARMID_MOTORSTALL, ALARM_ENABLE, ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},

				// 0x50 ~ 0x5F
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				// 0x60 ~ 0x6F
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_RC_INVALID, 	ALARM_ENABLE,  ALARM_TYPE_NONCRITICAL, 	TRIG_MODE_HIGH,  9, ALARM_COUNTER_MAX, 3},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				// 0x70 ~ 0x7F
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_CRITICAL, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX},
				{ALARMID_UNUSED, ALARM_DISABLE, ALARM_TYPE_NONE, TRIG_MODE_HIGH,  ALARM_THRESHOLD_MAX, ALARM_COUNTER_MAX, ALARM_COUNTER_MAX}
		},

		.BackRollTable = \
		{
				5.8, -500.0
		},

		.MotorReserveTableInfo = \
		{
			.reserve1 = 0.0,
			.reserve2 = 0.0,
			.reserve3 = 0.0,
			.reserve4 = 0.0,
			.reserve5 = 0.0,
			.reserve6 = 0.0,
			.p2DTable = &SystemTable.ReservedTable[0][0]
		},

		.ReservedTable = \
		{
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0},
		},

		.WindingDeratingTable = { 2600, 2340, 2110, 1900, 1710, 1540, 1400, 1400 },
		.WindingDeratingInfo = { 8, 1.0f, 164.0f, 0.1f, &SystemTable.WindingDeratingTable[0] },

		.MosDeratingTable = { 2600, 2480, 2360, 2240, 2120, 2000, 1880, 1760, 1640, 1520, 1400, 1400 },
		.MosDeratingInfo = { 12, 1.0f, 50.0f, 0.1f, &SystemTable.MosDeratingTable[0] },

		.CapDeratingTable = { 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600, 2600},
		.CapDeratingInfo = { 22, 1.0f, 110.0f, 0.1f, &SystemTable.CapDeratingTable[0] },

		.SysParamTableInfoArray =
		{
				// Property = {HasShift[11], 10^n[10:8], read location[7:6], size[5], decimal[4:3], Authority[2:0]}. See in ParamTable.h.
				// ============================ P0 SystemParams ============================
				//    Min,		  Max,	  Default,   Property,		*pAddr
				{	    0,			0,			0,		    0,		0},	//P0-00
				{	   10,		10000,		  700,		 0x4A,		&DriveParams.SystemParams.BattVoltMax},	//P0-01
				{	   10,		10000,		  240,		 0x4A,		&DriveParams.SystemParams.BattVoltMin},	//P0-02
				{		0,			0,		    0,		    0,		0},	//P0-03
				{		0,			0,		    0,		    0,		0},	//P0-04
				{		0,			0,		    0,		    0,		0},	//P0-05
				{		0,			0,		    0,		    0,		0},	//P0-06
				{		0,			0,		    0,		    0,		0},	//P0-07
				{		0,			0,		    0,		    0,		0},	//P0-08
				{		0,			0,		    0,		    0,		0},	//P0-09
				{		0,			0,		    0,		    0,		0},	//P0-10
				{		0,			0,		    0,		    0,		0},	//P0-11
				{		0,			0,		    0,		    0,		0},	//P0-12
				{		0,			0,		    0,		    0,		0},	//P0-13
				{		0,			0,		    0,		    0,		0},	//P0-14
				{		0,			0,			0,			0,		0},	//P0-15
				{		0,			0,			0,			0,		0},	//P0-16
				{		0,			0,			0,			0,		0},	//P0-17
				{		0,			0,			0,			0,		0},	//P0-18
				{		0,			0,			0,			0,		0},	//P0-19
				{		0,			0,			0,			0,		0},	//P0-20
				{		0,			0,			0,			0,		0},	//P0-21
				{		0,			0,			0,			0,		0},	//P0-22
				{		0,			0,			0,			0,		0},	//P0-23
				{		0,			0,			0,			0,		0},	//P0-24
				{		0,			0,			0,			0,		0},	//P0-25
				{		0,			0,			0,			0,		0},	//P0-26
				{		0,			0,			0,			0,		0},	//P0-27
				{		0,			0,			0,			0,		0},	//P0-28
				{		0,			0,			0,			0,		0},	//P0-29
				{		0,			0,			0,			0,		0},	//P0-30
				{		0,			0,			0,			0,		0},	//P0-31
				{		0,			0,			0,			0,		0},	//P0-32
				{		0,			0,			0,			0,		0},	//P0-33
				{		0,			0,			0,			0,		0},	//P0-34
				{		0,			0,			0,			0,		0},	//P0-35
				{		0,			0,			0,			0,		0},	//P0-36
				{		0,			0,			0,			0,		0},	//P0-37
				{		0,			0,			0,			0,		0},	//P0-38
				{		0,			0,			0,			0,		0},	//P0-39
				{		0,			0,			0,			0,		0},	//P0-40
				{		0,			0,			0,			0,		0},	//P0-41
				{		0,			0,			0,			0,		0},	//P0-42
				{		0,			0,			0,			0,		0},	//P0-43
				{		0,			0,			0,			0,		0},	//P0-44
				{		0,			0,			0,			0,		0},	//P0-45
				{		0,			0,			0,			0,		0},	//P0-46
				{		0,			0,			0,			0,		0},	//P0-47
				{		0,			0,			0,			0,		0},	//P0-48
				{		0,			0,			0,			0,		0},	//P0-49
				{		0,			0,			0,			0,		0},	//P0-50
				{		0,			0,			0,			0,		0},	//P0-51
				{		0,			0,			0,			0,		0},	//P0-52
				{		0,			0,			0,			0,		0},	//P0-53
				{		0,			0,			0,			0,		0},	//P0-54
				{		32768,		33968,		33348,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_START][0]},	//P0-55
				{		32768,		33968,		33348,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_START][1]},	//P0-56
				{		32768,		33968,		33348,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_START][2]},	//P0-57
				{		32768,		33968,		33348,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_START][3]},	//P0-58
				{		32768,		33968,		33348,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_START][4]},	//P0-59
				{		32768,		33968,		33868,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_MAX][0]},	//P0-60
				{		32768,		33968,		33868,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_MAX][1]},	//P0-61
				{		32768,		33968,		33868,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_MAX][2]},	//P0-62
				{		32768,		33968,		33868,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_MAX][3]},	//P0-63
				{		32768,		33968,		33868,		0x852,		&DriveParams.SystemParams.DriveCurve[DRIVE_PROPULSION_MAX][4]},	//P0-64
				{		32768,		33768,		32777,		0xA42,		&DriveParams.SystemParams.DriveCurve[DRIVE_POWER_MAX][0]},	//P0-65
				{		32768,		33768,		32777,		0xA42,		&DriveParams.SystemParams.DriveCurve[DRIVE_POWER_MAX][1]},	//P0-66
				{		32768,		33768,		32838,		0xA42,		&DriveParams.SystemParams.DriveCurve[DRIVE_POWER_MAX][2]},	//P0-67
				{		32768,		33768,		32838,		0xA42,		&DriveParams.SystemParams.DriveCurve[DRIVE_POWER_MAX][3]},	//P0-68
				{		32768,		33768,		32768,		0xA42,		&DriveParams.SystemParams.DriveCurve[DRIVE_POWER_MAX][4]},	//P0-69
				{		32768,		40768,		39400,		0x842,		&DriveParams.SystemParams.DriveCurve[DRIVE_SPEED_MAX][0]},	//P0-70
				{		32768,		40768,		39400,		0x842,		&DriveParams.SystemParams.DriveCurve[DRIVE_SPEED_MAX][1]},	//P0-71
				{		32768,		40768,		39400,		0x842,		&DriveParams.SystemParams.DriveCurve[DRIVE_SPEED_MAX][2]},	//P0-72
				{		32768,		40768,		39400,		0x842,		&DriveParams.SystemParams.DriveCurve[DRIVE_SPEED_MAX][3]},	//P0-73
				{		32768,		40768,		39400,		0x842,		&DriveParams.SystemParams.DriveCurve[DRIVE_SPEED_MAX][4]},	//P0-74
				{		32768,		62768,		32773,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_START][0]},	//P0-75
				{		32768,		62768,		32773,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_START][1]},	//P0-76
				{		32768,		62768,		32773,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_START][2]},	//P0-77
				{		32768,		62768,		32773,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_START][3]},	//P0-78
				{		32768,		62768,		32773,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_START][4]},	//P0-79
				{		2768,		32768,		32762,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_END][0]},	//P0-80
				{		2768,		32768,		32762,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_END][1]},	//P0-81
				{		2768,		32768,		32742,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_END][2]},	//P0-82
				{		2768,		32768,		32742,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_END][3]},	//P0-83
				{		2768,		32768,		32762,		0x85A,		&DriveParams.SystemParams.DriveCurve[DRIVE_SLOPE_END][4]},	//P0-84
				{		32768,		62768,		62768,		0x84A,		&DriveParams.SystemParams.DriveRisingRamp},	//P0-85
				{		2768,		32768,		2768,		0x84A,		&DriveParams.SystemParams.DriveFallingRamp},	//P0-86
				{		30,			100,		30,		    0x4A,		&DriveParams.SystemParams.LimpTransitSec},	//P0-87
				{		2,			8,			8,		    0x42,		&DriveParams.SystemParams.HFIInjVol},	//P0-88
				{		2,			20,			10,			0x42,		&DriveParams.SystemParams.IinitialAlignDelay},	//P0-89
				{		0,			0,			0,			0,		0},	//P0-90
				{		0,			0,			0,			0,		0},	//P0-91
				{	32768,		62768,		32798,		0x942,		&DriveParams.SystemParams.DrivePowerRampUpZeroThrottle},	//P0-92
				{	 2768,		32768,		32168,		0x942,		&DriveParams.SystemParams.DrivePowerRampDownZeroThrottle},	//P0-93
				{	32768,		62768,		32798,		0x942,		&DriveParams.SystemParams.DrivePowerRampUpFullThrottle},	//P0-94
				{	 2768,		32768,		32738,		0x942,		&DriveParams.SystemParams.DrivePowerRampDownFullThrottle},	//P0-95
				{	32769,		33268,		33268,		0x842,		&DriveParams.SystemParams.DrainRisingRamp},		//P0-96
				{	32268,		32767,		32268,		0x842,		&DriveParams.SystemParams.DrainFallingRamp},	//P0-97
				{		0,			0,			0,			0,		0},	//P0-98
				{		0,			0,			0,			0,		0},		//P0-99
				{	 2000,		 3850,		 2300,		 0x41,		&DriveParams.SystemParams.ThrottleMaxAdc},	//P1-00
				{		0,		  500,	 	  130,		 0x41,		&DriveParams.SystemParams.ThrottleMinAdc},	//P1-01
				{		0,		65535,			0,		 0x41,		&DriveParams.SystemParams.ThrottleScale[0]},	//P1-02
				{		0,		65535,			0,	     0x41,		&DriveParams.SystemParams.ThrottleScale[1]},	//P1-03
				{		0,		  100,		   10,		 0x52,		&DriveParams.SystemParams.ThrottleEmptyPt[0]},	//P1-04
				{		0,		  100,		   10,		 0x52,		&DriveParams.SystemParams.ThrottleEmptyPt[1]},	//P1-05
				{		0,		  100,		   10,		 0x52,		&DriveParams.SystemParams.ThrottleEmptyPt[2]},	//P1-06
				{		0,		  100,		   10,		 0x52,		&DriveParams.SystemParams.ThrottleEmptyPt[3]},	//P1-07
				{		0,		  100,		   10,		 0x52,		&DriveParams.SystemParams.ThrottleEmptyPt[4]},	//P1-08
				{		0,		  100,	   	   50,		 0x52,		&DriveParams.SystemParams.ThrottleHalfPt[0]},	//P1-09
				{		0,		  100,	   	   50,		 0x52,		&DriveParams.SystemParams.ThrottleHalfPt[1]},	//P1-10
				{		0,		  100,	   	   50,		 0x52,		&DriveParams.SystemParams.ThrottleHalfPt[2]},	//P1-11
				{		0,		  100,		   50,		 0x52,		&DriveParams.SystemParams.ThrottleHalfPt[3]},	//P1-12
				{		0,		  100,		   50,		 0x52,		&DriveParams.SystemParams.ThrottleHalfPt[4]},	//P1-13
				{		0,		  100,		   90,		 0x52,		&DriveParams.SystemParams.ThrottleFullPt[0]},	//P1-14
				{		0,		  100,		   90,		 0x52,		&DriveParams.SystemParams.ThrottleFullPt[1]},	//P1-15
				{		0,		  100,		   90,		 0x52,		&DriveParams.SystemParams.ThrottleFullPt[2]},	//P1-16
				{		0,		  100,		   90,		 0x52,		&DriveParams.SystemParams.ThrottleFullPt[3]},	//P1-17
				{		0,		  100,		   90,		 0x52,		&DriveParams.SystemParams.ThrottleFullPt[4]},	//P1-18
				{		0,		 1000,		    5,		 0x5A,		&DriveParams.SystemParams.ThrottleRamp[0]},	//P1-19
				{		0,		 1000,		    5,		 0x5A,		&DriveParams.SystemParams.ThrottleRamp[1]},	//P1-20
				{		0,		 1000,		    5,		 0x5A,		&DriveParams.SystemParams.ThrottleRamp[2]},	//P1-21
				{		0,		 1000,			5,		 0x5A,		&DriveParams.SystemParams.ThrottleRamp[3]},	//P1-22
				{		0,		 1000,			5,		 0x5A,		&DriveParams.SystemParams.ThrottleRamp[4]},	//P1-23
				{		0,			0,			0,			0,		0},	//P1-24
				{		0,		65535,		   10,		 0x45,		&DriveParams.SystemParams.SecTimeThresholdForDriveLock},	//P1-25
				{		0,		 6000,		  100,		 0x45,		&DriveParams.SystemParams.RpmToStartCntDriveLock},	//P1-26
				{		0,			0,			0,			0,		0},	//P1-27
				{		0,			0,			0,			0,		0},	//P1-28
				{		0,			0,			0,			0,		0},	//P1-29
				{		0,			0,			0,			0,		0},	//P1-30
				{		0,			0,			0,			0,		0},	//P1-31
				{	    3,	  	   51,		  13,		 0x4A,		&DriveParams.SystemParams.MaxAnaFoilSenSurf0p1V},	//P1-32
				{	    3,	  	   51,		  5,		 0x4A,		&DriveParams.SystemParams.MinAnaFoilSenSurf0p1V},	//P1-33
				{	    3,	  	   51,		  36,		 0x4A,		&DriveParams.SystemParams.MaxAnaFoilSenFoil0p1V},	//P1-34
				{	    3,	  	   51,		  28,		 0x4A,		&DriveParams.SystemParams.MinAnaFoilSenFoil0p1V},	//P1-35
				{		0,			0,			0,			0,		0},	//P1-36
				{		0,			0,			0,			0,		0},	//P1-37
				{		0,			0,			0,			0,		0},	//P1-38
				{		0,			0,			0,			0,		0},	//P1-39
				{	    0,	  	   7,		  5,		 0x45,		&DriveParams.SystemParams.ParamMgrSecurity},	//P1-40
				{		0,			0,			0,			0,		0},	//P1-41
				{		0,			0,			0,			0,		0},	//P1-42
				{		0,			0,			0,			0,		0},	//P1-43
				{		0,			0,			0,			0,		0},	//P1-44
				{		0,			0,			0,			0,		0},	//P1-45
				{		0,			0,			0,			0,		0},	//P1-46
				{		0,			0,			0,			0,		0},	//P1-47
				{		0,			0,			0,			0,		0},	//P1-48
				{		0,			0,			0,			0,		0},	//P1-49
				{		0,			0,			0,			0,		0},	//P1-50
				{		0,			0,			0,			0,		0},	//P1-51
				{		0,			0,			0,			0,		0},	//P1-52
				{		0,			0,			0,			0,		0},	//P1-53
				{		0,			0,			0,			0,		0},	//P1-54
				{		0,			0,			0,			0,		0},	//P1-55
				{		0,			0,			0,			0,		0},	//P1-56
				{		0,			0,			0,			0,		0},	//P1-57
				{		0,			0,			0,			0,		0},	//P1-58
				{		0,			0,			0,			0,		0},	//P1-59
				{		0,			0,			0,			0,		0},	//P1-60
				{		0,			0,			0,			0,		0},	//P1-61
				{		0,			0,			0,			0,		0},	//P1-62
				{		0,			0,			0,			0,		0},	//P1-63
				{		0,			0,			0,			0,		0},	//P1-64
				{		0,			0,			0,			0,		0},	//P1-65
				{		0,			0,			0,			0,		0},	//P1-66
				{		0,			0,			0,			0,		0},	//P1-67
				{		0,			0,			0,			0,		0},	//P1-68
				{		0,			0,			0,			0,		0},	//P1-69
				{		0,			0,			0,			0,		0},	//P1-70
				{		0,			0,			0,			0,		0},	//P1-71
				{		0,			0,			0,			0,		0},	//P1-72
				{		0,			0,			0,			0,		0},	//P1-73
				{		0,			0,			0,			0,		0},	//P1-74
				{		0,			0,			0,			0,		0},	//P1-75
				{		0,			0,			0,			0,		0},	//P1-76
				{		0,			0,			0,			0,		0},	//P1-77
				{		0,			0,			0,			0,		0},	//P1-78
				{		0,			0,			0,			0,		0},	//P1-79
				{		0,			0,			0,			0,		0},	//P1-80
				{		0,			0,			0,			0,		0},	//P1-81
				{		0,			0,			0,			0,		0},	//P1-82
				{		0,			0,			0,			0,		0},	//P1-83
				{		0,			0,			0,			0,		0},	//P1-84
				{		0,			0,			0,			0,		0},	//P1-85
				{		0,			0,			0,			0,		0},	//P1-86
				{		0,			0,			0,			0,		0},	//P1-87
				{		0,			0,			0,			0,		0},	//P1-88
				{		0,			0,			0,			0,		0},	//P1-89
				{	    32688,	  	32808,		32768,		0x842,		&DriveParams.SystemParams.MotorDeratingTempOffset},	//P1-90
				{	    20,			250,		100,		0x52,		&DriveParams.SystemParams.MotorDeratingScale},	//P1-91
				{	    32728,	  	32808,		32768,		0x842,		&DriveParams.SystemParams.MosDeratingTempOffset},	//P1-92
				{	    20,			250,		100,		0x52,		&DriveParams.SystemParams.MOSDeratingScale},	//P1-93
				{		0,			0,			0,			0,		0},	//P1-94
				{		0,			0,			0,			0,		0},	//P1-95
				{		0,			0,			0,			0,		0},	//P1-96
				{		0,			0,			0,			0,		0},	//P1-97
				{		0,			0,			0,			0,		0},	//P1-98
				{		0,			0,			0,			0,		0},	//P1-99
		},
#if IS_MF_CODE_BIN_FILE
		.SysFunAllow=
		{
			0,
			0,
			1,
		},
#else
		.SysFunAllow=
		{
			0,
			0,
			1,
		},
#endif
		.CheckWord = 0x4321
};

#endif
