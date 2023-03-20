/*
 * AlarmTable.h
 *
 *  Created on: 2020年2月25日
 *      Author: MikeSFWen
 */

#ifndef INC_ALARMTABLE_H_
#define INC_ALARMTABLE_H_

#include "stdint.h"

#define MAX_ALARM_NUM					128
#define ALARM_COUNTER_MAX				200
#define ALARM_THRESHOLD_MAX				0xFFFF
#define MAX_NOW_ALARM_SIZE 				10
#define MAX_HISTORY_ALARM_SIZE 			10

#define ALARMID_UNUSED					0xFF

#define ALARMID_CAN0_COMM_ERROR			0x01
#define ALARMID_CAN0_TIMEOUT			0x02
#define ALARMID_CAN1_COMM_ERROR			0x03
#define ALARMID_CAN1_TIMEOUT			0x04

#define ALARMID_POWER_TRANSISTOR_OC		0x10
#define ALARMID_BUFFER_IC_ERROR			0x11
#define ALARMID_PHASE_LOSS				0x12
#define ALARMID_MOTOR_OVER_SPEED		0x13
#define ALARMID_OVER_VOLTAGE_BUS		0x14
#define ALARMID_UNDER_VOLTAGE_BUS		0x15
#define ALARMID_UNDER_VOLTAGE_13V		0x16
#define ALARMID_IU_OCP					0x17
#define ALARMID_IV_OCP					0x18
#define ALARMID_IW_OCP					0x19
#define ALARMID_FLASH_UNINITIALIZED		0x1A
#define ALARMID_FLASH_READ_FAILED		0x1B
#define ALARMID_FLASH_DAMAGED			0x1C

#define ALARMID_FOIL_BREAK				0x22
#define ALARMID_FOIL_SHORT				0x23

#define ALARMID_OT_PCU_0				0x30
#define ALARMID_BREAK_NTC_PCU_0			0x31
#define ALARMID_SHORT_NTC_PCU_0			0x32
#define ALARMID_OT_PCU_1				0x33
#define ALARMID_BREAK_NTC_PCU_1			0x34
#define ALARMID_SHORT_NTC_PCU_1			0x35
#define ALARMID_OT_PCU_2				0x36
#define ALARMID_BREAK_NTC_PCU_2			0x37
#define ALARMID_SHORT_NTC_PCU_2			0x38
#define ALARMID_OT_MOTOR_0				0x39
#define ALARMID_BREAK_NTC_MOTOR_0		0x3A
#define ALARMID_SHORT_NTC_MOTOR_0		0x3B

#define ALARMID_MOTORSTALL				0x40

#define ALARMID_RC_ABNORMAL				0x62


enum E_ALARM_ENABLE {
	ALARM_DISABLE = 0,
	ALARM_ENABLE
};

enum E_ALARM_TRIGGER_MODE {
	TRIG_MODE_HIGH = 0,
	TRIG_MODE_LOW
};

enum E_ALARMMGR_TYPE {
	ALARM_TYPE_NONE,
	ALARM_TYPE_WARNING,
	ALARM_TYPE_ERROR,
	ALARM_TYPE_CRITICAL
};

typedef struct {
	uint16_t AlarmID:8;			// Alarm ID
	uint16_t AlarmEnable:1;		// Whether Enable
	uint16_t AlarmType:6;		// Default Alarm Type
	uint16_t AlarmMode:1;		// Default Alarm Mode, 0 = Increasing, 1 = Decreasing
	uint16_t AlarmThreshold;	// Counter increase threshold
	uint16_t ErrorCounter:8;	// trigger error
	uint16_t WarningCounter:8;	// trigger warning
} AlarmTableInfo_t;

#define ALARMTABLEINFO_DEFAULT { \
	0, \
	0, \
	0 }

extern const AlarmTableInfo_t AlarmTableInfo[MAX_ALARM_NUM];

#endif /* INC_ALARMTABLE_H_ */
