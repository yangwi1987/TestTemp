/*
 * GlobalAlarmDetect.c
 *
 *  Created on: 2021年4月16日
 *      Author: Kevin.Kuo
 */

#include "SystemTableLinker.h"
#include "ExtFlash.h"
#include "AlarmMgr.h"
#include "ICANInterface.h"
#include "GlobalAlarmDetect.h"

extern ExtFlash_t ExtFlash1;
extern AlarmMgr_t AlarmMgr1;
extern Axis_t Axis[MAX_AXIS_NUM];

void GlobalAlarmDetect_init( void )
{
	// register ExtFlash alarm
	if( SystemTable.AlarmTableInfo[ALARMID_FLASH_UNINITIALIZED].AlarmEnable == ALARM_ENABLE )
	{
		if( ExtFlash1.AlarmStatus & FLASHERROR_UNINITIALIZED )
		{
			AlarmMgr1.RegisterAlarm( &AlarmMgr1, TARGET_ID_GLOBAL, ALARMID_FLASH_UNINITIALIZED, SystemTable.AlarmTableInfo[ALARMID_FLASH_UNINITIALIZED].AlarmType );
		}
	}

	if( SystemTable.AlarmTableInfo[ALARMID_FLASH_READ_FAILED].AlarmEnable == ALARM_ENABLE )
	{
		if( (ExtFlash1.AlarmStatus & FLASHERROR_CHECKSUM_FAIL) ||
			(ExtFlash1.AlarmStatus & FLASHERROR_CHECKSUM_FAIL_TOTAL_TIME) )
		{
			AlarmMgr1.RegisterAlarm( &AlarmMgr1, TARGET_ID_GLOBAL, ALARMID_FLASH_READ_FAILED, SystemTable.AlarmTableInfo[ALARMID_FLASH_READ_FAILED].AlarmType );
		}
	}
}

void GlobalAlarmDetect_DoHouseKeeping( void )
{
	// register ExtFlash alarm
	if( SystemTable.AlarmTableInfo[ALARMID_FLASH_DAMAGED].AlarmEnable == ALARM_ENABLE )
	{
		if( ExtFlash1.AlarmStatus & FLASHERROR_DAMAGED )
		{
			AlarmMgr1.RegisterAlarm( &AlarmMgr1, TARGET_ID_GLOBAL, ALARMID_FLASH_DAMAGED, SystemTable.AlarmTableInfo[ALARMID_FLASH_DAMAGED].AlarmType );
		}
	}

	if( SystemTable.AlarmTableInfo[ALARMID_FLASH_READ_FAILED].AlarmEnable == ALARM_ENABLE )
	{
		if( ExtFlash1.AlarmStatus & FLASHERROR_CHECKSUM_FAIL )
		{
			AlarmMgr1.RegisterAlarm( &AlarmMgr1, TARGET_ID_GLOBAL, ALARMID_FLASH_READ_FAILED, SystemTable.AlarmTableInfo[ALARMID_FLASH_READ_FAILED].AlarmType );
		}
	}

}

void GlobalAlarmDetect_Accumulation( PROTECT_POLLING_TYPE *p, int Signal, int TargetID )
{
	uint16_t Trigger = 0;

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
			if( p->Counter > p->AlarmInfo.ErrorCounter )
			{
				AlarmMgr1.RegisterAlarm( &AlarmMgr1, TargetID, p->AlarmInfo.AlarmID, ALARM_TYPE_ERROR );
			}
			else if( p->Counter > p->AlarmInfo.WarningCounter )
			{
				AlarmMgr1.RegisterAlarm( &AlarmMgr1, TargetID, p->AlarmInfo.AlarmID, ALARM_TYPE_WARNING );
			}
		}
		else if( p->Counter > 0 )
		{
			p->Counter--;
		}
	}
}

void GlobalAlarmDetect_ConfigAlarmSystem( void )
{
	// Enable alarm registering mechanism when "entering" PWR_SM_POWER_ON
	if( Axis[0].PcuPowerState == PWR_SM_POWER_ON )
	{
		AlarmMgr1.State = ALARM_MGR_STATE_ENABLE;
	}

	// Disable alarm registering mechanism when "entering" PWR_SM_POWER_OFF
	if( Axis[0].PcuPowerState == PWR_SM_POWER_OFF )
	{
		AlarmMgr1.State = ALARM_MGR_STATE_DISABLE;
	}
}
