/*
 * AlarmMgr.h
 *
 *  Created on: 2019年12月9日
 *      Author: MikeSFWen
 */

#ifndef INC_ALARMMGR_H_
#define INC_ALARMMGR_H_

#include "main.h"
#include "AlarmTable.h"
#include "ParamTable.h"
#include "AlarmDetect.h"

#define ALARM_STACK_SIZE_BYTE_PER_AXIS	(MAX_NOW_ALARM_SIZE * sizeof(uint16_t) / sizeof(uint8_t))
#define ALARM_STACK_SIZE_BYTE			(MAX_AXIS_NUM * ALARM_STACK_SIZE_BYTE_PER_AXIS)
#define ALARM_FLAG_ARRAY_SIZE(Num)  ( ( Num - ( Num >> 4 ) * 16 ) == 0 ) ? ( Num >> 4 ) : ( ( Num >> 4 ) + 1 )

typedef enum
{
	ALARM_MGR_STATE_DISABLE = 0,
	ALARM_MGR_STATE_ENABLE = 1,
} E_ALARM_MGR_STATE;

typedef void (*functypeAlarmMgr_RegisterAlarm)(void*, uint16_t, uint16_t, uint16_t);
typedef void (*functypeAlarmMgr_ResetAllAlarm)(void*);
typedef void (*functypeAlarmMgr_ResetAllWarning)(void*);
typedef uint16_t (*pAlarmStack_FlagRead)( void*, uint16_t );

typedef struct {
	uint8_t State;
	uint16_t *pHasWarning[MAX_AXIS_NUM];
	uint16_t *pHasNonCriAlarm[MAX_AXIS_NUM];
	uint16_t *pHasCriAlarm[MAX_AXIS_NUM];
	functypeAlarmMgr_RegisterAlarm RegisterAlarm;
	functypeAlarmMgr_ResetAllAlarm ResetAllAlarm;
} AlarmMgr_t;

typedef struct {
	uint16_t TopIndicator;
	uint16_t NowAlarmID[MAX_NOW_ALARM_SIZE];
	uint16_t HistoryAlarmID[MAX_HISTORY_ALARM_SIZE];
	uint16_t AlarmFlag[ALARM_FLAG_ARRAY_SIZE(MAX_ALARM_NUM)];
	pAlarmStack_FlagRead FlagRead;
} AlarmStack_t;

void RegisterAlarm( AlarmMgr_t *v, uint16_t TargetID, uint16_t AlarmID, uint16_t AlarmType );
void ResetAllAlarm( AlarmMgr_t *v );
void ResetAllWarning( AlarmMgr_t *v );
void ResetAllNonCriticalAlarm( AlarmMgr_t *v );
void RegisterAxisAlarm( AlarmDetect_t *v, uint16_t AlarmID, uint16_t AlarmType );
uint16_t AlarmStack_FlagRead( AlarmStack_t *p, uint16_t AlarmID );

#define ALARM_STACK_DEFAULT { \
	0, \
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, \
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, \
	{0}, \
	(pAlarmStack_FlagRead)AlarmStack_FlagRead, \
	}

#define ALARM_MGR_DEFAULT { \
	ALARM_MGR_STATE_ENABLE, /* State */\
	{0, 0}, /* pHasWarning */ \
	{0, 0}, /* pHasNonCriAlarm */ \
	{0, 0}, /* pHasCriAlarm */ \
	(functypeAlarmMgr_RegisterAlarm)RegisterAlarm, \
	(functypeAlarmMgr_ResetAllAlarm)ResetAllAlarm}

extern AlarmStack_t AlarmStack[MAX_AXIS_NUM];

#endif /* INC_ALARMMGR_H_ */
