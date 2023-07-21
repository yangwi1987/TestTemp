/*
 * AlarmMgr.c
 *
 *  Created on: 2019年12月9日
 *      Author: MikeSFWen
 */
#include "AlarmMgr.h"

AlarmStack_t AlarmStack[MAX_AXIS_NUM] = {ALARM_STACK_DEFAULT, ALARM_STACK_DEFAULT};
extern AlarmMgr_t AlarmMgr1;

#define ALARM_FLAG_POS_MACRO( AlarmID, Array, Bit ) \
		Array = AlarmID >> 4; \
		Bit = AlarmID & 0x000F; \

static uint16_t NonCriAlarmBitArray = 0; // Indicate the NonCriAlarm index of alarmStack

static void AlarmStack_FlagSet( AlarmStack_t *p, uint16_t AlarmID )
{
	uint16_t Array = 0;
	uint16_t Bit = 0;
	ALARM_FLAG_POS_MACRO( AlarmID, Array, Bit )
	p->AlarmFlag[Array] |= (((uint16_t)1)<<Bit);
}

static void AlarmStack_FlagUnset( AlarmStack_t *p, uint16_t AlarmID )
{
	uint16_t Array = 0;
	uint16_t Bit = 0;
	ALARM_FLAG_POS_MACRO( AlarmID, Array, Bit )
	p->AlarmFlag[Array] &= ~((uint16_t)((0x0001)<<Bit));
}

uint16_t AlarmStack_FlagRead( AlarmStack_t *p, uint16_t AlarmID )
{
	uint16_t Array = 0;
	uint16_t Bit = 0;
	uint16_t Read = 0;
	uint16_t Flag = 0;
	ALARM_FLAG_POS_MACRO( AlarmID, Array, Bit )
	Read = p->AlarmFlag[Array] & (((uint16_t)1)<<Bit);
	Flag = ( Read == 0 ) ? 0 : 1 ;
	return Flag;
}

void UpdateAlarmStack( uint16_t AxisIndex, uint16_t AlarmID )
{
	AlarmStack_t *pAlarmStack = &AlarmStack[AxisIndex];
	int i;

	// This Alarm had been registered
	for( i = 0; i < MAX_NOW_ALARM_SIZE; i++ )
	{
		if( pAlarmStack->NowAlarmID[i] == AlarmID ) return;
	}

	// log alarm ID onto the alarm stack
	if( pAlarmStack->TopIndicator >= MAX_NOW_ALARM_SIZE )
	{
		return;
	}
	else
	{
		pAlarmStack->NowAlarmID[pAlarmStack->TopIndicator] = AlarmID;
		if(SystemTable.AlarmTableInfo[AlarmID].AlarmType == ALARM_TYPE_NONCRITICAL)
		{
			NonCriAlarmBitArray |= 1 << pAlarmStack->TopIndicator;
		}
		pAlarmStack->TopIndicator++;
		AlarmStack_FlagSet( pAlarmStack, AlarmID );
	}
}

void RegisterAlarm( AlarmMgr_t *v, uint16_t TargetID, uint16_t AlarmID, uint16_t AlarmType )
{
	if( AlarmMgr1.State == ALARM_MGR_STATE_DISABLE)
		return;

	if( TargetID > MAX_AXIS_NUM || TargetID < 0 )
		return;

	if( AlarmID > MAX_ALARM_NUM )
		return;

	int i;

	if( TargetID == TARGET_ID_GLOBAL )
	{
		for( i = 0; i < MAX_AXIS_NUM; i++ )
		{
			// On Alarm flag for state machine
			switch( AlarmType ) //TODO replace "AlarmType" with "SystemTable.AlarmTableInfo[AlarmID].AlarmType", and remove argument "uint16_t AlarmType"
			{
				case ALARM_TYPE_WARNING:
					*v->pHasWarning[i] = 1;
					break;

				case ALARM_TYPE_NONCRITICAL:
					*v->pHasNonCriAlarm[i] = 1;
					break;

				case ALARM_TYPE_CRITICAL:
					*v->pHasCriAlarm[i] = 1;
					break;

//				case ALARM_TYPE_CRITICAL:
//				v->CriticalAlarmFlag[i] = 1;
//				break;

				case ALARM_TYPE_NONE:
				default:
					break;
			}
			UpdateAlarmStack( i, AlarmID );
		}
	}
	else
	{
		int TargetIndex = TargetID - 1;
		// On Alarm flag for state machine
		switch( AlarmType )
		{
			case ALARM_TYPE_WARNING:
				*v->pHasWarning[TargetIndex] = 1;
				break;

			case ALARM_TYPE_NONCRITICAL:
				*v->pHasNonCriAlarm[TargetIndex] = 1;
				break;

			case ALARM_TYPE_CRITICAL:
				*v->pHasCriAlarm[TargetIndex] = 1;
				break;

			case ALARM_TYPE_NONE:
			default:
				break;
		}
		UpdateAlarmStack( TargetIndex, AlarmID );
	}
}

static void RegisterAxisAlarmStatic( uint16_t AxisID, uint16_t AlarmID, uint16_t AlarmType )
{
	if( AlarmMgr1.State == ALARM_MGR_STATE_DISABLE)
		return;

	if( AxisID > MAX_AXIS_NUM || AxisID < 0 )
		return;

	if( AlarmID > MAX_ALARM_NUM )
		return;

	{
		int AxisIndex = AxisID - 1;
		switch( AlarmType ) //TODO replace "AlarmType" with "SystemTable.AlarmTableInfo[AlarmID].AlarmType", and remove argument "uint16_t AlarmType"
		{

		case ALARM_TYPE_WARNING:
			*AlarmMgr1.pHasWarning[AxisIndex] = 1;
			break;

		case ALARM_TYPE_NONCRITICAL:
			*AlarmMgr1.pHasNonCriAlarm[AxisIndex] = 1;
			break;

		case ALARM_TYPE_CRITICAL:
			*AlarmMgr1.pHasCriAlarm[AxisIndex] = 1;
			break;

		case ALARM_TYPE_NONE:
			default:
				break;
		}
		UpdateAlarmStack( AxisIndex, AlarmID );
	}
}

void RegisterAxisAlarm( AlarmDetect_t *v, uint16_t AlarmID, uint16_t AlarmType )
{
	RegisterAxisAlarmStatic( v->AxisID, AlarmID, AlarmType );
}

uint16_t removeZeros(uint16_t arr[], int LengthOfArray)
{
	uint16_t TopIndex = 0;
	uint16_t i;

	// remove zeros and filled by the later numbers
    for ( i = 0; i < LengthOfArray; i++ )
    {
        if (arr[i] != 0)
        {
        	arr[TopIndex] = arr[i];
        	TopIndex++;
        }
    }

    // clear the rest of numbers
    for( i = TopIndex; i < LengthOfArray; i++ )
    {
    	arr[i] = 0;
    }
    return TopIndex;
}

void ResetAllAlarm( AlarmMgr_t *v )
{
	// Only if PCU stop register alarm, then PCU can reset alarm.
	if( AlarmMgr1.State == ALARM_MGR_STATE_ENABLE)
		return;

	AlarmStack_t *pAlarmStack;
	int i, j;

	for( j = 0; j < MAX_AXIS_NUM; j++ )
	{
		pAlarmStack = &AlarmStack[j];
		for( i = 0; i < MAX_NOW_ALARM_SIZE; i++ )
		{
			pAlarmStack->NowAlarmID[i] = 0;
			pAlarmStack->TopIndicator = 0;
		}

		*v->pHasWarning[j] = 0;
		*v->pHasNonCriAlarm[j] = 0;
		*v->pHasCriAlarm[j] = 0;
	}
}

// old function definition before 3.1.1.11, not necessary now
void ResetAllWarning( AlarmMgr_t *v )
{
	/*
	AlarmStack_t *pAlarmStack;
	int i, j;
	//v->State = ALARM_MGR_STATE_DISABLE; // can not register alarm while reseting?

	for( j = 0; j < MAX_AXIS_NUM; j++ )
	{
		pAlarmStack = &AlarmStack[j];
		for( i = 0; i < MAX_NOW_ALARM_SIZE; i++ )
		{
			uint16_t TempBit = 0x01<<i;
			if ( (WarningBitArray & TempBit) != 0)//pAlarmStack->NowAlarmID[i] is warning
			{
				AlarmStack_FlagUnset( pAlarmStack, pAlarmStack->NowAlarmID[i]);
				pAlarmStack->NowAlarmID[i] = 0;
			}
			else //pAlarmStack->NowAlarmID[i] is alarm
			{
			}
		}

		// todo sorting alarm to bottom, fill the warning void.
		pAlarmStack->TopIndicator = removeZeros(pAlarmStack->NowAlarmID, MAX_NOW_ALARM_SIZE);
		WarningBitArray = (uint16_t)0x0000;
		*v->pHasWarning[j] = 0;
	}

	//v->State = ALARM_MGR_STATE_ENABLE;
	*/
}

void ResetAllNonCriticalAlarm( AlarmMgr_t *v )
{
	/*
	AlarmStack_t *pAlarmStack;
	int i, j;
	//v->State = ALARM_MGR_STATE_DISABLE; // can not register alarm while reseting?

	for( j = 0; j < MAX_AXIS_NUM; j++ )
	{
		pAlarmStack = &AlarmStack[j];
		for( i = 0; i < MAX_NOW_ALARM_SIZE; i++ )
		{
			uint16_t TempBit = 0x01<<i;
			if ( (NonCriAlarmBitArray & TempBit) != 0)//pAlarmStack->NowAlarmID[i] is non critical Alarm
			{
				AlarmStack_FlagUnset( pAlarmStack, pAlarmStack->NowAlarmID[i]);
				pAlarmStack->NowAlarmID[i] = 0;
			}
			else //pAlarmStack->NowAlarmID[i] is critical alarm
			{
			}
		}

		// todo sorting alarm to bottom, fill the NonCriAlarm void.
		pAlarmStack->TopIndicator = removeZeros(pAlarmStack->NowAlarmID, MAX_NOW_ALARM_SIZE);
		NonCriAlarmBitArray = (uint16_t)0x0000;
		*v->pHasNonCriAlarm[j] = 0;
	}

	//v->State = ALARM_MGR_STATE_ENABLE;
	*/
}
