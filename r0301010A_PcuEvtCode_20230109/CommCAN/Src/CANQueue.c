/*
 * CANQueue.c
 *
 *  Created on: 2020年3月18日
 *      Author: Will.Yang.CYY
 */

#include "CANQueue.h"

uint8_t CanQueue_FlushQueue( STRUCT_CAN_QUEUE *pQin )
{
	uint8_t i, j;

	pQin->Front = 0;
	pQin->Rear = 0;

	// Clear Queue Data
	for( i = 0; i < CAN_TX_QUEUE_SIZE; i++ )
	{
		pQin->QData[i].ID.All = 0;
		pQin->QData[i].Size = 0;
		pQin->QData[i].Reserved = 0;
		for( j = 0; j < 8; j++ )
		{
			pQin->QData[i].Data[j] = 0;
		}
	}

	// Clear Buffer

	return QUEUE_EMPTY;
}

uint8_t CanQueue_EnQueue( STRUCT_CAN_QUEUE *pQin, STRUCT_CAN_DATA *QIn )
{
	int8_t	RearTemp = pQin->Rear;

	if( pQin->IsQFull(pQin) == 0 )
	{
		pQin->Rear = (pQin->Rear + 1) % CAN_TX_QUEUE_SIZE;
		pQin->QData[RearTemp] = *QIn;
		return QUEUE_OK;
	}
	else
	{
		return QUEUE_FULL;
	}
}

uint8_t CanQueue_DeQueue( STRUCT_CAN_QUEUE *pQin, STRUCT_CAN_DATA *QOut)
{
	int8_t	FrontTemp=pQin->Front;

	if( pQin->IsQEmpty(pQin) == 0 )
	{
		pQin->Front = ( pQin->Front + 1 ) % CAN_TX_QUEUE_SIZE;
		*QOut = pQin->QData[FrontTemp];
		return QUEUE_OK;
	}
	else
	{
		return QUEUE_EMPTY;
	}
}

uint8_t CanQueue_IsQueueEmpty( STRUCT_CAN_QUEUE *pQin )
{
	if( pQin->Front == pQin->Rear )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t CanQueue_IsQueueFull( STRUCT_CAN_QUEUE *pQin )
{
	int8_t Pos=0;
	Pos = ( pQin->Rear + 1 ) % CAN_TX_QUEUE_SIZE;

	if( pQin->Front == Pos )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
