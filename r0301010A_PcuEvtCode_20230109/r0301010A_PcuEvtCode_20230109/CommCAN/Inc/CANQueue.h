/*
 * CANQueue.h
 *
 *  Created on: 2020年3月18日
 *      Author: Will.Yang.CYY
 */

#ifndef INC_CANQUEUE_H_
#define INC_CANQUEUE_H_

#include "stdio.h"

#define ERROR_DATA_LENGTH_TOO_MANY	0xF0
#define CAN_TX_QUEUE_SIZE			8

#define	 QUEUE_OK		0x00
#define  QUEUE_FULL		0x01
#define  QUEUE_EMPTY	0x02

typedef uint8_t (*pFlushQueue)( void * );
typedef uint8_t (*pIsQueueEmpty)( void * );
typedef uint8_t (*pIsQueueFull)( void * );
typedef uint8_t (*pDeQueue)( void *, void * );
typedef uint8_t (*pEnQueue)( void *, void * );

typedef struct
{
	uint32_t	StdID_ext	:18;
	uint32_t	StdID		:11;
	uint32_t	RTR			:1;
	uint32_t	XTD			:1;
	uint32_t	IEF			:1;
} STRUCT_CAN_ID_STD;

typedef struct
{
	uint32_t	ExtID		:29;
	uint32_t	RTR			:1;
	uint32_t	XTD			:1;
	uint32_t	IEF			:1;
} STRUCT_CAN_ID_EXT;

typedef union
{
	uint32_t			All;
	STRUCT_CAN_ID_STD	StdFrame;
	STRUCT_CAN_ID_EXT	ExtFrame;
} UNION_CAN_ID;

typedef struct
{
	UNION_CAN_ID ID;
	uint8_t		Size;
	uint8_t		Reserved;
	uint8_t		Data[8];
} STRUCT_CAN_DATA;

typedef struct
{
	int8_t	Rear;
	int8_t	Front;
	uint8_t Status;
	STRUCT_CAN_DATA QData[CAN_TX_QUEUE_SIZE];
	pFlushQueue		FlushQ;
	pIsQueueEmpty 	IsQEmpty;
	pIsQueueFull 	IsQFull;
	pDeQueue		DeQ;
	pEnQueue		EnQ;
} STRUCT_CAN_QUEUE;

uint8_t CanQueue_FlushQueue(STRUCT_CAN_QUEUE * pQin );
uint8_t CanQueue_IsQueueEmpty(STRUCT_CAN_QUEUE * pQin );
uint8_t CanQueue_IsQueueFull(STRUCT_CAN_QUEUE * pQin );
uint8_t CanQueue_EnQueue( STRUCT_CAN_QUEUE *pQin, STRUCT_CAN_DATA *QIn );
uint8_t CanQueue_DeQueue( STRUCT_CAN_QUEUE *pQin, STRUCT_CAN_DATA *QOut);

#define CAN_DATA_DEFAULT { {0}, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0} }

#define QUEUE_DEFAULT 	\
{						\
	0,					\
	0,					\
	0,					\
	{						\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT,	\
		CAN_DATA_DEFAULT	\
	},												\
	(pFlushQueue)CanQueue_FlushQueue,				\
	(pIsQueueEmpty)CanQueue_IsQueueEmpty,			\
	(pIsQueueFull)CanQueue_IsQueueFull,				\
	(pDeQueue)CanQueue_DeQueue,						\
	(pEnQueue)CanQueue_EnQueue						\
}

#endif /* INC_CANQUEUE_H_ */
