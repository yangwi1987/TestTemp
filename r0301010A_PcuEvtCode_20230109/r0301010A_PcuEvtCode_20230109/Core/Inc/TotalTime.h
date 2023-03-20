/*
 * TotalTime.h
 *
 *  Created on: Nov 15, 2022
 *      Author: Kevin.Kuo
 */

#ifndef INC_TOTALTIME_H_
#define INC_TOTALTIME_H_

#include "TotalTimeQW.h"
#include "ExtFlash.h"

typedef enum
{
	TOTAL_TIME_NULL = 0,
	TOTAL_TIME_INITAL = 1, // initial is reading from external flash.
	TOTAL_TIME_READY = 2,
	TOTAL_TIME_WRITE = 3,
	TOTAL_TIME_ERROR = 4,
} E_TOTAL_TIME_RW_STATE;

typedef void (*functypeTotalTime_Init)( void* );
typedef void (*functypeTotalTime_Do3secLoop)( void* );
typedef void (*functypeTotalTime_Do30secLoop)( void* );
typedef void (*functypeTotalTime_RecTotalTime)( void* );

typedef struct
{
	ExtFlash_t *pExtFlash;
	uint32_t LocalTotalTime; // unit in 3 sec
	uint16_t LocalThisTime; // unit in 3 sec
	int16_t BufferServoOnState;
	int16_t NowServoOnState;
	TotalTimeQW_t BufferTotalTimeQW;	// total time buffer read/write from external flash.
	uint32_t RWState;
	functypeTotalTime_Init Init;
	functypeTotalTime_Do3secLoop Do3secLoop;
	functypeTotalTime_Do30secLoop Do30secLoop;
	functypeTotalTime_RecTotalTime RecTotalTime;
} TotalTime_t;

void TotalTime_Init( TotalTime_t* );
void TotalTime_Do3secLoop( TotalTime_t* );
void TotalTime_Do30secLoop( TotalTime_t* );
void TotalTime_RecTotalTime( TotalTime_t* );


//	TOTAL_TIME_QW_DEFAULT, 			LocalTotalTimeQW */

#define TOTAL_TIME_DEFAULT { \
	0, 								/* pExtFlash* */ \
	0, 			/* LocalTotalTime */ \
	0, 			/* LocalThisTime */ \
	0,			/* BufferServoOnState; */ \
	0,			/* NowServoOnState; */ \
	TOTAL_TIME_QW_DEFAULT, 			/* BufferTotalTimeQW */ \
	TOTAL_TIME_NULL,				/* RWState */ \
	(functypeTotalTime_Init) TotalTime_Init,\
	(functypeTotalTime_Do30secLoop) TotalTime_Do3secLoop,\
	(functypeTotalTime_Do30secLoop) TotalTime_Do30secLoop,\
	(functypeTotalTime_RecTotalTime) TotalTime_RecTotalTime }

#endif /* INC_TOTALTIME_H_ */
