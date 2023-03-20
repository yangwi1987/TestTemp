/*
 * TotalTime.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Kevin.Kuo
 */

#include "TotalTime.h"

void TotalTime_Init( TotalTime_t* v )
{
	v->RWState = TOTAL_TIME_INITAL;

	// assign buffer total time QW from External flash.
	ExtFlash_ReadLastOPTotalTime( v->pExtFlash ); // to do expand
//	v->LocalTotalTimeQW.TotalOPTimeMinu = v->BufferTotalTimeQW.TotalOPTimeMinu;
//	v->LocalTotalTimeQW.ThisOPTimeMinu = 0;
//	v->LocalTotalTimeQW.CheckWord = 0;
//	v->LocalTotalTimeQW.CheckSum = 0;

	if( v->pExtFlash->AlarmStatus & FLASHERROR_CHECKSUM_FAIL_TOTAL_TIME )
	{
		v->LocalTotalTime = 0;
		v->LocalThisTime = 0;
		v->RWState = TOTAL_TIME_ERROR;
	}
	else
	{
		v->LocalTotalTime = v->BufferTotalTimeQW.TotalOPTime3Sec;
		v->LocalThisTime = 0;
		v->RWState = TOTAL_TIME_READY;
	}

}

void TotalTime_Do30secLoop( TotalTime_t* v )
{
	v->LocalThisTime += 10;
	v->LocalTotalTime += 10;
}


void TotalTime_Do3secLoop( TotalTime_t* v )
{
	v->LocalThisTime++;
	v->LocalTotalTime++;
}

void TotalTime_RecTotalTime( TotalTime_t* v )
{
	if( v->RWState == TOTAL_TIME_READY )
	{
		v->RWState = TOTAL_TIME_WRITE;
//		v->BufferTotalTimeQW = v->LocalTotalTimeQW;
		v->BufferTotalTimeQW.ThisOPTime3Sec = v->LocalThisTime;
		v->BufferTotalTimeQW.TotalOPTime3Sec = v->LocalTotalTime;
		ExtFlash_LogTotalTime( v->pExtFlash );
		v->RWState = TOTAL_TIME_READY;
	}
	else
	{
		// if RWState is TOTAL_TIME_ERROR, never write data to flash
		// if RWState is TOTAL_TIME_INITAL, skip request this time and then write data after initial finished.
		// no condition will cause RWState is TOTAL_TIME_WRITE.
	}
}
