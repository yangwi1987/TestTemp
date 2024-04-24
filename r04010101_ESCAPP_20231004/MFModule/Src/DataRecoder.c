/*
 * DataRecoder.c
 *
 *  Created on: 2024年3月4日
 *      Author: Jeff.Chang
 *
 *
 */

#include <DataRecorder.h>
//#define FN_DREBUG_RECORD_FLAG		998
//#define PARAM_FN_REGS_SIZE		100
extern uint32_t DriveFnRegs[100];

float RecordedData[MAX_RECORD_CH][MAX_RECORD_NUM] = {0};

uint8_t IsRecordActive = UNUSE_FUNCTION;
uint16_t DataRECORDERCnt0 = 0;
uint16_t DataRECORDERCnt1 = 0;
uint16_t DataRECORDERCnt2 = 0;
uint16_t DataRECORDERCnt3 = 0;

void DataRecorder_Routine ( float Data0, float Data1, float Data2, float Data3 )
{
    if ( (IsRecordActive == USE_FUNCTION) || ( DriveFnRegs[98] == 1) )
    {
    	static uint16_t i = 0;
    	RecordedData[0][i] = Data0;
    	RecordedData[1][i] = Data1;
    	RecordedData[2][i] = Data2;
    	RecordedData[3][i] = Data3;
    	if ( i == ( MAX_RECORD_NUM - 1 ))
    	{
    		i = 0;
    		IsRecordActive = UNUSE_FUNCTION;
//    		DriveFnRegs[98] = 0;
    	}
    	else
    	{
    		i++;
    	}
    }
}


