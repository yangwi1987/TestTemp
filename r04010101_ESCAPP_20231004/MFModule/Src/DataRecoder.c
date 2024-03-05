/*
 * DataRecoder.c
 *
 *  Created on: 2024年3月4日
 *      Author: Jeff.Chang
 *
 *
 */

#include "DataRecoder.h"

float RecordedData[MAX_RECODE_CH][MAX_RECODE_NUM] = {0};

uint8_t IsRecordActive = UNUSE_FUNCTION;

void DataRecorder_Routine ( float Data0, float Data1, float Data2, float Data3 )
{
    if ( IsRecordActive == USE_FUNCTION )
    {
    	static uint16_t i = 0;
    	RecordedData[0][i] = Data0;
    	RecordedData[1][i] = Data1;
    	RecordedData[2][i] = Data2;
    	RecordedData[3][i] = Data3;
    	if ( i == ( MAX_RECODE_NUM - 1 ))
    	{
    		IsRecordActive = UNUSE_FUNCTION;
    	}
    	else
    	{
    		i++;
    	}
    }
}


