/*
 * DataRecoder.h
 *
 *  Created on: 2024年3月4日
 *      Author: Jeff Chang
 */

#ifndef INC_DATARECORDER_H_
#define INC_DATARECORDER_H_

#include "ConstantParamAndUseFunction.h"
#include "stdint.h"

#define MAX_RECORD_CH 4
#define MAX_RECORD_NUM 3000

extern float RecordedData[MAX_RECORD_CH][MAX_RECORD_NUM];
extern uint8_t IsRecordActive;
extern void DataRecorder_Routine ( float Data0, float Data1, float Data2, float Data3 );

#endif /* INC_DATARECORDER_H_ */
