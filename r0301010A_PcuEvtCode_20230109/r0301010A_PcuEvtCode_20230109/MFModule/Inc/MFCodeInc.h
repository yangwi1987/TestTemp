/*
 * MFCodeInc.h
 *
 *  Created on: 2020年8月5日
 *      Author: Hank.Chen.CHC
 */

#ifndef MFCODEINC_H_
#define MFCODEINC_H_

#include "main.h"
#include "math.h"
#include "ADC_DRI.h"
#include "CurrentCalibration.h"
#include "CalibrationThrottle.h"
#include "VoltageCalibration.h"

typedef void ( *funtypedef_CalMaxAvgCnt )( void*, uint32_t, uint16_t );
typedef void ( *funtypedef_CalSumRoot )( void*, void*, uint16_t );
typedef void ( *funtypedef_RootMeanSquare )( void*, uint16_t );
typedef void ( *funtypedef_ReadGpioInformation )( void * );

typedef struct{
	GPIO_UNION_DEF MFGpioInfo;
	RMS_DEF CurrRms;
	uint16_t RmsCnt;
	uint16_t MaxAvgCnt;
	uint16_t MaxAvgCntCalDone;
	CURR_CALIB_DEFINE CalibCurrent;
	VOLT_CALIB_DEFINE CalibDcBusVoltage;
	THROT_CALIB_STRUCT_DEFINE CalibThrot;
	funtypedef_CalMaxAvgCnt	CalMaxAvgCnt;
	funtypedef_CalSumRoot CalSumRoot;
	funtypedef_RootMeanSquare RMS;
	funtypedef_ReadGpioInformation GpioMfinfo;
}MFStation;

void MFFunc_CalMaxAvgCnt( MFStation *p, uint32_t SpdCmd, uint16_t Authority );
void MFFunc_CalSumRoot(MFStation *p, AdcStation *v, uint16_t Authority);
void MFFunc_RootMeanSquare( MFStation *p, uint16_t Authority);
void MFFunc_GpioReadInformation( MFStation *p );

#define MF_STATION_DEFAULT { 				\
		{BITS_DEF_DEFAULT},					\
		RMS_DEF_DEFAULT,					\
		0,									\
		0,															\
		0,															\
		GURR_CALIB_DEFINE_DEFAULT, 									\
		VOLT_CALIB_DEFINE_DEFAULT,									\
		THROT_CALIB_DEFINE_DEFAULT, 								\
		(funtypedef_CalMaxAvgCnt) MFFunc_CalMaxAvgCnt,				\
		(funtypedef_CalSumRoot) MFFunc_CalSumRoot,					\
		(funtypedef_RootMeanSquare) MFFunc_RootMeanSquare,				\
		(funtypedef_ReadGpioInformation) MFFunc_GpioReadInformation,	\
}\


#endif /* MFCODEINC_H_ */
