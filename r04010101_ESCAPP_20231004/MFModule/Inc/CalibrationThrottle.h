/*
 * ThrottleCalib.h
 *
 *  Created on: 2020年12月15日
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_CALIBRATIONTHROTTLE_H_
#define INC_CALIBRATIONTHROTTLE_H_

#include "ADC_DRI.h"
#include "MF_CONST_AND_STRUCT_DEF.h"


typedef void ( *functypeThrottleCalib_RMAA )( void *, uint16_t, uint16_t, uint16_t );
typedef void ( *functypeThrottleCalib_RAVOT )( void *, void *, uint32_t * );
typedef void ( *functypeThrottleCalib_CTG )( void *, uint32_t * );

typedef struct{
	uint16_t Authority;
	uint16_t CtrlMode;
	uint32_t AccumulatorADC;
	uint16_t AccumulatorCnt;
	uint16_t ThrottleAdValue[2];
	GAIN_TYPE_DEFINE CalibrationGain;
	functypeThrottleCalib_RMAA RMAA;
	functypeThrottleCalib_RAVOT RAVOT;
	functypeThrottleCalib_CTG CTG;
}THROT_CALIB_STRUCT_DEFINE;

void ReadModeAndAuthority( THROT_CALIB_STRUCT_DEFINE *v, uint16_t Mode, uint16_t Auth, uint16_t ServoOnSate );
void ReadAdcValueOfThrottle( THROT_CALIB_STRUCT_DEFINE *v, AdcStation *a, uint32_t *Enable );
void CalcThrottleGain( THROT_CALIB_STRUCT_DEFINE *v, uint32_t *Enable );

#define THROT_CALIB_DEFINE_DEFAULT { 	\
	0,	\
	0,	\
	0,	\
	0,	\
	{0, 0},	\
	UNION_TYPE_DEFINE_DEFAULT, \
	( functypeThrottleCalib_RMAA )ReadModeAndAuthority,\
	( functypeThrottleCalib_RAVOT )ReadAdcValueOfThrottle, \
	( functypeThrottleCalib_CTG )CalcThrottleGain \
}\

#endif /* INC_CALIBRATIONTHROTTLE_H_ */
