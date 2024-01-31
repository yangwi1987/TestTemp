/*
 * ThrotMap.h
 *
 *  Created on: 2021年1月6日
 *      Author: Hank.Chen.CHC
 */

#ifndef INC_THROTMAP_H_
#define INC_THROTMAP_H_

#include "stdio.h"
#include "SystemTableLinker.h"
#include "SmoothCurveChange.h"

#define DIMENSION_ITEM	 	3
#define OTHRER_ITEM			1
#define MAX_TN_CNT			5
#define THROT_MAXIMUM		(float)1.0f
#define THROT_MINIMUM		(float)0.0f
#define INTERVAL_WIDTH 		(1.0 / THROTTLE_INTERVALS)
#define INV_INTERVAL_WIDTH	(1.0 / INTERVAL_WIDTH)
#define THROTTLE_HANDLE_TIMEBASE 0.001f

typedef void (*functypeThrottleMapping_init)( void *, void * );
typedef void (*functypeThrottleMapping_calc)( void * );
typedef void (*functypeThrottleMapping_ramp)( void * );
typedef void (*functypeThrottleMapping_reset)( void * );

typedef struct {
	float EmptyThrottle;
	float HalfThrottle;
	float FullThrottle;
	float ThrotRiseRamp;
	float ThrotFallRamp;
} THROT_TABLE_BUF;

#define THROT_TABLE_BUF_DEFAULT {	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,   \
	0.0f }

typedef struct {
	uint16_t InitError;
	uint16_t ThrottleDI;
	uint8_t TnSelect;
	uint8_t TnSelectDelay;
	uint8_t TnSelectEmpty;
	uint8_t TnSelectHalf;
	uint8_t TnSelectFull;
	uint8_t Reserved0;
	uint16_t ThrottleReleaseFlag;
	float ThrottleRawIn;
	float ThrottleIn;
	float PercentageTarget;
	float PercentageOut;
	float EmptyThrottle;
	float HalfThrottle;
	float FullThrottle;
	float ChangeRamp;
	float ChangeTime;
	float EmptyThrottleRamp;
	float HalfThrottleRamp;
	float FullThrottleRamp;
	THROT_TABLE_BUF ThrottleTnTab[ MAX_TN_CNT ];
	functypeThrottleMapping_init Init;
	functypeThrottleMapping_calc Calc;
	functypeThrottleMapping_ramp Ramp;
	functypeThrottleMapping_reset Reset;
} ThrottleMapping_t;

void ThrottleMapping_Init( ThrottleMapping_t *v, DriveParams_t *a );
void ThrottleMapping_Calc( ThrottleMapping_t *v );
void ThrottleMapping_Ramp( ThrottleMapping_t *v );
void ThrottleMapping_Reset( ThrottleMapping_t *v );

#define THROTTLE_CALIB_DEFAULT { \
	0, \
	1, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	{THROT_TABLE_BUF_DEFAULT, THROT_TABLE_BUF_DEFAULT, THROT_TABLE_BUF_DEFAULT, THROT_TABLE_BUF_DEFAULT, THROT_TABLE_BUF_DEFAULT}, \
	(functypeThrottleMapping_init)ThrottleMapping_Init, \
	(functypeThrottleMapping_calc)ThrottleMapping_Calc, \
	(functypeThrottleMapping_ramp)ThrottleMapping_Ramp, \
	(functypeThrottleMapping_reset)ThrottleMapping_Reset }


#endif /* INC_THROTMAP_H_ */
