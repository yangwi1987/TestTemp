/*
 * SpeedInfo.h
 *
 *  Created on: 2022年2月8日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_SPEEDINFO_H_
#define INC_SPEEDINFO_H_
#include "stdint.h"

typedef uint16_t (*pfunSpeedInfo_Init)(void*, float);

typedef struct
{
	float MotorMechSpeedRad;
	float MotorMechSpeedRPM;
	float MotorMechSpeedRadAbs;
	float MotorMechSpeedRPMAbs;
	float ElecSpeed;
	float ElecSpeedAbs;
	float Polepair;
	float DividePolepair;
	pfunSpeedInfo_Init Init;
}SpeedInfo_t;

uint16_t SpeedInfo_Init( SpeedInfo_t* p,float Polepair );

#define SPEED_INFO_DEFAULT { \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	0.0f, \
	(pfunSpeedInfo_Init)SpeedInfo_Init \
}


#endif /* INC_SPEEDINFO_H_ */
