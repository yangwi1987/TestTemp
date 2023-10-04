/*
 * SixWave.h
 *
 *  Created on: 2020年4月15日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_SIXWAVE_H_
#define INC_SIXWAVE_H_

#include "stdint.h"
#include "Pid.h"

typedef void (*pfunSixWaveHall_Degree)( void*, uint16_t );
typedef void (*pfunSixWaveCtrl120_CurrFb)( void*, uint16_t, float, float, float );
typedef void (*pfunSixWaveCtrl120_Clean)( void* );
typedef uint16_t (*pfunSixWaveCtrl120_Init)( void* , float, float, float );

enum SIX_WAVE_HALL_DIRECTION_ENUM
{
	SIX_WAVE_HALL_DIRECTION_ELEC_POSITIVE = 0,
	SIX_WAVE_HALL_DIRECTION_ELEC_NEGATIVE,
};

enum SIX_WAVE_HALL_TYPE_ENUM
{
	SIX_WAVE_HALL_TYPE_REAL,
	SIX_WAVE_HALL_TYPE_TEST,
};

enum SIX_WAVE_TEST_HALL_OUT_ENUM
{
	TEST_HALL_OUT_VHWL = 1,
	TEST_HALL_OUT_WHUL,
	TEST_HALL_OUT_VHUL,
	TEST_HALL_OUT_UHVL,
	TEST_HALL_OUT_UHWL,
	TEST_HALL_OUT_WHVL,
};

typedef struct
{
	float CurrFb;
	float CurrCmd;
	PI_TYPE Regulator;
	pfunSixWaveCtrl120_CurrFb GetCurrFb;
	pfunSixWaveCtrl120_Clean Clean;
	pfunSixWaveCtrl120_Init Init;
}SIX_WAVE_120_CURRENT_CONTROL_TYPE;

typedef struct
{
	float HallDegree;
	int16_t HallSection;
	uint16_t HallPrevious : 3;
	uint16_t Direction : 1;
	uint16_t HallType : 1;
	uint16_t InitFinish : 1;
	uint16_t SixWaveType : 1;
	uint16_t Reversed : 9;
	pfunSixWaveHall_Degree CalHallAngle;
}SIX_WAVE_HALL_TYPE;

void SixWaveHall_Degree( SIX_WAVE_HALL_TYPE *p, uint16_t Hall );
void SixWaveCtrl120_CurrFb( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p, uint16_t Section, float Iu, float Iv, float Iw );
void SixWaveCtrl120_Clean( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p);
uint16_t SixWaveCtrl120_Init( SIX_WAVE_120_CURRENT_CONTROL_TYPE *p, float kp, float Ki, float Period);

#define SIX_WAVE_CURRENT_CONTROL_DEFAULT	\
{				\
	0.0f,		\
	0.0f, 		\
	PI_DEFAULT,	\
	(pfunSixWaveCtrl120_CurrFb) SixWaveCtrl120_CurrFb,	\
	(pfunSixWaveCtrl120_Clean) SixWaveCtrl120_Clean,	\
	(pfunSixWaveCtrl120_Init) SixWaveCtrl120_Init,		\
}


#define SIX_WAVE_HALL_DEFAULT	\
{			\
	0.0f,	\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	(pfunSixWaveHall_Degree) SixWaveHall_Degree,	\
}

#endif /* INC_SIXWAVE_H_ */
