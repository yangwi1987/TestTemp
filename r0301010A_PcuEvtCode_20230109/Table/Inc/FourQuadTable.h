/*
 * FourQuadTable.h
 *
 *  Created on: 2020年3月6日
 *      Author: MikeSFWen
 */

#ifndef INC_FOURQUADTABLE_H_
#define INC_FOURQUADTABLE_H_

#include "Table_Type_Define.h"

#define DRIVE_TABLE_LENGTH				5
#define REGEN_TABLE_LENGTH				5
#define THROTTLE_TABLE_LENGTH			3

#define CURVE_PARA_VALUE_SHIFT	32768.0f
#define REGEN_CURVE_ID_START	20
#define DRIVE_CURVE_ID_START	55


enum DRIVE_CURVE_PARAM_ENUM
{
	DRIVE_PROPULSION_START = 0,
	DRIVE_PROPULSION_MAX,
	DRIVE_POWER_MAX,
	DRIVE_SPEED_MAX,
	DRIVE_SLOPE_START,
	DRIVE_SLOPE_END,
	DRIVE_ALL_PARA_NUM
};

typedef struct
{
	float ThrottleCalib[5];
} THROTTLE_TABLE_TYPE;

typedef struct
{
	float Para[DRIVE_ALL_PARA_NUM];
} DRIVE_TABLE_TYPE;

typedef struct
{
	float PropulsionMax;	// N
	float SpeedMax;			// kph
} BACK_ROLL_TABLE_TYPE;

#define DRIVE_TABLE_DEFAULT { \
	{0.0f}, \
	}

#define BACK_ROLL_TABLE_DEFAULT { \
	0.0f, \
	0.0f }


#endif /* INC_FOURQUADTABLE_H_ */
