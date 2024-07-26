/*
 * FourQuadTable.h
 *
 *  Created on: 2020年3月6日
 *      Author: MikeSFWen
 */

#ifndef INC_FOURQUADTABLE_H_
#define INC_FOURQUADTABLE_H_

#include "Table_Type_Define.h"

#define THROTTLE_TABLE_LENGTH			3

#define CURVE_PARA_VALUE_SHIFT	32768.0f
#define DRIVE_CURVE_ID_START	25

enum DRIVE_CURVE_TABLE_ENUM
{
	DRIVE_TABLE_LIMP = 0,
	DRIVE_TABLE_REVERSE,
	DRIVE_TABLE_BAMBINI_NORMAL,
	DRIVE_TABLE_BAMBINI_BOOST,
	DRIVE_TABLE_MICRO_NORMAL,
	DRIVE_TABLE_MICRO_BOOST,
	DRIVE_TABLE_MINI_NORMAL,
	DRIVE_TABLE_MINI_BOOST,
	DRIVE_TABLE_PERFORMANCE_NORMAL,
	DRIVE_TABLE_PERFORMANCE_BOOST,
	DRIVE_TABLE_LENGTH,
};

enum DRIVE_CURVE_TABLE_NOW_ENUM
{
	DRIVE_TABLE_LIMP_NOW = 0,
	DRIVE_TABLE_REVERSE_NOW,
	DRIVE_TABLE_NORMAL_NOW,
	DRIVE_TABLE_BOOST_NOW,
	DRIVE_TABLE_LENGTH_NOW,
};

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



#endif /* INC_FOURQUADTABLE_H_ */
