/*
 * Polling.h
 *
 *  Created on: 2021年6月17日
 *      Author: Fernando.Wang.HHW
 */

#ifndef INC_POLLING_H_
#define INC_POLLING_H_

#include "stdint.h"

typedef void (*pPolling_Init1Tolereance1Level)( void*, uint16_t, float, uint16_t, uint16_t, uint16_t, uint16_t );
typedef void (*pPolling_Detect1Tolereance1Level)( void*, float );


enum POLLLING_WAY_ENUM
{
	POLLING_WAY_GREATER = 0,
	POLLING_WAY_GREATER_OR_EQUAL,
	POLLING_WAY_LESS,
	POLLING_WAY_LESS_OR_EQUAL,
};

enum POLLING_RESULT_ENUM
{
	POLLING_RESULT_NONE = 0,
	POLLING_RESULT_TOLERANCE1,
};

typedef struct
{
	uint16_t Way:2;
	uint16_t Result:1;
	uint16_t Reserved:13;
	uint16_t Tolerance;
	uint16_t Add;
	uint16_t Sub;
	uint16_t Cnt;
	uint16_t MaxCnt;
	float Level;
	pPolling_Init1Tolereance1Level Init;
	pPolling_Detect1Tolereance1Level Detect;
} Polling1T1L_t;

void Polling_Init1Tolereance1Level( Polling1T1L_t *p, uint16_t Way, float Level, uint16_t Add, uint16_t Sub, uint16_t MaxCnt, uint16_t Tolerance );
void Polling_Detect1Tolereance1Level( Polling1T1L_t *p, float Value );

#define POLLING_1T1L_DEFAULT \
{	\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	(pPolling_Init1Tolereance1Level) Polling_Init1Tolereance1Level,		\
	(pPolling_Detect1Tolereance1Level) Polling_Detect1Tolereance1Level,	\
}

#endif /* INC_POLLING_H_ */
