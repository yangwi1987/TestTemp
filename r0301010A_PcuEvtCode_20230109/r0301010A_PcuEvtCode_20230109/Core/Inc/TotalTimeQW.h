/*
 * TotalTimeQW.h
 *
 *  Created on: Nov 15, 2022
 *      Author: Kevin.Kuo
 */

#ifndef INC_TOTALTIMEQW_H_
#define INC_TOTALTIMEQW_H_

#include "stdint.h"

// total time Qword data structure
typedef struct
{
	uint32_t TotalOPTime3Sec;		// 4 Bytes, total operation time in 3 second
	uint16_t ThisOPTime3Sec;		// 2 Bytes, this operation time in 3 second
	uint8_t CheckWord;				// 1 Byte
	uint8_t CheckSum;				// 1 Byte
} TotalTimeQW_t;

#define TOTAL_TIME_QW_DEFAULT { \
	0, /* TotalOPTimeMinu */	\
	0, /* ThisOPTimeMinu */ 	\
	0, /* CheckWord */			\
	0 /* CheckSum */ 			}

#endif /* INC_TOTALTIMEQW_H_ */
