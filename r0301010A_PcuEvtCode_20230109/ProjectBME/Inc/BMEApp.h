/*
 * BMEApp.h
 *
 *  Created on: 2023年7月14日
 *      Author: egghe
 */

#ifndef INC_BMEAPP_H_
#define INC_BMEAPP_H_

#include "stdio.h"
#include "string.h"

/*==== AC Power info Start ====*/
#define INSTANT_POWER_LOG_SIZE 10
#define AVERAGE_POWER_CALCULATE_PERIOD_IN_SEC	0.1		/* 100 ms */
#define INSTANT_POWER_UPDATE_PERIOD_IN_SEC		0.01 	/* 10 ms  = INSTANT_POWER_CALCULATE_PERIOD_IN_SEC / INSTANT_POWER_LOG_SIZE */

typedef void (*functypeBMEApp_AcPowerInfo_100HzLoop)(void *, float);
typedef void (*functypeBMEApp_AcPowerInfo_Reset)(void *); 

typedef struct
{
	uint8_t LogCnt;
	uint32_t TotalLogCnt;		/* total oeperation time =  TotalLogCnt * 0.1 sec */
	float TotalJoule;		
	float InstPower;
	float AvgPower;
	float InstPwrSum;
	functypeBMEApp_AcPowerInfo_Reset Reset;
	functypeBMEApp_AcPowerInfo_100HzLoop _100HzLoop;
} AcPwrInfo_t;

#define AC_POWER_INFO_DEFAULT \
{							\
	0,		/*LogCnt*/ 			\
	0,		/*TotalLogCnt*/		\
	0.0f,	/*TotalJoule*/		\
	0.0f,	/*InstPower*/		\
	0.0f,	/*AvgPower*/		\
	0.0f,	/*InstPwrSum*/		\
	(functypeBMEApp_AcPowerInfo_Reset)AcPowerInfo_Reset,		\
	(functypeBMEApp_AcPowerInfo_100HzLoop)AcPowerInfo_100HzLoop,	\
}							\

extern AcPwrInfo_t AcPwrInfo;


/*==== AC Power info End ====*/


#endif /* INC_BMEAPP_H_ */
