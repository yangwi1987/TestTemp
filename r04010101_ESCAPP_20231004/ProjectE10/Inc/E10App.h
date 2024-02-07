/*
 * BMEApp.h
 *
 *  Created on: 20230714
 *      Author: Will
 */

#ifndef INC_BMEAPP_H_
#define INC_BMEAPP_H_

#include "stdio.h"
#include "string.h"

/*==== AC Power info Start ====*/
#define INSTANT_POWER_LOG_SIZE 10
#define INSTANT_POWER_LOG_SIZE_INVERSE 0.1				/* = 1/INSTANT_POWER_LOG_SIZE */
#define AVERAGE_POWER_CALCULATE_PERIOD_IN_SEC	0.1		/* 100 ms */
#define INSTANT_POWER_UPDATE_PERIOD_IN_SEC		0.01 	/* 10 ms  = INSTANT_POWER_CALCULATE_PERIOD_IN_SEC / INSTANT_POWER_LOG_SIZE */

typedef void (*functypeBMEApp_AcPowerInfo_do100HzLoop)(void *, float);
typedef void (*functypeBMEApp_AcPowerInfo_Reset)(void *); 

typedef struct
{
	uint8_t LogCnt;
	uint32_t TotalLogCnt;		/* total oeperation time =  TotalLogCnt * 0.1 sec */
	float TotalPower;		
	float InstPower;
	float AvgPower;
	float InstPwrSum;
	functypeBMEApp_AcPowerInfo_Reset Reset;
	functypeBMEApp_AcPowerInfo_do100HzLoop do100HzLoop;
} AcPwrInfo_t;


extern void AcPowerInfo_Reset(AcPwrInfo_t *p);
extern void AcPowerInfo_do100HzLoop(AcPwrInfo_t *p, float PowerIn);

#define AC_POWER_INFO_DEFAULT \
{							\
	0,		/*LogCnt*/ 			\
	0,		/*TotalLogCnt*/		\
	0.0f,	/*TotalPower*/		\
	0.0f,	/*InstPower*/		\
	0.0f,	/*AvgPower*/		\
	0.0f,	/*InstPwrSum*/		\
	(functypeBMEApp_AcPowerInfo_Reset)AcPowerInfo_Reset,		\
	(functypeBMEApp_AcPowerInfo_do100HzLoop)AcPowerInfo_do100HzLoop,	\
}							\

extern AcPwrInfo_t AcPwrInfo;


/*==== AC Power info End ====*/


/*=============== Button handle start ===============*/

#define BTN_INITIAL_TIME_COUNT 10 - 1			/* Time interval for initializing, unit: 10ms*/
#define BTN_DEBOUNCE_TIME_COUNT 10 - 1		/* Time interval count for debouncing, unit: 10ms */
#define BTN_RECORDED_EVENT_AUTOCLEAR_TIME_THRESHOLD 100 - 1		/* Time interval count for clearing the unused event, unit: 10ms */

typedef enum 
{
    BTN_IDX_KILL_SW = 0,	/* killing switch */
		BTN_IDX_BST_BTN,			/* Boost button */
		BTN_IDX_REV_BTN,			/* Reverse button */	
    BTN_IDX_MAX,
} BtnIdx_e;

typedef enum 
{
    BTN_STATE_LOW = 0,
    BTN_STATE_HIGH,
    BTN_STATE_INITIALIZING,
} BtnState_e;

typedef enum 
{
    BTN_EVENT_IDLE,
    BTN_EVENT_RISING,
    BTN_EVENT_FALLING,
	BTN_EVENT_INITIALIZING,
} BtnEvent_e;

typedef struct
{
    BtnEvent_e EvtRecord;   /* the last signal event detected */
    int8_t Now;             /* The signal value read now */
    BtnState_e Before;      /* The state before */
    uint16_t SignalTimeCnt; /* Time counter after signal is changed */
		uint16_t EvtTimeCnt;    /* Time counter after event is recorded */
} BtnStateFb_t;

extern void Btn_Reset (BtnIdx_e Idx);
extern BtnEvent_e Btn_EvtRead (BtnIdx_e Idx);
extern void Btn_SignalWrite (BtnIdx_e Idx, int8_t DataIn);
extern BtnState_e Btn_StateRead (BtnIdx_e Idx);
extern void Btn_Init (void);
extern void Btn_Do100HzLoop (void);

/*=============== Button handle End ===============*/


#endif /* INC_BMEAPP_H_ */
