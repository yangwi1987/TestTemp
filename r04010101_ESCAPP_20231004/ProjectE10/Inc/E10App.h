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

#define SOC_INDICATION_VIA_FRONTLIGHT 1

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
	BTN_IDX_BRAKE_BTN,			/* Brake button */
	BTN_IDX_SS_BTN,				/* Start/Stop button*/
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

typedef enum
{
    DIO_KILL_SWITCH_STATE_OPEN = 0,
	DIO_KILL_SWITCH_STATE_CLOSED,
} DIO_KillSwState_e;

#define BTN_BOOST_PRESS		BTN_STATE_LOW
#define BTN_BOOST_RELEASE	BTN_STATE_HIGH

#define BTN_REVERSE_PRESS	BTN_STATE_LOW
#define BTN_REVERSE_RELEASE	BTN_STATE_HIGH

#define BTN_KILL_SW_PRESS	BTN_STATE_LOW
#define BTN_KILL_SW_RELEASE BTN_STATE_HIGH

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

/*=============== Button handle End ===============*/


/*=============== DO Indication control start ===============*/
typedef enum 
{
  DO_IDX_FRONT = 0,    /* FRONT READY LIGHT */
  DO_IDX_REAR,        	/* REAR READY LIGHT */
  DO_IDX_BOOST,       	/* BOOST LIGHT */
  DO_IDX_REV,    		/* REVERSE LIGHT */
  DO_IDX_KILLSW,    	/* KILL SWITCH */
  /* SOC-01 */
  DO_IDX_SOC1_R,
  DO_IDX_SOC1_G,
  DO_IDX_SOC1_B,
  /* SOC-02 */
  DO_IDX_SOC2_R,
  DO_IDX_SOC2_G,
  DO_IDX_SOC2_B,
  /* SOC-03 */
  DO_IDX_SOC3_R,
  DO_IDX_SOC3_G,
  DO_IDX_SOC3_B,
  /* SOC-04 */
  DO_IDX_SOC4_R,
  DO_IDX_SOC4_G,
  DO_IDX_SOC4_B,
  DO_IDX_MAX,
} DOIdx_e;

typedef enum
{
  DO_RGB_IDX_R,
  DO_RGB_IDX_G,
  DO_RGB_IDX_B,
} DORGBIdx_e;

typedef enum
{
  DO_CMD_OFF = 0,
  DO_CMD_ON,
} DOCmd_e;

#define DO_NBR_TO_BLINK_FOREVER 0	/* if number to blink == 0, the DO will blinks until the mode is updated */

typedef enum
{
  DO_MODE_OFF = 0,
  DO_MODE_ON,
  DO_MODE_BLINK,
} DOMode_e;

typedef struct
{
  uint16_t Period;      /* unit = 10ms */
  uint16_t OnTime;      /* unit = 10ms */
} DOBlinkConfig_t;

/* Add pre-defined configuration if required */
#define DO_BLINK_CONFIG_STEADY 	(DOBlinkConfig_t){0, 0}
#define DO_BLINK_CONFIG_1HZ 		(DOBlinkConfig_t){100, 50}
#define DO_BLINK_CONFIG_2HZ 		(DOBlinkConfig_t){50, 25}
#define DO_BLINK_CONFIG_4HZ 		(DOBlinkConfig_t){25, 12}
#define DO_BLINK_CONFIG_5HZ 		(DOBlinkConfig_t){20, 10}

typedef struct
{
  uint16_t TimeCnt;
  uint16_t NbrCnt;
  uint16_t NbrToBlink;
  DOBlinkConfig_t BlinkConfig;
  DOMode_e Mode;
  DOCmd_e Cmd;
} DOCtrl_t;

extern void DO_TurnOnReq(DOIdx_e Idx);
extern void DO_TurnOffReq(DOIdx_e Idx);
extern void DO_CtrlReq(DOIdx_e Idx, DOMode_e ModeIn, uint16_t NbrToBlink, DOBlinkConfig_t BlinkConfig);
extern DOCmd_e DO_CmdGet(DOIdx_e Idx);


typedef enum
{
  SOC_DISPLAY_MODE_DIRECTLY_ACCESS = 0,   /* bypass SOC display function */
  SOC_DISPLAY_MODE_SOC_LED_ARRAY,         /* show SOC value via RBG LED array */
  SOC_DISPLAY_MODE_SOC_SINGLE_LED,        /* show SOC value via single LED */
} SocDisplayMode_e;

typedef struct
{
  uint8_t 			Soc;
  uint8_t 			ReqFlag;
  SocDisplayMode_e  DisplayMode;
  DORGBIdx_e     	ColorReq;
  DOMode_e       	DOMode;
  DOBlinkConfig_t 	BlinkConfig;
} SocLightCtrl_t;

extern void SocLightModeSet( SocDisplayMode_e ModeIn);
extern void SocCtrlSet(DORGBIdx_e ColorIn, DOMode_e ModeIn, DOBlinkConfig_t DOConfig);
extern void SocValueSet( uint8_t SocIn);
void SocIndicationByBLinking(DOIdx_e DOIdxIn);
extern void SocLightCtrl_Do100HzLoop(void);

/*=============== DO Indication control End ===============*/


//extern void DIO_KillSWSignalCheck(void);
extern DIO_KillSwState_e DIO_KillSwStateGet(void);
extern void DIO_Do100HzLoop(void);
extern void DIO_Init(void);

#endif /* INC_BMEAPP_H_ */
