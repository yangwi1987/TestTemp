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
		BTN_IDX_BRK_BTN,
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
extern void Btn_Init (void);
extern void Btn_Do100HzLoop (void);

/*=============== Button handle End ===============*/


/*=============== LED Indication control start ===============*/
typedef enum 
{
  LED_IDX_FRONT = 0,    /* FRONT READY LIGHT */
  LED_IDX_REAR,        	/* REAR READY LIGHT */
  LED_IDX_BOOST,       	/* BOOST LIGHT */
  LED_IDX_REV,    		  /* REVERSE LIGHT */
  /* SOC-01 */
  LED_IDX_SOC1_R,
  LED_IDX_SOC1_G,
  LED_IDX_SOC1_B,
  /* SOC-02 */
  LED_IDX_SOC2_R,
  LED_IDX_SOC2_G,
  LED_IDX_SOC2_B,
  /* SOC-03 */
  LED_IDX_SOC3_R,
  LED_IDX_SOC3_G,
  LED_IDX_SOC3_B,
  /* SOC-04 */
  LED_IDX_SOC4_R,
  LED_IDX_SOC4_G,
  LED_IDX_SOC4_B,
  LED_IDX_MAX,
} LedIdx_e;

typedef enum
{
  LED_RGB_IDX_R,
  LED_RGB_IDX_G,
  LED_RGB_IDX_B,
} LedRGBIdx_e;

typedef enum
{
  LED_CMD_OFF = 0,
  LED_CMD_ON,
} LedCmd_e;

#define LED_NBR_TO_BLINK_FOREVER 0	/* if number to blink == 0, the LED will blinks until the mode is updated */

typedef enum
{
  LED_MODE_OFF = 0,
  LED_MODE_ON,
  LED_MODE_BLINK,
} LedMode_e;

/* Add pre-defined configuration if required */
#define LED_BLINK_CONFIG_STEADY 	(LedBlinkConfig_t){0, 0}
#define LED_BLINK_CONFIG_1HZ 		(LedBlinkConfig_t){100, 50}
#define LED_BLINK_CONFIG_2HZ 		(LedBlinkConfig_t){50, 25}
#define LED_BLINK_CONFIG_4HZ 		(LedBlinkConfig_t){25, 12}
#define LED_BLINK_CONFIG_5HZ 		(LedBlinkConfig_t){20, 10}

typedef struct
{
  uint16_t Period;      /* unit = 10ms */
  uint16_t OnTime;      /* unit = 10ms */
} LedBlinkConfig_t;

typedef struct
{
  uint16_t TimeCnt;
  uint16_t NbrCnt;
  uint16_t NbrToBlink;
  LedBlinkConfig_t BlinkConfig;
  LedMode_e Mode;
  LedCmd_e Cmd;
} LedCtrl_t;

extern void Led_TurnOnReq(LedIdx_e Idx);
extern void Led_TurnOffReq(LedIdx_e Idx);
extern void Led_CtrlReq(LedIdx_e Idx, LedMode_e ModeIn, uint16_t NbrToBlink, LedBlinkConfig_t BlinkConfig);
extern void Led_Do100HzLoop(void);
extern LedCmd_e Led_CmdGet(LedIdx_e Idx);


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
  LedRGBIdx_e     	ColorReq;
  LedMode_e       	LedMode;
  LedBlinkConfig_t 	BlinkConfig;
} SocLightCtrl_t;

extern void SocLightModeSet( SocDisplayMode_e ModeIn);
extern void SocCtrlSet(LedRGBIdx_e ColorIn, LedMode_e ModeIn, LedBlinkConfig_t LedConfig);
extern void SocValueSet( uint8_t SocIn);
void SocIndicationByBLinking(LedIdx_e LedIdxIn);
extern void SocLightCtrl_Do100HzLoop(void);

/*=============== LED Indication control End ===============*/

#endif /* INC_BMEAPP_H_ */
