/*
 * PWM_TRA.h
 *
 *  Created on: Jan 14, 2020
 *      Author: Hank Chen
 */

#ifndef SRC_PWM_TRA_H_
#define SRC_PWM_TRA_H_

#include "stdint.h"
#include "PCUTableLinker.h"


typedef void ( *functypePwmStation_Init )( void* );

typedef struct{
	uint16_t UPhase  : 5;
	uint16_t VPhase  : 5;
	uint16_t WPhase  : 5;
	uint16_t Reserved : 1;
} PWM_CH_STATE;

#define PWM_CH_STATE_DEFAULT 	\
{							 	\
	0,							\
	0,							\
	0,							\
	0							\
}

typedef struct{
	TIM_HandleTypeDef *Group;
	uint16_t UpChannel;
	uint16_t UnChannel;
	uint16_t VpChannel;
	uint16_t VnChannel;
	uint16_t WpChannel;
	uint16_t WnChannel;
} PWM_CHANNEL_AXIS;

#define PWM_CHANNEL_AXIS_DEFAULT { \
	0, \
	0, \
	0, \
	0, \
	0, \
	0, \
	0 }

typedef struct{
	uint32_t RepetCounter; // = RCR bit in timer register
	uint32_t PWMPeriodCnt; // = (ARR bit in timer register + 1)
	PWM_CH_STATE PwmChState;
	PWM_CHANNEL_AXIS PwmCh[PWM_AXIS_SIZE];
	PWM_TIM_INIT_PARA PwmTimerParam[PWM_AXIS_SIZE];
	functypePwmStation_Init Init;
} PwmStation;

void PwmStation_Init( PwmStation *v );

#define PWM_STATION_DEFAULT { \
	INITIAL_REPET_COUNTER, /* RepetCounter */ \
	INITIAL_PWM_PERIOD, /* PWMPeriodCnt */ \
	PWM_CH_STATE_DEFAULT, \
	{PWM_CHANNEL_AXIS_DEFAULT, PWM_CHANNEL_AXIS_DEFAULT}, \
	{PWM_TIM_INIT_PARA_DEFAULT, PWM_TIM_INIT_PARA_DEFAULT}, \
	(functypePwmStation_Init)PwmStation_Init, \
}

#endif /* SRC_PWM_TRA_H_ */
