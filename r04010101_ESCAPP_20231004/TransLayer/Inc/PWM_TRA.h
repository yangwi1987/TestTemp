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

#define BOOTSTRAP_COUNTER_MAX		100

enum PWM_MODE
{
	PWM_COMPLEMENTARY = 0b000111111,
	PWM_UbVbWx = 0b00001111,
	PWM_UxVbWb = 0b00111100,
	PWM_UbVxWb = 0b00110011,
};

enum PWM_6_STEP_STATE {
	PWM_CH_DISABLE = 0,
	PWM_CH_UP_ARM_ONLY,
	PWM_CH_LOW_ARM_ONLY,
	PWM_CH_ENABLE
};

typedef void ( *functypePwmStation_Init )( void* );
typedef void ( *functypePwmStation_AxisChannelLock )( void *, uint16_t );
typedef void ( *functypePwmStation_AxisChannelUnlock )( void *, uint16_t );
typedef uint16_t ( *functypePwmStation_AxisDutyToPwmCount )( void *, uint16_t, uint16_t );

typedef struct{
	uint16_t PwmMode;
	float Duty[PWM_CH_SIZE];
} DUTY_INPUT;

#define DUTY_INPUT_DEFAULT { \
	0,						\
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f } }

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
	uint16_t PwmMode;
	uint8_t ASC_Enable;
	float Duty2PwmCountGain;
	uint32_t RepetCounter; // = RCR bit in timer register
	uint32_t PWMPeriodCnt; // = (ARR bit in timer register + 1)
	DUTY_INPUT DutyCmd;
	PWM_CH_STATE PwmChState;
	PWM_CHANNEL_AXIS PwmCh[PWM_AXIS_SIZE];
	PWM_TIM_INIT_PARA PwmTimerParam[PWM_AXIS_SIZE];
	functypePwmStation_Init Init;
	functypePwmStation_AxisChannelLock AxisChannelLock;
	functypePwmStation_AxisChannelUnlock AxisChannelUnlock;
	functypePwmStation_AxisDutyToPwmCount AxisDutyToPwmCount;
} PwmStation;

void PwmStation_Init( PwmStation *v );
void PwmStation_AxisChannelLock( PwmStation *v, uint16_t AxisID );
void PwmStation_AxisChannelUnlock( PwmStation *v, uint16_t AxisID );
void PwmStation_AxisChannelStartup( PwmStation *v, uint16_t AxisID, const PWM_START_TABLE_INFO *pTable );
void PwmStation_AxisChannelMapping( PwmStation *v, uint16_t AxisID, const PWM_START_TABLE_INFO *pTable );
uint16_t PwmStation_AxisDutyToPwmCount( PwmStation *v, uint16_t AxisID, uint16_t PwmMode );

#define PWM_STATION_DEFAULT { \
	0, \
	0, \
	0.0f, \
	INITIAL_REPET_COUNTER, /* RepetCounter */ \
	INITIAL_PWM_PERIOD, /* PWMPeriodCnt */ \
	DUTY_INPUT_DEFAULT, \
	PWM_CH_STATE_DEFAULT, \
	{PWM_CHANNEL_AXIS_DEFAULT, PWM_CHANNEL_AXIS_DEFAULT}, \
	{PWM_TIM_INIT_PARA_DEFAULT, PWM_TIM_INIT_PARA_DEFAULT}, \
	(functypePwmStation_Init)PwmStation_Init, \
	(functypePwmStation_AxisChannelLock)PwmStation_AxisChannelLock, \
	(functypePwmStation_AxisChannelUnlock)PwmStation_AxisChannelUnlock, \
	(functypePwmStation_AxisDutyToPwmCount)PwmStation_AxisDutyToPwmCount \
}

#endif /* SRC_PWM_TRA_H_ */
