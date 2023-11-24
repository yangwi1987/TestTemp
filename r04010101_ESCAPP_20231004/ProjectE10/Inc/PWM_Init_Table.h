/*
 * PWM_Init_Table.h
 *
 *  Created on: 2020年2月6日
 *      Author: MikeSFWen
 */
 
#if E10
#ifndef INC_PWM_INIT_TABLE_H_
#define INC_PWM_INIT_TABLE_H_

#include "stm32g4xx_hal.h"
#include "main.h"

/*
 * Extern variable
 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim20;

// Initial Define
#define PWM_CH_SIZE			6
#define PWM_AXIS_SIZE		2
#define PWM_GROUP_SIZE		3

/*
 * Enum declare
 */
enum ChannelName
{
	CH_PWM_UP = 0,
	CH_PWM_UN,
	CH_PWM_VP,
	CH_PWM_VN,
	CH_PWM_WP,
	CH_PWM_WN,
};

enum AxisNameEnum
{
	AXIS_0 = 0,
	AXIS_1,
};

enum PwmEnum
{
	// Axis
	TIM_1 = 0,
	TIM_8,
	TIM_20,
};

enum PwmEnaEnum
{
	PWM_DISABLE = 0,
	PWM_ENABLE,
};

enum PwmErrorEnum
{
	PWM_SUCCESS = 0,
	PWM_ERROR
};

enum PwmChEnum {
	TIM_CH1 = 0,
	TIM_CH2,
	TIM_CH3,
	TIM_CH4,
	TIM_CH5,
	TIM_CH6
};

enum PwmDone{
	PWM_NONE = 0,
	PWM_DONE
};

/*
 * PWM Structure define
 */
typedef struct {
	uint16_t PwmName: 		4;
	uint16_t GroupEnable: 	1;
	uint16_t Group:			3;
	uint16_t Channel: 		3;
	uint16_t reserved: 		5;
} PWM_START_TABLE_INFO;

typedef struct{
	TIM_TypeDef* GroupIdx;
	TIM_HandleTypeDef *TimGroup;
	uint16_t Channel1;
	uint16_t Channel2;
	uint16_t Channel3;
	uint16_t PreScale;
	uint16_t CounterMode;
	uint16_t Period;
	uint16_t Division;
	uint16_t RcrPara;
	uint16_t AutoPreloadMode;
	uint16_t ClkSource;
	uint16_t TrigSource1;
	uint16_t TrigSource2;
	uint16_t SlaveMode;
	uint16_t BreakSource : 		8;
	uint16_t BreakSourceEanble :8;
	uint16_t BreakSourcePriority;
	uint16_t OCMode;
	uint16_t InitDutyCntCh1st;
	uint16_t OCPolarity;
	uint16_t OCNPolarity;
	uint16_t OCFastMode;
	uint16_t OCIdleState;
	uint16_t OCNIdleState;
	uint16_t InitDutyCntCh2ed;
	uint16_t InitDutyCntCh3rd;
	uint16_t OffStateRunMode;
	uint16_t OffStateIDLEMode;
	uint16_t LockLevel;
	uint16_t DeadTimePara;
	uint16_t BreakState;
	uint16_t BreakPolarity;
	uint16_t BreakFilter;
	uint16_t BreakAFMode;
	uint16_t Break2State;
	uint32_t Break2Polarity;
	uint16_t Break2Filter;
	uint16_t Break2AFMode;
	uint16_t AutomaticOutput;
	uint16_t MinimumTime;
} PWM_TIM_INIT_PARA;

#define PWM_TIM_INIT_PARA_DEFAULT { \
	0, /*	GroupIdx			*/\
	0, /*	TimGroup			*/\
	0, /*	Channel1			*/\
	0, /*	Channel2			*/\
	0, /*	Channel3			*/\
	0, /*	PreScale			*/\
	0, /*	CounterMode			*/\
	0, /*	Period				*/\
	0, /*	RcrPara				*/\
	0, /*	AutoPreloadMode		*/\
	0, /*	ClkSource			*/\
	0, /*	TrigSource1			*/\
	0, /*	TrigSource2			*/\
	0, /*	SlaveMode			*/\
	0, /*	BreakSource			*/\
	0, /*	BreakSourceEanble	*/\
	0, /*	BreakSourcePriority	*/\
	0, /*	OCMode				*/\
	(8500-1), /*	InitDutyCntCh1st	*/\
	0, /*	OCPolarity			*/\
	0, /*	OCNPolarity			*/\
	0, /*	OCFastMode			*/\
	0, /*	OCIdleState			*/\
	0, /*	OCNIdleState		*/\
	(8500-1), /*	InitDutyCntCh2ed	*/\
	(8500-1), /*	InitDutyCntCh3rd	*/\
	0, /*	OffStateRunMode		*/\
	0, /*	OffStateIDLEMode	*/\
	0, /*	LockLevel			*/\
	0, /*	DeadTimePara		*/\
	0, /*	BreakState			*/\
	0, /*	BreakPolarity		*/\
	0, /*	BreakFilter			*/\
	0, /*	BreakAFMode			*/\
	0, /*	Break2State			*/\
	0, /*	Break2Polarity		*/\
	0, /*	Break2Filter		*/\
	0, /*	Break2AFMode		*/\
	0, /*	AutomaticOutput		*/\
	0  /*	MinimumTime			*/}

extern void PwmTIMx_Init_Func( const PWM_TIM_INIT_PARA *Para );
extern const PWM_TIM_INIT_PARA Pwm_Tim_Initializer_Axis1;
extern const PWM_START_TABLE_INFO PwmStartUpTable_Axis1[PWM_CH_SIZE];
extern const PWM_START_TABLE_INFO PwmStartUpTable_Axis2[PWM_CH_SIZE];

#endif /* INC_PWM_INIT_TABLE_H_ */
#endif /* E10 */
