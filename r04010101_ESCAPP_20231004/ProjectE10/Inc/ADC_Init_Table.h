/*
 * ADC_Init_Table.h
 *
 *  Created on: 2020年2月7日
 *      Author: MikeSFWen
 */

#if E10
#ifndef INC_ADC_INIT_TABLE_H_
#define INC_ADC_INIT_TABLE_H_

#include "stm32g4xx_hal.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc5;

/* WARNING: DO NOT CHANGE THE ORDER*/
enum AdcInjGroupEnum {
	ISE_U_A0 = 0,
	ISE_V_A0,
	ISE_W_A0,
	ISE_U_A1,
	ISE_V_A1,
	ISE_W_A1,
	BAT_VDC,
	INJ_REVD_8,
	INJ_REVD_9
};

/* WARNING: DO NOT CHANGE THE ORDER*/
enum AdcRegGroupEnum {
	HW_ID2 = 0,
	ES5V_FB,
	HW_ID1,
	E5V_FB,
	EA5V_FB,
	PREC_FB,
	S13V8,
	ACC_FB2,
	ACC_FB1,
	REG_REVD_9,
	REG_REVD_10,
	REG_REVD_11,
	REG_REVD_12
};

/* WARNING: DO NOT CHANGE THE ORDER*/
enum AdcThermoNameEnum {
	MOS_NTC_1 = 0,
	MOS_NTC_2,
	CAP_NTC,
	PCU_NTC_3,
	PCU_NTC_4,
	MOTOR_NTC_0_A0,
	MOTOR_NTC_1_A0,
	MOTOR_NTC_2_A0,
	MOTOR_NTC_0_A1,
	MOTOR_NTC_1_A1,
	MOTOR_NTC_2_A1,
	NTC_NONE
};

enum AdcGroup {
	GROUP_DISABLE = 0,
	ADC_1,
	ADC_2,
	ADC_3,
	ADC_4,
	ADC_5,
};

enum AdcRank {
	RANK_DISABLE = 0,
	RANK_1,
	RANK_2,
	RANK_3,
	RANK_4,
	RANK_5,
	RANK_6,
	RANK_7,
	RANK_8,
	RANK_9,
	RANK_10,
	RANK_11,
	RANK_12,
	RANK_13,
	RANK_14,
	RANK_15,
	RANK_16
};

enum AdcEnable {
	ADC_DISABLE = 0,
	ADC_ENABLE = 1
};

enum AdcThermoTableIdxEnum {
	NO_TABLE = 0,
	NTCG163JX103DT1S,			// On board NTC for MOS
	GWX_LS103H20,				// Motor NTC
	GWX_LS103H12,				// On board NTC for CAP
};

/* Table Size Define */
#define ADC_GROUP_CNT							5
#define ADC_ADDRESS_SIZE						6
#define ADC_DMA_CHANNEL_SIZE					8
#define ADC_DMA_GROUP_SIZE						5
#define CURRENT_LOOP_CHANNEL_SIZE				9
#define PLC_LOOP_CHANNEL_SIZE					13
#define THERMAL_CHANNEL_SIZE					NTC_NONE
#define REGULAR_RANK_SIZE						16
#define INJECTED_RANK_SIZE						5
#define NTC_TYPE_NUMBER							3

/* Gain AD to V */
#define AD2VGAIN								( float )0.000805600f // thermal ADC to 3.3V analog check again! todo
#define AC_CURR_GAIN							( float )-0.372265625f // motor phase current sensor
#define DC_VOLT_GAIN							( float )0.019306152f // DC BUS, pre-charge input
#define V13V_GAIN								( float )0.004672852f // sense 13 V pin
#define _5V_GAIN								( float )0.001220703f // pedal input 1,2
#define _9.9V_GAIN								( float )0.002416992f // sense 5V pin, but max analog value is 9.9V

#define DEFAULT_GAIN							( float )1.0f  // HW ID1,2

/* NTC Table Define */
#define NTC_TYPE_CNT							4
#define NTC_LENGTH								64

/* Zero Calibration Define */
#define ADC_ZERO_CALIB_SHIFT_NUM				7
#define ADC_ZEROCALIB_MAX_CNT					0x1 << ADC_ZERO_CALIB_SHIFT_NUM

typedef struct{
	uint16_t Group:					4;
	uint16_t GroupEna:				4;
	uint16_t DmaEnable:				4;
	uint16_t InjectedMode:		 	4;
} ADC_GROUP_INFO;

typedef struct{
	uint16_t	NameDef: 		5;
	uint16_t	ZeroEnable: 	1;
	uint16_t 	ChEnable:		1;
	uint16_t	AdcGroup:		4;
	uint16_t	AdcInjRank:		5;
	float		GainValue;
} ADC_INJ_GROUP_TABLE;

typedef struct{
	uint16_t	NameDef: 		5;
	uint16_t	ZeroEnable: 	1;
	uint16_t 	ChEnable:		1;
	uint16_t    AdcGroup:	    4;
	uint16_t	AdcRegRank:     5;
	float		GainValue;
} ADC_DMA_GROUP_TABLE;

typedef struct{
	uint16_t	NameDef: 		4;
	uint16_t	ZeroEnable: 	1;
	uint16_t	TableIdx:		3;
	uint16_t 	ChEnable:		1;
	uint16_t	AdcGroup:		3;
	uint16_t	AdcRank:		4;
} ADC_THERMO_TRA_TABLE;

typedef struct{
	float A6;
	float A5;
	float A4;
	float A3;
	float A2;
	float A1;
	float A0;
} NTC_CURVE_GAIN;

#define NTC_CURVE_GAIN_DEFAULT { \
	0.0, \
	0.0, \
	0.0, \
	0.0, \
	0.0, \
	0.0, \
	0.0 }

#define NTC_BREAK_REPLACE_TABLE_DEFAULT	\
{			\
	0,		\
	0,		\
	0.0f,	\
	0.0f	\
}

extern const ADC_GROUP_INFO AdcSetupTab[ ADC_GROUP_CNT ];
extern const ADC_INJ_GROUP_TABLE AdcInjectionGroupTable[ CURRENT_LOOP_CHANNEL_SIZE ];
extern const ADC_DMA_GROUP_TABLE AdcRegularGroupTable[ PLC_LOOP_CHANNEL_SIZE ];
extern const ADC_THERMO_TRA_TABLE AdcThermoTable[ THERMAL_CHANNEL_SIZE ];
extern const NTC_CURVE_GAIN AdcCurveFittingTable[ NTC_TYPE_NUMBER ];

#endif /* INC_ADC_INIT_TABLE_H_ */
#endif /* E10 */
