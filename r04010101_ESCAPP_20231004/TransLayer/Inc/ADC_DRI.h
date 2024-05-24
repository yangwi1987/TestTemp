/*
 * ADC_DRI.h
 *
 *  Created on: Dec 30, 2019
 *      Author: Hank Chen
 */

#include "main.h"
#include "stm32g4xx_hal.h"
#include "PCUTableLinker.h"

#ifndef ADC_DRI_H_
#define ADC_DRI_H_

enum AdcStateEnum{
	ADC_INITIAL = 0,
	ADC_PRELOAD,
	ADC_READING,
	ADC_PROCESS
};

enum AdcDmaRead {
	ADC1_DMA_ENABLE = 0x01,
	ADC2_DMA_ENABLE = 0x02,
	ADC3_DMA_ENABLE = 0x04,
	ADC4_DMA_ENABLE = 0x08,
	ADC5_DMA_ENABLE = 0x10
};

enum AdcChConvClpt {
	ADC_NONE = 0,
	ADC_DONE
};

typedef void ( *functypeAdcStation_Init )( void* );
typedef void ( *functypeAdcStation_DoCurrentLoop )( void * );
typedef void ( *functypeAdcStation_MarkInjectionGroupReadFlag )( void *, void * );
typedef void ( *functypeAdcStation_MarkRegularGroupReadFlag )( void *, void * );
typedef void ( *functypeAdcStation_ReadInjectionGroupValue )( void *, void * );
typedef void ( *functypeAdcStation_DoPLCLoop )( void * );
typedef void ( *functypeAdcStation_Do100HzLoop )( void * );

typedef struct{
	ADC_HandleTypeDef	*AdcGroup;
	uint16_t	NameDef: 		4;
	uint16_t	ZeroEnable: 	1;
	uint16_t 	ChEnable:		1;
	uint16_t	AdcInjRank:		10;
	float		GainValue;
} ADC_INJ_GROUP_CHANNEL;

#define ADC_INJ_GROUP_CHANNEL_DEFAULT { \
	0, \
	0, \
	0.0f }

typedef struct{
	uint16_t	AdcGroupIndex:	4;
	uint16_t	NameDef: 		5;
	uint16_t	ZeroEnable: 	1;
	uint16_t 	ChEnable:		1;
	uint16_t	AdcRankIndex:   5;
	float		GainValue;
} ADC_REG_GROUP_CHANNEL;

#define ADC_REG_GROUP_CHANNEL_DEFAULT { \
	0, \
	0.0f }

typedef struct{
	uint16_t	AdcGroupIndex:	3;
	uint16_t	NameDef: 		4;
	uint16_t	ZeroEnable: 	1;
	uint16_t	TableIndex:		3;
	uint16_t 	ChEnable:		1;
	uint16_t	AdcRankIndex:	4;
} ADC_THERMO_CHANNEL;

#define ADC_THERMO_CHANNEL_DEFAULT { \
	0 }

typedef struct{
	long 		ZeroValue;						// Zero calibration value
	uint16_t	RawAdcValue;					// Present Adc value
	uint16_t 	ZeroFinish :		4;			// Zero finish flag
	uint16_t	AdcReadDone:		4;			// Adc channel read finish
	uint16_t 	ZeroCnt	   :		8;			// Zero calibration count, it used to counting the zero times
} ADC_CH_DATA;

#define ADC_CH_DATA_DEFAULT { \
	0, \
	0, \
	0 }

typedef struct{
	ADC_CH_DATA Inj[CURRENT_LOOP_CHANNEL_SIZE];
	ADC_CH_DATA Reg[PLC_LOOP_CHANNEL_SIZE];
} ADC_CH_RAW_DATA;

#define ADC_CH_RAW_DATA_DEFAULT { \
	{ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, \
	ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT}, \
	{ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, \
	ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, \
	ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT, ADC_CH_DATA_DEFAULT} }

typedef struct{
	float  Iu[2];
	float  Iv[2];
	float  Iw[2];
	float  BatVdc;
	float  Idc; // no use
	float  HwID1;
	float  HwID2;
	float  ES5V; // sense 5V for monitor voltage source of I2C
	float  E5V; // sense 5V for monitor voltage source of position sensor
	float  EA5V; // sense 5V for monitor voltage source of pedal sensor
	float  PreC; // analog input pin to indicate the precharge finish
	float  S13V8;
	float  Pedal_V1;
	float  Pedal_V2;
	float  PCU_NTC[3];
	float  MOTOR_NTC_0;
	float  MOTOR_NTC_1;
	float  MOTOR_NTC_2;
	float  Throttle;
} ADC_TRA_DATA;

typedef union{
	float FDta;
	int16_t	Int16Type[2];
	uint16_t Uint16Type[2];
	int8_t Int8Type[4];
	uint8_t Uint8Type[4];
}ADC_EXE_TYPE_DEFINE;

#define ADC_EXE_TYPE_DEFINE_DEFAULT {	\
	0.0f	\
}	\

#define ADC_TRA_DATA_DEFAULT { \
	{0.0, 0.0},         /*Iu[2];      */\
	{0.0, 0.0},         /*Iv[2];      */\
	{0.0, 0.0},         /*Iw[2];      */\
	0.0,                /*BatVdc;     */\
	0.0,                /*Idc;        */\
	0.0,                /*HwID1;      */\
	0.0,                /*HwID2;      */\
	0.0,                /*ES5V;       */\
	0.0,                /*E5V;        */\
	0.0,                /*EA5V;       */\
	0.0,                /*PreC;       */\
	0.0,                /*S13V8;      */\
	0.0,                /*Pedal_V1;   */\
	0.0,                /*Pedal_V2;   */\
	{0.0f, 0.0f, 0.0f}, /*PCU_NTC[3]; */\
	0.0f,               /*MOTOR_NTC;  */\
    0.0f,               /*MOTOR_NTC_1;*/\
    0.0f,               /*MOTOR_NTC_2;*/\
	0.0f                /*Throttle;   */\
	}

typedef struct {
	uint16_t NTCID;
	uint16_t ReplaceID;
	float *Temperature;
	float *ReplaceTemperature;
	float p0;
	float p1;
} NTC_BREAK_REPLACE_t;

#define NTC_BREAK_REPLACE_DEFAULT { \
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	0.0f,	\
}

typedef struct {
	uint16_t 			  AdcState;									// Indicating ADC status
	uint16_t			  ThermoTraIndex;							// A ring queue index indicating the current processing thermo channel
	uint16_t			  ThermoADCCatcher;							// Catch temperature ADC every 10ms
	uint16_t			  ThermoADCCatchValue[THERMAL_CHANNEL_SIZE];// Catch temperature ADC every 10ms
	uint16_t			  ZeroCalibInjChCount;						// Record the total injection group channels that need zero calibration
	uint16_t			  ZeroCalibInjDone;							// All the injection group channels that need zero calibration done
	uint16_t			  ZeroCalibRegChCount;						// Record the total regular group channels that need zero calibration
	uint16_t			  ZeroCalibRegDone;							// All the regular group channels that need zero calibration done
	uint16_t			  AdcInjGroup;								// A flag recording which ADC injection group had done in that current loop interrupt
	uint16_t			  AdcRegGroup;								// A flag recording which ADC regular group had done in that system loop interrupt
	uint16_t			  AdcInjGroupFlag;							// A flag recording all the ADC injection group that need to be done in current loop
	uint16_t			  AdcRegGroupFlag;							// A flag recording all the ADC regular group that need to be done in current loop
	ADC_EXE_TYPE_DEFINE   AdcExeGain[CURRENT_LOOP_CHANNEL_SIZE];	// Execution Gain Point U, V, W
	uint16_t		      AdcExeZeroP[CURRENT_LOOP_CHANNEL_SIZE]; 	// Execution Zero Point U, V, W
	ADC_EXE_TYPE_DEFINE	  AdcExeThrotGain;							// Execution Throt Gain
	float			      AdcExeThrotZero;							// Execution Throt Zero
	float		    	  AdcExeThrotMax;							// Execution Throt Max
	ADC_INJ_GROUP_CHANNEL InjCh[CURRENT_LOOP_CHANNEL_SIZE];			// Record injection channel informations from table at initial
	ADC_REG_GROUP_CHANNEL RegCh[PLC_LOOP_CHANNEL_SIZE];				// Record regular channel informations from table at initial
	ADC_THERMO_CHANNEL	  ThermoCh[THERMAL_CHANNEL_SIZE];			// Record thermo channel informations from table at initial
	NTC_CURVE_GAIN		  NTCTable[NTC_TYPE_NUMBER];				// Record thermo tables at initial
	ADC_TRA_DATA		  AdcTraOut;								// Output: Adc values after  transition
	ADC_CH_RAW_DATA		  AdcRawData;								// Record Adc raw data
	uint16_t			  AdcDmaChCnt[ADC_DMA_GROUP_SIZE];
	uint16_t			  AdcDmaData[ADC_DMA_GROUP_SIZE][ADC_DMA_CHANNEL_SIZE];
	uint16_t			  *pTempADCValue[THERMAL_CHANNEL_SIZE];				// Temperature ADC pointer
	float			  ThrotADCRawRatio;
	uint16_t			  NTCIsAbnormal;
	functypeAdcStation_Init	Init;
	functypeAdcStation_DoCurrentLoop DoCurrentLoop;
	functypeAdcStation_MarkInjectionGroupReadFlag MarkInjectionGroupReadFlag;
	functypeAdcStation_MarkRegularGroupReadFlag MarkRegularGroupReadFlag;
	functypeAdcStation_ReadInjectionGroupValue ReadInjectionGroupValue;
	functypeAdcStation_DoPLCLoop DoPLCLoop;
	functypeAdcStation_Do100HzLoop Do100HzLoop;
} AdcStation;

void AdcStation_Init( AdcStation *v );
void AdcStation_ZeroCalibInjectionGroup( AdcStation *v );
void AdcStation_ZeroCalibRegularGroup( AdcStation *v );
void AdcStation_MarkInjectionGroupReadFlag( AdcStation *v, ADC_HandleTypeDef* hadc );
void AdcStation_MarkRegularGroupReadFlag( AdcStation *v, ADC_HandleTypeDef* hadc );
void AdcStation_ReadInjectionGroupValue( AdcStation *v, ADC_HandleTypeDef* hadc );
float AdcStation_DoThermoTransition( AdcStation *v );
void AdcStation_DoCurrentLoop( AdcStation *v );
void AdcStation_DoPLCLoop( AdcStation *v );
void AdcStation_Do100HzLoop( AdcStation *v );


#define ADC_STATION_DEFAULT { \
	0, \
	0, \
	0, \
	{0,0,0,0,0,0,0,0,0,0,0}, \
	0, \
	0, \
	0, \
    0, \
	0, \
	0, \
	0, \
	0, \
	{ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, \
	ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT, ADC_EXE_TYPE_DEFINE_DEFAULT },	\
	{0,    0,    0,    0,    0,    0,    0,    0,    0    },	\
	ADC_EXE_TYPE_DEFINE_DEFAULT,	\
	0.0f,	\
	0.0f,	\
	{ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, \
	 ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT, ADC_INJ_GROUP_CHANNEL_DEFAULT}, \
	{ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, \
	 ADC_REG_GROUP_CHANNEL_DEFAULT,	ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, \
	 ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT, ADC_REG_GROUP_CHANNEL_DEFAULT}, \
	{ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, \
	 ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT, ADC_THERMO_CHANNEL_DEFAULT}, \
	{NTC_CURVE_GAIN_DEFAULT, NTC_CURVE_GAIN_DEFAULT}, \
	ADC_TRA_DATA_DEFAULT, \
	ADC_CH_RAW_DATA_DEFAULT, \
	{0,0,0,0,0}, \
	{{0,0,0,0,0,0,0,0}, \
	 {0,0,0,0,0,0,0,0}, \
	 {0,0,0,0,0,0,0,0}, \
	 {0,0,0,0,0,0,0,0}, \
	 {0,0,0,0,0,0,0,0}}, \
	 {0,0,0,0,0,0,0,0,0,0,0}, \
	 0.0f, \
	 0, \
	(functypeAdcStation_Init)AdcStation_Init, \
	(functypeAdcStation_DoCurrentLoop)AdcStation_DoCurrentLoop, \
	(functypeAdcStation_MarkInjectionGroupReadFlag)AdcStation_MarkInjectionGroupReadFlag, \
	(functypeAdcStation_MarkRegularGroupReadFlag)AdcStation_MarkRegularGroupReadFlag, \
	(functypeAdcStation_ReadInjectionGroupValue)AdcStation_ReadInjectionGroupValue, \
	(functypeAdcStation_DoPLCLoop)AdcStation_DoPLCLoop, \
	(functypeAdcStation_Do100HzLoop) AdcStation_Do100HzLoop }



#define ADC_HANDLE_INJECTION_GROUP_MACRO( v, hadc ) \
	if( __HAL_ADC_GET_FLAG( hadc, ADC_FLAG_JEOS ) )	\
	{												\
		if( hadc->Instance == ADC1 ) 			\
		{										\
			v->AdcInjGroup |= 0x01;				\
			v->AdcRawData.Inj[0].RawAdcValue = hadc->Instance->JDR1;	\
			v->AdcRawData.Inj[2].RawAdcValue = hadc->Instance->JDR2;	\
			v->AdcRawData.Inj[6].RawAdcValue = hadc->Instance->JDR3;	\
		} 										\
		else if( hadc->Instance == ADC2 ) 		\
		{										\
			v->AdcInjGroup |= 0x02;				\
			v->AdcRawData.Inj[1].RawAdcValue = hadc->Instance->JDR1;	\
		} 										\
		else if( hadc->Instance == ADC3 ) 		\
		{										\
			v->AdcInjGroup |= 0x04;				\
		} 										\
		else if( hadc->Instance == ADC4 ) 		\
		{										\
			v->AdcInjGroup |= 0x08;				\
		} 										\
		else if( hadc->Instance == ADC5 ) 		\
		{										\
			v->AdcInjGroup |= 0x10;				\
		}										\
	}


/* Brief this function ADC_HANDLE_INJECTION_GROUP_MACRO_E10.
 * Add enabled ADC group in the flow control of "hadc->Instance"
 * The index of v->AdcRawData.Inj[index] is the row number of AdcInjectionGroupTable started from 0.
 * The x of hadc->Instance->JDRx is the ADC injection rank of the ADC group.
 * */
#define ADC_HANDLE_INJECTION_GROUP_MACRO_E10( v, hadc ) \
	if( __HAL_ADC_GET_FLAG( hadc, ADC_FLAG_JEOS ) )	\
	{												\
/*		if( hadc->Instance == ADC1 ) 			\
		{										\
			v->AdcInjGroup |= 0x01;				\
		} 										\
		else if( hadc->Instance == ADC2 ) 	  */\
		if( hadc->Instance == ADC2 ) 			\
		{										\
			v->AdcRawData.Inj[2].RawAdcValue = hadc->Instance->JDR1;	\
			v->AdcInjGroup |= 0x02;				\
		} 										\
		else if( hadc->Instance == ADC3 ) 		\
		{										\
			v->AdcRawData.Inj[1].RawAdcValue = hadc->Instance->JDR1;	\
			v->AdcRawData.Inj[6].RawAdcValue = hadc->Instance->JDR2;	\
			v->AdcInjGroup |= 0x04;				\
		} 										\
		else if( hadc->Instance == ADC4 ) 		\
		{										\
			v->AdcRawData.Inj[0].RawAdcValue = hadc->Instance->JDR1;	\
			v->AdcInjGroup |= 0x08;				\
		} 										\
		else if( hadc->Instance == ADC5 ) 		\
		{										\
			v->AdcInjGroup |= 0x10;				\
		}										\
	}

#endif /* ADC_DRI_H_ */
