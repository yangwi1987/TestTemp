/*
 * Filter.h
 *
 *  Created on: Mar 11, 2020
 *      Author: Fernando
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#define MAX_COUNT_N 21

enum FILTER_TYPE_ENUM
{
	FILTER_TYPE_LPF=0,
	FILTER_TYPE_HPF,
};

enum FILTER_INIT_STATUS_ENUM
{
	FILTER_INIT_OK=0,
	FILTER_INIT_ERROR_PERIOD_IS_NOT_POSITIVE=0x8001,
	FILTER_INIT_ERROR_BANDWITH_IS_NOT_POSITIVE,
	FILTER_INIT_ERROR_TYPE_IS_NOT_DEFINED,
	FILTER_INIT_ERROR_Q_IS_NOT_POSITIVE,
};

enum FILTER_TIMES_ENUM
{
	FILTER_TIMES_FIRST=0,
	FILTER_TIMES_ABOVE_TWICE,
};

typedef void (*pfunFilter_Bilinear1OrderClean)( void* );
typedef uint16_t (*pfunFilter_Bilinear1OrderInit)( void*, void* );
typedef float (*pfunFilter_Bilinear1OrderCalc)( void*, float );
typedef float (*pfunBandPassCalc)( void*, float );
typedef void (*pfunZohBandPass_InitGain)( void*, float, float, float );
typedef void (*pfunZohBandPass_OnLineGain)( void*, float );

typedef void (*functypeMovingAverage_Clean)( void* );
typedef void (*functypeMovingAverage_Init)( void*, uint16_t );
typedef void (*functypeMovingAverage_Calc)( void*, float );

typedef void (*pfunFilter_BilinearBPFClean)( void* );
typedef uint16_t (*pfunFilter_BilinearBPFInit)( void*, void* );
typedef float (*pfunFilter_BilinearBPFCalc)( void*, float );




typedef struct
{
	float Gain;
	float PreviousX;
	float PreviousY;
	uint16_t Type : 15;
	uint16_t Times : 1;
	pfunFilter_Bilinear1OrderClean Clean;
	pfunFilter_Bilinear1OrderInit Init;
	pfunFilter_Bilinear1OrderCalc Calc;
}FILTER_BILINEAR_1ORDER_TYPE;

typedef struct
{
	float Period;
	float BandwithHz;
	uint16_t Type;
}FILTER_INIT_BILINEAR_1ORDER_TYPE;

typedef struct
{
	float XPrevious;
	float XPrevious2;
	float YPrevious;
	float YPrevious2;
	float Gain1;
	float Gain2;
	float Gain3;
	float Gain4;
	float GainConst1;
	float GainConst2;
	float Period;
	pfunBandPassCalc Calc;
	pfunZohBandPass_InitGain InitGain;
	pfunZohBandPass_OnLineGain OnLineGain;
}FILTER_ZOH_BANDPASS_TYPE;

typedef struct
{
	float LastInput[MAX_COUNT_N];
	float InputSum;
	uint16_t Index;
	uint16_t MovAvgNumber;
	float OutFilteredData;
	functypeMovingAverage_Clean Clean;
	functypeMovingAverage_Init Init;
	functypeMovingAverage_Calc Calc;
} MovingAverage_t;


typedef struct
{
	float Period;
	float BandwithHz;
	float Q;
}FilterBilinearBPFSetting_t;

typedef struct
{
	uint16_t Times;
	float XPre1;
	float XPre2;
	float YPre1;
	float YPre2;
	float GainX;
	float GainXPre2;
	float GainYPre1;
	float GainYPre2;
	float Out;
	pfunFilter_BilinearBPFClean Clean;
	pfunFilter_BilinearBPFInit Init;
	pfunFilter_BilinearBPFCalc Calc;
}FilterBilinearBPF_t;

void Filter_Bilinear1OrderClean(FILTER_BILINEAR_1ORDER_TYPE *p);
uint16_t Filter_Bilinear1OrderInit(FILTER_BILINEAR_1ORDER_TYPE *p, FILTER_INIT_BILINEAR_1ORDER_TYPE *pSetting);
float Filter_Bilinear1OrderCalc(FILTER_BILINEAR_1ORDER_TYPE *p, float X);
float ZohBandPass_Calc(FILTER_ZOH_BANDPASS_TYPE *p,float Input);
void ZohBandPass_InitGain(FILTER_ZOH_BANDPASS_TYPE *p,float Period, float Fc, float F0);
void ZohBandPass_OnLineGain(FILTER_ZOH_BANDPASS_TYPE *p,float Rad);
void MovingAverage_Clean( MovingAverage_t *v );
void MovingAverage_Init( MovingAverage_t *v, uint16_t Number);
void MovingAverage_Calc( MovingAverage_t *v, float Input);
void Filter_BilinearBPFClean( FilterBilinearBPF_t *p );
uint16_t Filter_BilinearBPFInit( FilterBilinearBPF_t *p, FilterBilinearBPFSetting_t *pSetting );
float Filter_BilinearBPFCalc( FilterBilinearBPF_t *p, float X );

#define FILTER_BILINEAR_1ORDER_DEFAULT	\
{										\
	0.0f,								\
	0.0f,								\
	0.0f,								\
	0,									\
	FILTER_TIMES_FIRST,					\
	(pfunFilter_Bilinear1OrderClean)Filter_Bilinear1OrderClean,	\
	(pfunFilter_Bilinear1OrderInit)Filter_Bilinear1OrderInit,	\
	(pfunFilter_Bilinear1OrderCalc)Filter_Bilinear1OrderCalc,	\
}

#define FILTER_INIT_BILINEAR_1ORDER_DEFAULT	\
{											\
	0.0f,									\
	0.0f,									\
	0,										\
}

#define FILTER_ZOH_BANDPASS_DEFAULT	\
{	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.01134f,	\
	-0.01134f,	\
	1.449f,		\
	-0.9875f,	\
	0.0f,		\
	0.0f,		\
	0.0001f,	\
	(pfunBandPassCalc) ZohBandPass_Calc,					\
	(pfunZohBandPass_InitGain) ZohBandPass_InitGain,		\
	(pfunZohBandPass_OnLineGain) ZohBandPass_OnLineGain,	\
}

#define MOVING_AVERAGE_DEFAULT	\
{	{0.0f}, \
	0.0f, \
	0, \
	0, 			/*MovAvgNumber*/ \
	0.0f, \
	(functypeMovingAverage_Clean) MovingAverage_Clean, \
	(functypeMovingAverage_Init) MovingAverage_Init, \
	(functypeMovingAverage_Calc) MovingAverage_Calc \
}

#define FILTER_BILINEAR_BPF_SETTING_DEFAULT \
{			\
	0.0f,	\
	0.0f,	\
	0.0f,	\
}

#define FILTER_BILINEAR_BPF_DEFAULT \
{			\
	0,		\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	0.0f,	\
	(pfunFilter_BilinearBPFClean) Filter_BilinearBPFClean,	\
	(pfunFilter_BilinearBPFInit) Filter_BilinearBPFInit,	\
	(pfunFilter_BilinearBPFCalc) Filter_BilinearBPFCalc,	\
}

#define FILTER_BILINEAR_1ORDER_CALC_MACRO( p, X, Y, Out )							\
{																					\
	Y = 0.0f;																		\
	if ( p->Times == FILTER_TIMES_FIRST)											\
	{																				\
		Y = 0.0f;																	\
		p->Times = FILTER_TIMES_ABOVE_TWICE;										\
	}																				\
	else																			\
	{																				\
		Y = p->Gain * ( X + p->PreviousX ) + ( 1 - 2 * p->Gain ) * p->PreviousY;	\
	}																				\
	p->PreviousY = Y;																\
	p->PreviousX = X;																\
	if ( p->Type == FILTER_TYPE_LPF )												\
	{																				\
																					\
	}																				\
	else	/*p->Type == TYPE_HPF*/													\
	{																				\
		Y = X - Y;																	\
	}																				\
	Out = Y;																		\
}

__STATIC_FORCEINLINE float Filter_Bilinear1OrderCalc_LPF_inline(FILTER_BILINEAR_1ORDER_TYPE *p, float X)
{
	float Y=0.0f;
	if ( p->Times != FILTER_TIMES_FIRST)
	{
		Y = p->PreviousY - p->Gain * (( 2 * p->PreviousY ) - X - p->PreviousX );
	}
	else
	{
		Y = 0.0f;
		p->Times = FILTER_TIMES_ABOVE_TWICE;
	}
	p->PreviousY = Y;
	p->PreviousX = X;

	return Y;
}

__STATIC_FORCEINLINE float Filter_Bilinear1OrderCalc_HPF_inline(FILTER_BILINEAR_1ORDER_TYPE *p, float X)
{
	float Y=0.0f;
	if ( p->Times != FILTER_TIMES_FIRST)
	{
		Y = p->PreviousY - p->Gain * (( 2 * p->PreviousY ) - X - p->PreviousX );
	}
	else
	{
		Y = 0.0f;
		p->Times = FILTER_TIMES_ABOVE_TWICE;
	}
	p->PreviousY = Y;
	p->PreviousX = X;

	Y = X - Y;

	return Y;
}

#define FILTER_BILINEAR_BPF_CALC_MACRO( p, X, Y )	\
{							\
	if ( p->Times < 2 )		\
	{						\
		p->Out = X;			\
		p->Times++;			\
	}						\
	else					\
	{						\
		p->Out = p->GainYPre1 * p->YPre1 + p->GainYPre2 * p->YPre2 + p->GainX * X + p->GainXPre2 * p->XPre2;	\
	}						\
	p->XPre2 = p->XPre1;	\
	p->XPre1 = X;			\
	p->YPre2 = p->YPre1;	\
	p->YPre1 = p->Out;		\
	Y = p->Out;				\
}

#endif /* INC_FILTER_H_ */
