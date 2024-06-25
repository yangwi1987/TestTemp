/*
 * FunctionGen.h
 *
 *  Created on: 2024年6月25日
 *      Author: Jeff C
 */

#ifndef INC_FUNCTIONGEN_H_
#define INC_FUNCTIONGEN_H_

#include "MathFunctionInclude.h"
#include "Constant.h"

typedef void (*functypeFunctionGenerator_Init)(void*,float,float,float,float,float,float);
typedef float (*functypeFunctionGenerator_Calc)(void*);

typedef enum
{
	FG_SIGNAL_SELECT_NONE = 0,
	FG_SIGNAL_SELECT_SINE,
	FG_SIGNAL_SELECT_SQUARE,
	FG_SIGNAL_SELECT_TRIANGLE,
	FG_SIGNAL_SELECT_ALL
}FG_SIGNAL_SELECT_e;

typedef enum
{
	FG_SIGNAL_PATTERN_ONCE = 0,
	FG_SIGNAL_PATTERN_REPEAT,
	FG_SIGNAL_PATTERN_UPDOWN,
}FG_PATTERN_SELECT_e;

typedef struct
{
	FG_SIGNAL_SELECT_e Signal_Select;
	uint8_t Start;
	FG_PATTERN_SELECT_e Pattern;
	uint8_t Direction;
	uint32_t Total_Step;
	uint32_t Step_Now;
    float Signal_Now;
    float Freq_Now;
    float Freq_Step;
    float Offset;
    float Amplitude;
    float ExePeriod;
    float StartFreq;
    float EndFreq;
	functypeFunctionGenerator_Init Init;
	functypeFunctionGenerator_Calc Calc;
}FG_t;

#define FG_DEFAULT {\
	FG_SIGNAL_SELECT_NONE,/*Signal_Select*/\
	0,/*Start*/\
	FG_SIGNAL_PATTERN_ONCE,/*Pattern*/\
    0,/*Direction*/\
	0,/*Total_Step*/\
	0,/*Step_Now*/\
	0.0f,/*Signal_Now*/\
    0.0f,/*Freq_Now; */\
    0.0f,/*Freq_Step;*/\
    0.0f,/*Offset;   */\
    0.0f,/*Amplitude;*/\
    0.0f,/*ExePeriod;*/\
	0.0f,/*StartFreq;*/\
	0.0f,/*EndFreq;*/\
	(functypeFunctionGenerator_Init)FunctionGenerator_Init,\
	(functypeFunctionGenerator_Calc)FunctionGenerator_None_Calc,\
}

void FunctionGenerator_Init(FG_t* v, float Start_Freq, float End_Freq, float duration, float ExePeriod, float Amplitude, float offset);
float FunctionGenerator_Sin_Calc( FG_t* v );
float FunctionGenerator_Square_Calc( FG_t* v );
float FunctionGenerator_Triangle_Calc( FG_t* v );
float FunctionGenerator_None_Calc( FG_t* v );



#endif /* INC_FUNCTIONGEN_H_ */
