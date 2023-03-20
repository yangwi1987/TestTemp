/*
 * Pid.h
 *
 *  Created on: 2019年12月4日
 *      Author: Fernando
 */

#ifndef INC_PID_H_
#define INC_PID_H_

//#define PID_INIT_OK											0
//#define PID_ERROR_INIT_PERIOD_IS_NOT_NEGATIVE				0x8001
//#define PID_ERROR_INIT_LOWER_LIMIT_IS_LARGER_UPPER_LIMIT	0x8002
//#define PID_ERROR_INIT_KP_IS_NAGATIVE						0x8003
//#define PID_ERROR_INIT_KI_IS_NAGATIVE						0x8004

enum PID_INIT_STATUS_ENUM
{
	PID_INIT_OK = 0,
	PID_INIT_ERROR_PERIOD_IS_NOT_NEGATIVE	= 0x8001,
	PID_INIT_ERROR_LOWER_LIMIT_IS_LARGER_UPPER_LIMIT,
	PID_INIT_ERROR_KP_IS_NAGATIVE,
	PID_INIT_ERROR_KI_IS_NAGATIVE
};

typedef uint16_t (*pPiInit)( float, float, float, float, float, void* );
typedef void (*pPiClean)( void* );
typedef void (*pPiControl)( float, float, void*);
typedef void (*pPiControlNoLimit)( float, float, void* );

typedef struct
{
	float Error;
	float Period;
	float Kp;
	float Ki;
	float Up;
	float Ui;
	float Output;
	float UpperLimit;
	float LowerLimit;
	pPiInit	Init;
	pPiClean Clean;
	pPiControl Calc;
	pPiControlNoLimit CalcNoLimit;
} PI_TYPE;

uint16_t Pid_PiInit( float Period, float Kp, float Ki, float UpperLimit, float LowerLimit, PI_TYPE* p );
void Pid_PiClean( PI_TYPE* p );
void Pid_PiControl( float Command, float Feedback, PI_TYPE* p );
void Pid_PiControlNoLimit( float Command, float Feedback, PI_TYPE* p );

#define PI_DEFAULT 	\
{ 					\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	0.0f,			\
	(pPiInit)Pid_PiInit,		\
	(pPiClean)Pid_PiClean,	\
	(pPiControl)Pid_PiControl,	\
	(pPiControlNoLimit)Pid_PiControlNoLimit,	\
}

#define PID_PI_CONTROL_MACRO( Command, Feedback, p )			\
	p->Error = Command - Feedback;								\
	\
	p->Up = p->Kp * p->Error;									\
	\
	p->Ui += ( p->Ki * p->Error * p->Period );					\
	p->Ui = ( p->Ui > p->UpperLimit ) ? p->UpperLimit : p->Ui;	\
	p->Ui = ( p->Ui < p->LowerLimit ) ? p->LowerLimit : p->Ui;	\
	\
	p->Output = p->Up + p->Ui;									\


#endif /* INC_PID_H_ */
