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

typedef uint16_t (*pPiComplexVectorInit)( float, float, float, float, uint8_t, void* );
typedef void (*pPiComplexVectorClean)( void* );

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

typedef struct
{
	float Error;
	float Errorj;
	float Period;
	float Kp;
	float Ki;
	float Kij;
	float Up;
	float Uij;
	float Output_Raw;
	float Output;
	float Limit;
	uint8_t UseAntiWindup;
	pPiComplexVectorInit	Init;
	pPiComplexVectorClean Clean;
} PI_COMPLEX_VECTOR_TYPE;

uint16_t Pid_PiInit( float Period, float Kp, float Ki, float UpperLimit, float LowerLimit, PI_TYPE* p );
void Pid_PiClean( PI_TYPE* p );
void Pid_PiControl( float Command, float Feedback, PI_TYPE* p );
void Pid_PiControlNoLimit( float Command, float Feedback, PI_TYPE* p );

uint16_t Pid_PiComplexVectorInit( float Period, float Kp, float Ki, float Kij, uint8_t UseAntiWindup, PI_COMPLEX_VECTOR_TYPE* p );
void Pid_PiComplexVectorClean( PI_COMPLEX_VECTOR_TYPE* p );


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

#define PI_COMPLEX_VECTOR_DEFAULT 	\
{ 					\
	0.0f,			/*float Error;     */\
	0.0f,			/*float Errorj;     */\
	0.0f,			/*float Period;    */\
    0.0f,			/*float Kp;        */\
	0.0f,			/*float Ki;        */\
	0.0f,			/*float Kij;       */\
	0.0f,			/*float Up;        */\
	0.0f,			/*float Uij;       */\
	0.0f,			/*float Output_Raw;*/\
	0.0f,			/*float Output;    */\
	0.0f,			/*float Limit;     */\
	0,/*UseAntiWindup*/\
	(pPiComplexVectorInit)Pid_PiComplexVectorInit,		\
	(pPiComplexVectorClean)Pid_PiComplexVectorClean,	\
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


__STATIC_FORCEINLINE void Pid_PiComplexVectorControl( float error, float errorj, float VsSaturation, PI_COMPLEX_VECTOR_TYPE* p )
{
	p->Error = error;
	p->Errorj = errorj;

    p->Up = p->Kp * p->Error;

	if ( p->UseAntiWindup == 1)
	{
	    float Uij = 0.0f;
	    float Uijtemp = 0.0f;

	    Uijtemp = p->Ki * p->Error + p->Kij * p->Errorj;
        Uij = p->Uij + ( Uijtemp * p->Period);

        p->Output_Raw = p->Up + Uij;

        if (( p->Output_Raw > VsSaturation ))
        {
        	p->Output_Raw = VsSaturation;
        	if ( Uijtemp < 0.0f )
        	{
        		p->Uij = Uij;
        	}
        }
        else if (( p->Output_Raw < -VsSaturation ))
        {
        	p->Output_Raw = -VsSaturation;
        	if ( Uijtemp > 0.0f )
        	{
        		p->Uij = Uij;
        	}
        }
        else
        {
        	p->Uij = Uij;
        }

	}
	else
	{
		p->Uij += (( p->Ki * p->Error + p->Kij * p->Errorj ) * p->Period );

		p->Output_Raw = p->Up + p->Uij;
	}

    p->Output = p->Output_Raw;
}

#endif /* INC_PID_H_ */
