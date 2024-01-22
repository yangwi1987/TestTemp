/*
 * GeneratePwmDuty.h
 *
 *  Created on: 2019年12月3日
 *      Author: Fernando
 */

#ifndef INC_GENERATEPWMDUTY_H_
#define INC_GENERATEPWMDUTY_H_

enum DUTY_COMMAND_INIT_STATUS_ENUM
{
	 DUTY_COMMAND_INIT_OK = 0,
	 DUTY_COMMAND_INIT_ERROR_PERIOD_IS_NOT_POSITIVE = 0x8001,
	 DUTY_COMMAND_INIT_ERROR_DEADTIME_IS_NOT_POSITIVE,
	 DUTY_COMMAND_INIT_ERROR_MINTIME_IS_NEGATIVE,
	 DUTY_COMMAND_INIT_ERROR_DEADTIME_IS_LARGER_PERIOD,
	 DUTY_COMMAND_INIT_ERROR_MINTIME_IS_LARGER_PERIOD,
	 DUTY_COMMAND_INIT_ERROR_CALC_RESULT_IS_OVER_RAGNE
};

enum SIX_WAVE_120_SECTION_ENUM
{
	 VHWL = 0,
	 VHUL,
	 WHUL,
	 WHVL,
	 UHVL,
	 UHWL,
};

enum SIX_WAVE_TYPE_ENUM
{
	 SIX_WAVE_120 = 0,
	 SIX_WAVE_180,
};

enum SIX_WAVE_TORQUE_POLARITY_ENUM
{
	 SIX_WAVE_TORQUE_POLARITY_POSITIVE = 0,
	 SIX_WAVE_TORQUE_POLARITY_NEGATIVE,
};

typedef enum
{
	SVPWM = 0,
	DPWM_H,
	DPWM_L,
	DPWM_ANGLE_SGIFT
}DPWM_SELECT_e;

typedef void (*pDutyCommand120Degree)( float, void*, uint16_t );
typedef uint16_t (*pDutyCommandInit)( float, float, float, void*);
typedef void (*pDutyCommandClean)( void* );
typedef void (*pSvpwmCalc)( float, float, float, void*);
typedef void (*pSixWave120Calc)( void*, float );
typedef void (*pfunGeneratePwmDuty_CompensateDeadtimeByPhase)( void*, float, float, float );
typedef void (*pfunGeneratePwmDuty_CompensateDeadtimeByPhaseInit)( void*, float, float, float, float);

typedef struct
{
	uint16_t PwmMode;
	float Duty[3];
} DUTY_TYPE;

typedef struct
{
	DUTY_TYPE CompDuty;
	float DeadtimeDutyP;
	float CornerCurrP;
	float SlopeP;
	float DeadtimeDutyN;
	float CornerCurrN;
	float SlopeN;
	float StartCmdSquare;
	pfunGeneratePwmDuty_CompensateDeadtimeByPhaseInit Init;
	pfunGeneratePwmDuty_CompensateDeadtimeByPhase Compensation;
} COMPENSATION_DEADTIME_TYPE;

typedef struct
{
	DUTY_TYPE DutyLimitation;
	float MaxDuty;
	float MinDuty;
	pDutyCommand120Degree Set120;
	pDutyCommandInit Init;
	pDutyCommandClean Clean;
} DUTY_COMMAND_TYPE;

typedef struct
{
	DUTY_TYPE Duty;
	DPWM_SELECT_e DPWM_Select;
} SVPWM_TYPE;

typedef struct
{
	uint16_t TorquePolarityFlag : 1;
	uint16_t ReversedFlag2 : 1;
	uint16_t ReversedFlag3 : 1;
	uint16_t ReversedFlag4 : 1;
	uint16_t ReversedFlag5 : 1;
	uint16_t ReversedFlag6 : 1;
	uint16_t ReversedFlag7 : 1;
	uint16_t ReversedFlag8 : 1;
	uint16_t ReversedFlag9 : 1;
	uint16_t ReversedFlag10 : 1;
	uint16_t ReversedFlag11 : 1;
	uint16_t ReversedFlag12 : 1;
	uint16_t ReversedFlag13 : 1;
	uint16_t ReversedFlag14 : 1;
	uint16_t ReversedFlag15 : 1;
	uint16_t ReversedFlag16 : 1;
	float DutyCmd;
	int16_t PwmStatus;
	int16_t HallSection;
	float VoltCmd;
	pSixWave120Calc Calc;
} SIX_WAVE_120_DUTY_TYPE;

void GeneratePwmDuty_DutyCommand120Degree( float DutyCmd, DUTY_COMMAND_TYPE* p2, uint16_t Section);
uint16_t GeneratePwmDuty_DutyCommandInit( float PwmPeriod, float DeadTime, float MinTime, DUTY_COMMAND_TYPE* p);
void GeneratePwmDuty_DutyCommandClean( DUTY_COMMAND_TYPE* p );
void GeneratePwmDuty_SixWave120Calc( SIX_WAVE_120_DUTY_TYPE* p, float DivideVbus );
void GeneratePwmDuty_CompensateDeadtimeByPhaseInit( COMPENSATION_DEADTIME_TYPE* p, float DeadtimeDutyP, float CornerCurrP, float DeadtimeDutyN, float CornerCurrN);
void GeneratePwmDuty_CompensateDeadtimeByPhase( COMPENSATION_DEADTIME_TYPE* p, float Iu, float Iv, float Iw);


#define DUTY_DEFAULT	\
{						\
	0,					\
	{0.0f,0.0f,0.0f}	\
}

#define DUTY_COMMAND_DEFAULT 			\
{										\
	DUTY_DEFAULT,						\
	1.0f,								\
	0.0f,								\
	(pDutyCommand120Degree)GeneratePwmDuty_DutyCommand120Degree,	\
	(pDutyCommandInit)GeneratePwmDuty_DutyCommandInit,				\
	(pDutyCommandClean)GeneratePwmDuty_DutyCommandClean,			\
}

#define SVPWM_DEFAULT 			\
{								\
	DUTY_DEFAULT,				\
	SVPWM,                      \
}

#define SIX_WAVE_120_DUTY_DEFAULT	\
{			\
	1,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0,		\
	0.0f,	\
	UHVL,	\
	270,	\
	0.0f,	\
	(pSixWave120Calc) GeneratePwmDuty_SixWave120Calc,	\
}

#define COMPENSATION_DEADTIME_DEFAULT \
{						\
	DUTY_DEFAULT,		\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	0.0f,				\
	(pfunGeneratePwmDuty_CompensateDeadtimeByPhaseInit) GeneratePwmDuty_CompensateDeadtimeByPhaseInit,	\
	(pfunGeneratePwmDuty_CompensateDeadtimeByPhase) GeneratePwmDuty_CompensateDeadtimeByPhase,	\
}

__STATIC_FORCEINLINE void GENERATE_PWM_DUTY_DUTY_COOMMAND_180DEGREE( DUTY_TYPE* p1, DUTY_COMMAND_TYPE* p2 )				\
{
	p2->DutyLimitation.Duty[PHASE_U] = LIMIT(p1->Duty[PHASE_U], p2->MinDuty, p2->MaxDuty);
	p2->DutyLimitation.Duty[PHASE_V] = LIMIT(p1->Duty[PHASE_V], p2->MinDuty, p2->MaxDuty);
	p2->DutyLimitation.Duty[PHASE_W] = LIMIT(p1->Duty[PHASE_W], p2->MinDuty, p2->MaxDuty);
	p2->DutyLimitation.PwmMode = PWM_COMPLEMENTARY;						\
}

__STATIC_FORCEINLINE void GENERATE_PWM_DUTY_SVPWM_INLINE( float Alpha, float Beta, float DevideVbus, SVPWM_TYPE* p)
{
	int16_t Section = 1;												\
	float U = 0.0f;														\
	float V = 0.0f;														\
	float W = 0.0f;														\
	float Sqrt3Alpha;													\
	\
	Sqrt3Alpha = 1.732050808f * Alpha;									\
	\
	Section = ( Beta > Sqrt3Alpha ) ? ( Section + 1 ) : ( Section );	\
	Section = ( Beta < -Sqrt3Alpha ) ? ( Section + 1 ) : ( Section );	\
	Section = ( Beta > 0.0f ) ? ( Section ) : ( 7 - Section );			\
	\
	switch (Section)													\
	{																	\
		case 1:															\
		case 4:															\
		{																\
			U = 0.750000000f * Alpha + 0.433012701f * Beta;				\
			V = -0.750000000f * Alpha + 1.299038106f * Beta;			\
			W = -0.750000000f * Alpha - 0.433012701f * Beta;			\
			break;														\
		}																\
		case 2:															\
		case 5:															\
		{																\
			U = 1.500000000f * Alpha;									\
			V = 0.866025403f * Beta;									\
			W = -0.866025403f * Beta;									\
			break;														\
		}																\
		case 3:															\
		case 6:															\
		{																\
			U = 0.750000000f * Alpha - 0.433012701f * Beta;				\
			V = -0.750000000f * Alpha + 0.433012701f * Beta;			\
			W = -0.750000000f * Alpha - 1.299038106f * Beta;			\
			break;														\
		}																\
		default :														\
		{																\
			U = 0.0f;													\
			V = 0.0f;													\
			W = 0.0f;													\
		}																\
	}																	\
	\
	p->Duty.Duty[PHASE_U] = 0.5f + DevideVbus * U;						\
	p->Duty.Duty[PHASE_V] = 0.5f + DevideVbus * V;						\
	p->Duty.Duty[PHASE_W] = 0.5f + DevideVbus * W;						\
}

__STATIC_FORCEINLINE void GENERATE_PWM_DUTY_SVPWM_Calc( float Alpha, float Beta, float DevideVbus, SVPWM_TYPE* p)
{
	float U = 0.0f;
	float V = 0.0f;
	float W = 0.0f;
	float Max = 0.0f;
	float Min = 0.0f;

	U = Alpha;
	V = -0.5f * Alpha + 0.866025403f * Beta;
	W = -0.5f * Alpha - 0.866025403f * Beta;

    if ( U > V)
    {
    	Max = U;
    	Min = V;
    }
    else
    {
    	Max = V;
    	Min = U;
    }
    if ( W > Max )
    {
    	Max = W;
    }
    else if ( W < Min )
    {
    	Min = W;
    }

    if ( p->DPWM_Select == SVPWM )
    {
    	float ZeroSequence = 0.0f;
        ZeroSequence = 0.5f * ( Max + Min );
    	p->Duty.Duty[PHASE_U] = 0.5f + DevideVbus * ( U - ZeroSequence );
    	p->Duty.Duty[PHASE_V] = 0.5f + DevideVbus * ( V - ZeroSequence );
    	p->Duty.Duty[PHASE_W] = 0.5f + DevideVbus * ( W - ZeroSequence );
    }
    else if ( p->DPWM_Select == DPWM_H )
	{
    	p->Duty.Duty[PHASE_U] = 1.0f + DevideVbus * ( U - Max );
    	p->Duty.Duty[PHASE_V] = 1.0f + DevideVbus * ( V - Max );
    	p->Duty.Duty[PHASE_W] = 1.0f + DevideVbus * ( W - Max );
	}
    else if ( p->DPWM_Select == DPWM_L )
    {
    	p->Duty.Duty[PHASE_U] = DevideVbus * ( U - Min );
    	p->Duty.Duty[PHASE_V] = DevideVbus * ( V - Min );
    	p->Duty.Duty[PHASE_W] = DevideVbus * ( W - Min );
    }
}

#endif /* INC_GENERATEPWMDUTY_H_ */
