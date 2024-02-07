/*
 * GeneratePwmDuty.c
 *
 *  Created on: 2019年12月3日
 *      Author: Fernando
 */

#include <MotorControl.h>
const int16_t SixWave120PWMStatus[6]={ WHVL, UHVL, UHWL, VHWL, VHUL, WHUL };

void GeneratePwmDuty_DutyCommand120Degree( float DutyCmd, DUTY_COMMAND_TYPE* p2, uint16_t PwmStatus)
{
	float LimitationDuty = 0;
	if ( DutyCmd > p2->MaxDuty )
	{
		LimitationDuty = p2->MaxDuty;
	}
	else if ( DutyCmd < p2->MinDuty )
	{
		LimitationDuty = p2->MinDuty;
	}
	else
	{
		LimitationDuty = DutyCmd;
	}

	switch (PwmStatus)
	{
		case UHVL:
			p2->DutyLimitation.Duty[PHASE_U] = LimitationDuty;
			p2->DutyLimitation.Duty[PHASE_V] = 0;
			p2->DutyLimitation.Duty[PHASE_W] = 0;
			p2->DutyLimitation.PwmMode = PWM_UbVbWx;
			break;

		case VHWL:
			p2->DutyLimitation.Duty[PHASE_U] = 0;
			p2->DutyLimitation.Duty[PHASE_V] = LimitationDuty;
			p2->DutyLimitation.Duty[PHASE_W] = 0;
			p2->DutyLimitation.PwmMode = PWM_UxVbWb;
			break;

		case WHUL:
			p2->DutyLimitation.Duty[PHASE_U] = 0;
			p2->DutyLimitation.Duty[PHASE_V] = 0;
			p2->DutyLimitation.Duty[PHASE_W] = LimitationDuty;
			p2->DutyLimitation.PwmMode = PWM_UbVxWb;
			break;

		case UHWL:
			p2->DutyLimitation.Duty[PHASE_U] = LimitationDuty;
			p2->DutyLimitation.Duty[PHASE_V] = 0;
			p2->DutyLimitation.Duty[PHASE_W] = 0;
			p2->DutyLimitation.PwmMode = PWM_UbVxWb;
			break;

		case VHUL:
			p2->DutyLimitation.Duty[PHASE_U] = 0;
			p2->DutyLimitation.Duty[PHASE_V] = LimitationDuty;
			p2->DutyLimitation.Duty[PHASE_W] = 0;
			p2->DutyLimitation.PwmMode = PWM_UbVbWx;
			break;

		case WHVL:
			p2->DutyLimitation.Duty[PHASE_U] = 0;
			p2->DutyLimitation.Duty[PHASE_V] = 0;
			p2->DutyLimitation.Duty[PHASE_W] = LimitationDuty;
			p2->DutyLimitation.PwmMode = PWM_UxVbWb;
			break;

		default :
			p2->DutyLimitation.Duty[PHASE_U] = 0;
			p2->DutyLimitation.Duty[PHASE_V] = 0;
			p2->DutyLimitation.Duty[PHASE_W] = 0;
			p2->DutyLimitation.PwmMode = PWM_COMPLEMENTARY;
			break;


	}
}

uint16_t GeneratePwmDuty_DutyCommandInit( float PwmPeriod, float DeadTime, float MinTime, DUTY_COMMAND_TYPE* p)
{
	p->Clean(p);
	p->MaxDuty = 1.0f;
	p->MinDuty = 0.0f;

	if ( PwmPeriod <= 0.0f )
	{
		return DUTY_COMMAND_INIT_ERROR_PERIOD_IS_NOT_POSITIVE;
	}
	if ( DeadTime <= 0.0f )
	{
		return DUTY_COMMAND_INIT_ERROR_DEADTIME_IS_NOT_POSITIVE;
	}
	if ( MinTime < 0 )
	{
		return DUTY_COMMAND_INIT_ERROR_MINTIME_IS_NEGATIVE;
	}
	if ( PwmPeriod < DeadTime )
	{
		return DUTY_COMMAND_INIT_ERROR_DEADTIME_IS_LARGER_PERIOD;
	}
	if ( PwmPeriod < MinTime )
	{
		return DUTY_COMMAND_INIT_ERROR_MINTIME_IS_LARGER_PERIOD;
	}

	p->MinDuty = ( MinTime + DeadTime ) / PwmPeriod;
	p->MaxDuty = 1 - p->MinDuty;
	if ( p->MinDuty > 1.0f)
	{
		p->MaxDuty = 1.0f;
		p->MinDuty = 0.0f;
		return DUTY_COMMAND_INIT_ERROR_CALC_RESULT_IS_OVER_RAGNE;
	}
	else
	{
		return DUTY_COMMAND_INIT_OK;
	}
}

void GeneratePwmDuty_DutyCommandClean( DUTY_COMMAND_TYPE* p )
{
	uint16_t CounterTmp = 0;

	for ( CounterTmp = 0; CounterTmp < MOTOR_PHASE; CounterTmp++ )
	{
		p->DutyLimitation.Duty[CounterTmp] = 0.0f;
	}

}

void GeneratePwmDuty_CompensateDeadtimeByPhaseInit( COMPENSATION_DEADTIME_TYPE* p, float DeadtimeDutyP, float CornerCurrP, float DeadtimeDutyN, float CornerCurrN)
{
	p->DeadtimeDutyP = DeadtimeDutyP;
	p->CornerCurrP = CornerCurrP;
	p->SlopeP = p->DeadtimeDutyP / p->CornerCurrP;
	p->DeadtimeDutyN = DeadtimeDutyN;
	p->CornerCurrN = CornerCurrN;
	p->SlopeN = p->DeadtimeDutyN / p->CornerCurrN;
	p->StartCmdSquare = p->CornerCurrP * p->CornerCurrP;
}

void GeneratePwmDuty_SixWave120Calc( SIX_WAVE_120_DUTY_TYPE* p, float DivideVbus )
{
	p->DutyCmd = p->VoltCmd * DivideVbus;
	int16_t PWMSection = p->HallSection;
	if( p->TorquePolarityFlag == SIX_WAVE_TORQUE_POLARITY_POSITIVE )
	{
		PWMSection = p->HallSection + 2;
	}
	else
	{
		PWMSection = p->HallSection - 1;
	}
	PWMSection = ( PWMSection > 6 ) ? PWMSection - 6 : PWMSection;
	PWMSection = ( PWMSection < 1 ) ? PWMSection + 6 : PWMSection;
	p->PwmStatus = SixWave120PWMStatus[ PWMSection - 1 ];
}

void GeneratePwmDuty_CompensateDeadtimeByPhase( COMPENSATION_DEADTIME_TYPE* p, float Iu, float Iv, float Iw)
{
	float PhaseCurr[3]={0.0f,0.0f,0.0f};
	uint16_t Index=0;
	float CurrAbs=0.0f;
	float CurrSign=0.0f;
	float Slope=0.0f;
	float CompDutyTmp=0.0f;
	float CornerCurr=0.0f;
	float DeadtimeDuty=0.0f;

	PhaseCurr[PHASE_U] = Iu;
	PhaseCurr[PHASE_V] = Iv;
	PhaseCurr[PHASE_W] = Iw;


	for (Index=0;Index<3;Index++)
	{
		if ( PhaseCurr[Index] > 0.0f )
		{
			CurrSign = 1.0f;
			Slope = p->SlopeP;
			CornerCurr = p->CornerCurrP;
			DeadtimeDuty = p->DeadtimeDutyP;
		}
		else
		{
			CurrSign = -1.0f;
			Slope = p->SlopeN;
			CornerCurr = p->CornerCurrN;
			DeadtimeDuty = p->DeadtimeDutyN;
		}
		CurrAbs = CurrSign * PhaseCurr[Index];
		if ( CurrAbs >= CornerCurr )
		{
			CompDutyTmp = DeadtimeDuty;
		}
		else
		{
			CompDutyTmp = CurrAbs * Slope;
		}
		p->CompDuty.Duty[Index] = CompDutyTmp * CurrSign;
	}
}









