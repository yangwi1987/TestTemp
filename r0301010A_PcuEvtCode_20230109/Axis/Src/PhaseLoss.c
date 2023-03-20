/*
 * PhaseLoss.c
 *
 *  Created on: 2020年8月1日
 *      Author: Fernando.Wang.HHW
 */

#define PHASE_LOSS_NORMAL 0
#define PHASE_LOSS_ERROR 1

#define TOTAL_PHASE_CURRENT_SQUARE 1600.0f // at least one phase current is (40 Arms)^2
#define LEAST_SINGLE_PHASE_CURRENT_SQUARE 0.81f // at least one phase current is (0.9 Arms)^2
#define MEC_SPEED_100_RPM 41.887902f // 100 RPM mechanical speed to electric speed in (rad/sec)
#define MIN_VOLT_IN_PHASE_LOSS 0.67f // the volt correspond to 1 Nm at zero speed.
#define LOSS_PHASE_DURATION 100 // in ms

#include "PhaseLoss.h"
static uint16_t UlossCNT = 0;
static uint16_t VlossCNT = 0;
static uint16_t WlossCNT = 0;

void PhaseLoss_RunTimeDetect( PHASE_LOSS_TYPE *p )
{

	if( p->PLCLoopVcmdAmp > MIN_VOLT_IN_PHASE_LOSS )
	{
		// Detect sum of each phase current below HFI speed
		if( p->PLCLoopElecSpeedAbs < EEMF_START_SPEED )
		{
			if( (p->Avg1KhzCurrSqrU + p->Avg1KhzCurrSqrV + p->Avg1KhzCurrSqrW) < LEAST_SINGLE_PHASE_CURRENT_SQUARE )
			{
				// This condition will occur, when:
				// Two phase lines are disconnected.
				// There is small torque command and one phase line is disconnected.
				p->PhaseLossCnt++;
			}
			else
			{
				// avoid underflow
				if( p->PhaseLossCnt > 0 )
				{
					p->PhaseLossCnt--;
				}
			}

		}
		else
		{
			// avoid underflow
			if( p->PhaseLossCnt > 0 )
			{
				p->PhaseLossCnt--;
			}
		}

		// Detect each phase current when speed is over 100 RPM and torque is greater than 0.066Nm.
		if( (p->PLCLoopElecSpeedAbs >= MEC_SPEED_100_RPM) && \
			(p->PLCLoopTorqueCmd > 0.066f)					  )
		{
			// As rotating, if one phase line is zero current,then phase loss occur.
			if( p->Avg1KhzCurrSqrU < LEAST_SINGLE_PHASE_CURRENT_SQUARE )
			{
				UlossCNT++;
			}
			else
			{
				UlossCNT = 0;
			}

			if( p->Avg1KhzCurrSqrV < LEAST_SINGLE_PHASE_CURRENT_SQUARE )
			{
				VlossCNT++;
			}
			else
			{
				VlossCNT = 0;
			}

			if( p->Avg1KhzCurrSqrW < LEAST_SINGLE_PHASE_CURRENT_SQUARE )
			{
				WlossCNT++;
			}
			else
			{
				WlossCNT = 0;
			}
		}

	}
	else
	{
		//do nothing
	}

	// Register error by each Counter(CNT)
	if( p->PhaseLossCnt > LOSS_PHASE_DURATION ) // Error status keeps for 150 ms, then register alarm.
	{
		p->Error = 1;
		// avoid overflow
		p->PhaseLossCnt = LOSS_PHASE_DURATION;
	}
	if( UlossCNT > LOSS_PHASE_DURATION ) // Error status keeps for 150 ms, then register alarm.
	{
		p->Error = 1;
		// avoid overflow
		UlossCNT = LOSS_PHASE_DURATION;
	}
	if( VlossCNT > LOSS_PHASE_DURATION ) // Error status keeps for 150 ms, then register alarm.
	{
		p->Error = 1;
		// avoid overflow
		VlossCNT = LOSS_PHASE_DURATION;
	}
	if( WlossCNT > LOSS_PHASE_DURATION ) // Error status keeps for 150 ms, then register alarm.
	{
		p->Error = 1;
		// avoid overflow
		WlossCNT = LOSS_PHASE_DURATION;
	}
}
void PhaseLoss_RunTimeClean( PHASE_LOSS_TYPE *p )
{
	p->Avg1KhzCurrSqrU = 0.0f;
	p->Avg1KhzCurrSqrV = 0.0f;
	p->Avg1KhzCurrSqrW = 0.0f;

	p->Acc10KhzCurrSqrU = 0.0f;
	p->Acc10KhzCurrSqrV = 0.0f;
	p->Acc10KhzCurrSqrW = 0.0f;
}
void PhaseLoss_Clean( PHASE_LOSS_TYPE *p )
{
	p->State = TEST_HALL_OUT_UHVL;
	p->Timer = 0;
	p->Start = FUNCTION_NO;
	p->IuAbsMax = 0.0f;
	p->IvAbsMax = 0.0f;
	p->IwAbsMax = 0.0f;
}

uint16_t PhaseLoss_Init( PHASE_LOSS_TYPE *p, float CurrLevel, float CurrCmd, float ExeTime, float Period )
{
	int32_t ExeTimeTmp = 0;
	p->Enable = FUNCTION_ENABLE;
	PhaseLoss_Clean( p );
	if( CurrLevel < 0.0f )
	{
		p->CurrLevel = 0.0f;
		p->Enable = FUNCTION_DISABLE;
		return PHASE_LOSS_INIT_ERROR_CURR_LEVEL;
	}
	else
	{
		p->CurrLevel = CurrLevel;
	}
	if( CurrCmd < 0.0f )
	{
		p->CurrCmd = 0.0f;
		p->Enable = FUNCTION_DISABLE;
		return PHASE_LOSS_INIT_ERROR_CURR_CMD;
	}
	else
	{
		p->CurrCmd = CurrCmd;
	}
	ExeTimeTmp = (int32_t)(ExeTime / Period);
	if( ExeTimeTmp <= 0 )
	{
		p->ExeCnt = 0;
		p->Enable = FUNCTION_DISABLE;
		return PHASE_LOSS_INIT_ERROR_EXE_TIME;
	}
	else if( ExeTimeTmp > 65535 )
	{
		p->ExeCnt = 65535;
		p->Enable = FUNCTION_DISABLE;
		return PHASE_LOSS_INIT_ERROR_EXE_TIME;
	}
	else
	{
		p->ExeCnt = ExeTimeTmp;
	}
	return PHASE_LOSS_INIT_OK;
}

void PhaseLoss_Detection( PHASE_LOSS_TYPE *p, MOTOR_CONTROL_TYPE *v, int16_t *CtrlMode )
{
	if( p->Start == FUNCTION_YES )
	{
		float IuAbs = (v->SensorFb.Iu >= 0.0f) ? v->SensorFb.Iu : -v->SensorFb.Iu;
		float IvAbs = (v->SensorFb.Iv >= 0.0f) ? v->SensorFb.Iv : -v->SensorFb.Iv;
		float IwAbs = (v->SensorFb.Iw >= 0.0f) ? v->SensorFb.Iw : -v->SensorFb.Iw;
		p->IuAbsMax = (p->IuAbsMax > IuAbs) ? p->IuAbsMax : IuAbs;
		p->IvAbsMax = (p->IvAbsMax > IvAbs) ? p->IvAbsMax : IvAbs;
		p->IwAbsMax = (p->IwAbsMax > IwAbs) ? p->IwAbsMax : IwAbs;

		*CtrlMode = FUNCTION_MODE_SIX_WAVE_120_CLOSE_LOOP;
		v->Cmd.SixWaveCurrCmd = p->CurrCmd;
		v->SensorFb.TestHallSignal = p->State;
		v->HallSelect = SIX_WAVE_HALL_TYPE_TEST;
		p->Timer++;
		if( p->Timer == 1 )
		{
			v->SixWave120CurrentControl.Clean( &(v->SixWave120CurrentControl) );
		}
		else if( p->Timer >= p->ExeCnt )
		{
			switch( p->State )
			{
				case TEST_HALL_OUT_UHVL:
					{
					p->Error = (p->IuAbsMax < p->CurrLevel) ? PHASE_LOSS_ERROR_HAPPEN : 0;
					if( p->Error == 1 )
					{
						PhaseLoss_Clean( p );
						p->Enable = FUNCTION_DISABLE;
					}
					else
					{
						p->State = TEST_HALL_OUT_VHWL;
						p->Timer = 0;
					}
					break;
				}
				case TEST_HALL_OUT_VHWL:
					{
					p->Error = (p->IvAbsMax < p->CurrLevel) ? PHASE_LOSS_ERROR_HAPPEN : p->Error;
					if( p->Error == 1 )
					{
						PhaseLoss_Clean( p );
						p->Enable = FUNCTION_DISABLE;
					}
					else
					{
						p->State = TEST_HALL_OUT_WHUL;
						p->Timer = 0;
					}
					break;
				}
				case TEST_HALL_OUT_WHUL:
					{
					p->Error = (p->IwAbsMax < p->CurrLevel) ? PHASE_LOSS_ERROR_HAPPEN : p->Error;
					PhaseLoss_Clean( p );
					p->Enable = FUNCTION_DISABLE;
					break;
				}
			}
		}
	}
	else // p->Start == FUNCTION_NO
	{
		PhaseLoss_Clean( p );
		p->Enable = FUNCTION_DISABLE;
	}
}
