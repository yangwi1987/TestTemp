/*
 * AngleObserver.c
 *
 *  Created on: 2022年1月10日
 *      Author: Fernando.Wang.HHW
 */

#include "AngleObserver.h"

void AngleObserver_Clean( AngleObserver_t *p, float AngleNow, float SpeedNow, float AcelNow )
{
	p->L1Result = 0.0f;
	p->L2Result = 0.0f;
	p->L3Result = 0.0f;
	p->AcelTmp = AcelNow;
	p->Acel = AcelNow;
	p->SpeedTmp = SpeedNow;
	p->Speed = SpeedNow;
	p->Angle = AngleNow;
	p->AngleError = 0.0f;
	p->CorrespondingAngleError = 0.0f;
}

uint16_t AngleObserver_Init( AngleObserver_t *p, AngleObserverInitParm_t *pSetting )
{
	p->Error = ANGLE_OBSERVER_INIT_OK;
	AngleObserver_Clean(p,0.0f,0.0f,0.0f);
	p->Period = pSetting->Period;
	p->L1 = pSetting->L1;
	p->L2 = pSetting->L2;
	p->L3 = pSetting->L3;
	if( pSetting->J <= 0.0f )
	{
		p->J = 0.0f;
		p->DivideJ = 0.0f;
		p->Error = ANGLE_OBSERVER_ERROR_J;
	}
	else
	{
		p->J = pSetting->J;
		p->DivideJ = 1.0f / p->J;
	}
	if( pSetting->SpeedLowerLimit > pSetting->SpeedUpperLimit )
	{
		p->SpeedUpperLimit = 0.0f;
		p->SpeedLowerLimit = 0.0f;
		p->Error = ANGLE_OBSERVER_ERROR_SPEED_LIMIT;
	}
	else
	{
		p->SpeedUpperLimit = pSetting->SpeedUpperLimit;
		p->SpeedLowerLimit = pSetting->SpeedLowerLimit;
	}
	if( pSetting->AcelLowerLimit > pSetting->AcelUpperLimit )
	{
		p->AcelUpperLimit = 0.0f;
		p->AcelLowerLimit = 0.0f;
		p->Error = ANGLE_OBSERVER_ERROR_ACEL_LIMIT;
	}
	else
	{
		p->AcelUpperLimit = pSetting->AcelUpperLimit;
		p->AcelLowerLimit = pSetting->AcelLowerLimit;
	}
	return p->Error;
}

void AngleObserver_Calc( AngleObserver_t *p, float AngleIn )
{
	// It's replaced by ANGLE_OBSERVER_CALC_MACRO
}

