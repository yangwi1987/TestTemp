/*
 * BMEApp.c
 *
 *  Created on: 20230714
 *      Author: Will
 */


#include "BMEApp.h"



AcPwrInfo_t AcPwrInfo = AC_POWER_INFO_DEFAULT;


void AcPowerInfo_Reset(AcPwrInfo_t *p)
{
	p->AvgPower = 0;
	p->InstPower = 0;
	p->InstPwrSum = 0;
	p->TotalPower = 0;
	p->LogCnt = 0;
	p->TotalLogCnt = 0;
}

void AcPowerInfo_do100HzLoop(AcPwrInfo_t *p, float PowerIn)
{
	p->InstPwrSum += PowerIn;
	p->LogCnt++;

	if(p->LogCnt >= INSTANT_POWER_LOG_SIZE)
	{
		p->InstPower = p->InstPwrSum * INSTANT_POWER_LOG_SIZE_INVERSE;			/* get "averaged" instantaneous power */
		p->TotalPower += p->InstPower;											/* J= sum ( P * time interval) */
		p->TotalLogCnt++;

		if(p->TotalLogCnt > 0)
		{
			p->AvgPower = p->TotalPower / (float)p->TotalLogCnt;				/* get the average power */											/* Load value */
		}
		p->LogCnt = 0;															/* clear log counter */
		p->InstPwrSum = 0;														/* clear the accumlate value */
	}
}


