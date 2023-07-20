/*
 * BMEApp.c
 *
 *  Created on: 2023年7月14日
 *      Author: Will
 */


#include "BMEApp.h"

void AcPowerInfo_Reset(AcPwrInfo_t *p);
void AcPowerInfo_100HzLoop(AcPwrInfo_t *p, float PowerIn);

AcPwrInfo_t AcPwrInfo = AC_POWER_INFO_DEFAULT;


void AcPowerInfo_Reset(AcPwrInfo_t *p)
{
	memset(p, 0, sizeof(AcPwrInfo_t));
}

void AcPowerInfo_100HzLoop(AcPwrInfo_t *p, float PowerIn)
{
	p->InstPwrSum += PowerIn;
	p->LogCnt++;

	if(p->LogCnt >= INSTANT_POWER_LOG_SIZE)
	{
		p->InstPower = p->InstPwrSum / INSTANT_POWER_LOG_SIZE;					/* get "averaged" instantaneous power */
		p->TotalJoule += p->InstPower;											/* J= sum ( P * time interval) */
		p->TotalLogCnt++;

		if(p->TotalLogCnt > 0)
		{
			p->AvgPower = p->TotalJoule / (float)p->TotalLogCnt;				/* get the average power */											/* Load value */
		}
		p->LogCnt = 0;															/* clear log counter */
		p->InstPwrSum = 0;														/* clear the accumlate value */
	}
}


