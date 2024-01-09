/*
 * BMEApp.c
 *
 *  Created on: 20230714
 *      Author: Will
 */


#include "E10App.h"

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

/*=============== Button handle start ===============*/
static BtnStateFb_t BtnTable[BTN_IDX_MAX];

void Btn_Reset (BtnIdx_t Idx)
{
	BtnTable[Idx].Before = BTN_STATE_INITIALIZING;
	BtnTable[Idx].Now = BTN_STATE_INITIALIZING;
	BtnTable[Idx].TimeCnt = 0;
	BtnTable[Idx].EvtRecord = BTN_EVENT_INITIALIZING;
}

void Btn_Handle(BtnIdx_t Idx)
{
	int8_t i8Temp;

	if(BtnTable[Idx].EvtRecord == BTN_EVENT_INITIALIZING)
	{
		if(BtnTable[Idx].TimeCnt++ > 10)
		{
			BtnTable[Idx].TimeCnt = 0;
			BtnTable[Idx].Before = BtnTable[Idx].Now;
			BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
		}
	}
	else
	{
		i8Temp = BtnTable[Idx].Now - BtnTable[Idx].Before;

		if(i8Temp != 0) /*signal change is detected */
		{
			BtnTable[Idx].TimeCnt++;

			if(BtnTable[Idx].TimeCnt > 10)
			{
				BtnTable[Idx].Before = BtnTable[Idx].Now;
				BtnTable[Idx].EvtRecord = (i8Temp > 0) ? BTN_EVENT_RISING : BTN_EVENT_FALLING;
				BtnTable[Idx].TimeCnt = 0;
			}
		}
		else
		{
			BtnTable[Idx].TimeCnt = 0;
		}
	}

}


/*
 * Read the latest event detected by IO handler
 */
BtnEvent_t Btn_EvtRead(BtnIdx_t Idx)
{
	BtnEvent_t ret;
	ret = BtnTable[Idx].EvtRecord;
	BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
	return ret;
}

/*
 * Read the handled state of signal
 */
BtnState_t Btn_StateRead(BtnIdx_t Idx)
{
	return BtnTable[Idx].Before;
}

/*
 * write the Digital value detected by MCU into IO handler anywhere
 */
void Btn_SignalWrite(BtnIdx_t Idx, int8_t DataIn)
{
	BtnTable[Idx].Now = DataIn;
}

/*
 * Btn period loop
 */
void Btn_Do100HzLoop()
{
	for(uint8_t i=0 ; i< BTN_IDX_MAX; i++)
	{
		Btn_Handle(i);
	}
}

void Btn_Init()
{
	for(uint8_t i=0 ; i< BTN_IDX_MAX; i++)
	{
		Btn_Reset(i);
	}
}

/*=============== Button handle End ===============*/
