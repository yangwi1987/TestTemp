/*
 * BMEApp.c
 *
 *  Created on: 20230714
 *      Author: Will
 */


#include "E10App.h"
#include "ExtranetCANStation.h"

extern ExtranetCANStation_t ExtranetCANStation;
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

void Btn_Reset (BtnIdx_e Idx)
{
    BtnTable[Idx].Before = BTN_STATE_INITIALIZING;
    BtnTable[Idx].Now = BTN_STATE_INITIALIZING;
    BtnTable[Idx].SignalTimeCnt = 0;
    BtnTable[Idx].EvtRecord = BTN_EVENT_INITIALIZING;
}

/*
 * Put this handling function in 100Hz timer loop
 * */
void Btn_Handle (BtnIdx_e Idx)
{
    int8_t i8Temp;

    if (BtnTable[Idx].EvtRecord == BTN_EVENT_INITIALIZING)
    {
        if(BtnTable[Idx].SignalTimeCnt++ > BTN_INITIAL_TIME_COUNT)
        {
            BtnTable[Idx].SignalTimeCnt = 0;
            BtnTable[Idx].Before = BtnTable[Idx].Now;
            BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
        }
        else
        {
            BtnTable[Idx].Before = BTN_STATE_INITIALIZING;
            BtnTable[Idx].EvtRecord = BTN_EVENT_INITIALIZING;
        }
    }
    else
    {
        i8Temp = BtnTable[Idx].Now - BtnTable[Idx].Before;

        if(i8Temp != 0) /*signal change is detected */
        {
            BtnTable[Idx].SignalTimeCnt++;

            if (BtnTable[Idx].SignalTimeCnt > BTN_DEBOUNCE_TIME_COUNT)
            {
                BtnTable[Idx].Before = BtnTable[Idx].Now;
                BtnTable[Idx].EvtRecord = (i8Temp > 0) ? BTN_EVENT_RISING : BTN_EVENT_FALLING;
                BtnTable[Idx].SignalTimeCnt = 0;
                BtnTable[Idx].EvtTimeCnt = 0;
            }
        }
        else
        {
            BtnTable[Idx].SignalTimeCnt = 0;
            
            if ((BtnTable[Idx].EvtRecord == BTN_EVENT_RISING) ||
            	(BtnTable[Idx].EvtRecord == BTN_EVENT_FALLING))
            {
                BtnTable[Idx].EvtTimeCnt ++;

                if(BtnTable[Idx].EvtTimeCnt > BTN_RECORDED_EVENT_AUTOCLEAR_TIME_THRESHOLD)
                {
                    BtnTable[Idx].EvtTimeCnt = 0;
                    BtnTable[Idx].EvtRecord = BTN_EVENT_IDLE;
                }
            }
        }
    }
}

/*
 * Read the latest event detected by IO handler
 */
BtnEvent_e Btn_EvtRead (BtnIdx_e Idx)
{
    BtnEvent_e ret;
    ret = BtnTable[Idx].EvtRecord;
    BtnTable[Idx].EvtRecord = (BtnTable[Idx].EvtRecord ==BTN_EVENT_INITIALIZING) ? BTN_EVENT_INITIALIZING : BTN_EVENT_IDLE;
    return ret;
}

/*
 * Read the handled state of signal
 */
BtnState_e Btn_StateRead (BtnIdx_e Idx)
{
    return BtnTable[Idx].Before;
}

/*
 * write the Digital value detected by MCU into IO handler anywhere
 */
void Btn_SignalWrite (BtnIdx_e Idx, int8_t DataIn)
{
    BtnTable[Idx].Now = DataIn;
}

/*
 * Btn period loop
 */
void Btn_Do100HzLoop (void)
{
    for(uint8_t i = 0 ; i < BTN_IDX_MAX; i++)
    {
        Btn_Handle(i);
    }
}

void Btn_Init()
{
    for(uint8_t i = 0 ; i < BTN_IDX_MAX; i++)
    {
        Btn_Reset(i);
    }
}



void Btn_Test01In100HzLoop ( void )
{
	static uint16_t PeriodCnt = 0;
	static uint16_t DutyTarget = 0;
	BtnState_e StateTemp = 0;
	BtnEvent_e EvtTemp = 0;
	uint8_t signalTemp = 0;
	STRUCT_CAN_DATA CanTemp;

	if(DutyTarget < 30)
	{
		if(PeriodCnt <= DutyTarget)
		{
			signalTemp = 1;
		}
		else
		{
			signalTemp = 0;

		}
		Btn_SignalWrite(BTN_IDX_KILL_SW, signalTemp);
		StateTemp = Btn_StateRead(BTN_IDX_KILL_SW);
		EvtTemp =Btn_EvtRead(BTN_IDX_KILL_SW);
		PeriodCnt++;

		CanTemp.ID.All = 0;
		CanTemp.Size = 3;
		CanTemp.ID.StdFrame.StdID = 0x788;
		CanTemp.Data[0] =signalTemp;
		CanTemp.Data[1] =StateTemp;
		CanTemp.Data[2] =EvtTemp;
		CanTemp.Data[3] = 0;
		CanTemp.Data[4] = 0;
		CanTemp.Data[5] = 0;
		CanTemp.Data[6] = 0;
		CanTemp.Data[7] = 0;

		ExtranetCANStation.TxQ.EnQ(&ExtranetCANStation.TxQ, &CanTemp);

		if(PeriodCnt == 50)
		{
			PeriodCnt = 0;
			DutyTarget++;
		}
	}
}

void Btn_Test02In100HzLoop ( void )
{
	static uint16_t PeriodCnt = 0;
	BtnState_e StateTemp = 0;
	BtnEvent_e EvtTemp = 0;
	uint8_t signalTemp = 0;
	STRUCT_CAN_DATA CanTemp;

	if(PeriodCnt < 1000)
	{

		if(400 < PeriodCnt && PeriodCnt < 600)
		{
			signalTemp = 1;
		}
		else
		{
			signalTemp = 0;
		}

		Btn_SignalWrite(BTN_IDX_KILL_SW, signalTemp);
		StateTemp = Btn_StateRead(BTN_IDX_KILL_SW);
		EvtTemp =BtnTable[BTN_IDX_KILL_SW].EvtRecord;

		CanTemp.ID.All = 0;
		CanTemp.Size = 3;
		CanTemp.ID.StdFrame.StdID = 0x788;
		CanTemp.Data[0] =signalTemp;
		CanTemp.Data[1] =StateTemp;
		CanTemp.Data[2] =EvtTemp;
		CanTemp.Data[3] = 0;
		CanTemp.Data[4] = 0;
		CanTemp.Data[5] = 0;
		CanTemp.Data[6] = 0;
		CanTemp.Data[7] = 0;

		ExtranetCANStation.TxQ.EnQ(&ExtranetCANStation.TxQ, &CanTemp);

		PeriodCnt++;
	}
}

/*=============== Button handle End ===============*/
