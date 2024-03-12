/*
 * BMEApp.c
 *
 *  Created on: 20230714
 *      Author: Will
 */


#include "E10App.h"
#include "main.h"


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
    BtnTable[Idx].Now = 0;
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

/*=============== Button handle End ===============*/





uint16_t A1333_DataRead(SPI_HandleTypeDef *hspi, uint8_t AddrIn )
{
	A1333SpCmdUnit_t CmdTemp;
	HAL_StatusTypeDef Ret;
	uint16_t DummyCmd = 0;
	uint16_t ret = 0;
	CmdTemp.Access = A1333_ACCESS_READ;
	CmdTemp.Addr = (0x3F & AddrIn);
	CmdTemp.Data = 0x00;

//	CmdArray[0] = 0x11;
//	CmdArray[1] = 0x22;
//	CmdArray[2] = 0x33;
//	CmdArray[3] = 0x44;

	HAL_GPIO_WritePin(MSPI_CS_GPIO_Port, MSPI_CS_Pin, GPIO_PIN_RESET);
	Ret = HAL_SPI_Transmit(hspi, (uint8_t*)&CmdTemp, 1, 100);

	if(Ret != HAL_OK)
	{
		HAL_SPI_Abort(hspi);
	}

	HAL_SPI_TransmitReceive(hspi, (uint8_t*)&DummyCmd, (uint8_t*)&ret, 1, 100);
	HAL_GPIO_WritePin(MSPI_CS_GPIO_Port, MSPI_CS_Pin, GPIO_PIN_SET);

	return ret;
}


