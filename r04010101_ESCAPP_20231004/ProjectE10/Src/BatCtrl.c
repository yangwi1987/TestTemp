#include <BatCtrl.h>
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "Protocol.h"
#include "Drive.h"
#include "VehicleCtrl.h"

ByteOrderMapping_e BatCommByteOrder = BYTE_ORDER_BIG_ENDIAN;
BatCtrl_t BatCtrl = {0};
BatInfo_t BatInfo[BAT_IDX_NUMBER] = {CAN_INFORM_FORM_BAT_DEFAULT, CAN_INFORM_FORM_BAT_DEFAULT, CAN_INFORM_FORM_BAT_DEFAULT};
BatCmd_t BatCmd = {0};

uint8_t MsgChksmXorBuild(uint8_t *pArrayIn, uint8_t Num)
{
	uint8_t ret = *pArrayIn;

	for( uint8_t i = 1; i < Num; i++ )
	{
		ret ^= *(pArrayIn + i);
	}
	return ret;
}

void Bat_ShutdownReq(void)
{
	BatCmd.shutdownReq = BAT_FLG_ON;
}

void ArrayToSmallEndian(uint8_t *pArrayIn, uint8_t Size)
{
	uint8_t u8Temp ;
	uint8_t IdxStart = 0;
	uint8_t IdxEnd = Size -1;

	while(IdxStart < IdxEnd)
	{
		u8Temp = pArrayIn[IdxStart];
		pArrayIn[IdxStart] = pArrayIn[IdxEnd];
		pArrayIn[IdxEnd] = u8Temp;
		IdxStart++;
		IdxEnd--;
	}
}

void Bat_ArrayToSmallEndian(uint8_t *pArrayIn, uint8_t Size)
{
	// if communication
	if(BatCommByteOrder == BYTE_ORDER_BIG_ENDIAN)
	{
		ArrayToSmallEndian(pArrayIn,Size);
	}
}

BatCanMsgTx_id31E_50_INV_Rvrt_t  BatCanMsgTx_id31E_50_INV = BATCANMSGTX_IDid31E_50_INV_DEFAULT;

uint8_t Bat_Id31E_50_INV_Send(void)
{
    /* request both */
    STRUCT_CAN_DATA CANDataTemp;
    CanIdField_u CanID;
    uint8_t lStatus = QUEUE_OK;
    BatCanMsgTx_id31E_50_INV_Rvrt_t *pTxMsg;
    pTxMsg = (BatCanMsgTx_id31E_50_INV_Rvrt_t*)&CANDataTemp.Data;
    static uint8_t TxCntr = 0;

    CanID.All = 0x31E;
    memset(pTxMsg,0,8);
    pTxMsg->Byte0.Bit.stInvr = Vehicle_MainSMGet();     // Todo: load inverter state from Inveter station
    pTxMsg->Byte0.Bit.flgInvrShdwnRdy = BatCmd.shutdownReq;
    pTxMsg->Byte6.Bit.cntrMsg15 = TxCntr;
    pTxMsg->nrMsgChksXor06 = MsgChksmXorBuild((uint8_t*)pTxMsg + 1, 7);
    Bat_ArrayToSmallEndian((uint8_t*)pTxMsg, 8);
    CANDataTemp.ID.StdFrame.StdID = CanID.All;
    CANDataTemp.ID.StdFrame.XTD = 0;
    CANDataTemp.Size = 8;
    lStatus = BatCtrl.pExCANStation->TxQ.EnQ(&BatCtrl.pExCANStation->TxQ, &CANDataTemp);

    if(TxCntr < 15)
    {
    	TxCntr++;
    }
    else
    {
    	TxCntr = 0;
    }

    return lStatus;
}

void Bat_CanMsgLoad(uint32_t Id, uint8_t *pData)
{
	BatCanMsgRx_u *pBatCanMsgRx;

	pBatCanMsgRx = (BatCanMsgRx_u*)pData;
	Bat_ArrayToSmallEndian(pData, 8);
	uint8_t BattIdx = 0;

	if(MsgChksmXorBuild(pData,8) == 0)
	{
		switch (Id)
		{
			case 0x30C: // From Master battery
			case 0x30D: // From slave battery
				BattIdx = Id - 0x30C;
				BatInfo[BattIdx].MainSM = pBatCanMsgRx->id30C.Byte0.Bit.StBatt;
				BatInfo[BattIdx].Flags.Bit.Alarm = pBatCanMsgRx->id30C.Byte1.bits.flgBattAlarm;
				BatInfo[BattIdx].Flags.Bit.Warn = pBatCanMsgRx->id30C.Byte1.bits.flgBattWarn;
				BatInfo[BattIdx].Flags.Bit.ChrgBsy = pBatCanMsgRx->id30C.Byte0.Bit.flgChrgBsy;
				BatInfo[BattIdx].Flags.Bit.BalBsy = pBatCanMsgRx->id30C.Byte1.bits.flgBalBsy;
				BatInfo[BattIdx].PrchSM = pBatCanMsgRx->id30C.stBattPreChrgSt;
				BatInfo[BattIdx].Flags.Bit.ShutdownReq = pBatCanMsgRx->id30C.Byte1.bits.flgBattShutdownReq;
				break;

			case 0x30F: //Summarized info from Master 
			case 0x318:	//From Slave battery
			case 0x31A:	//From Master battery
				BattIdx = (Id == 0x31A) ? BAT_IDX_MASTER : (Id == 0x318)? BAT_IDX_SLAVE : BAT_IDX_SUMMERY;
				BatInfo[BattIdx].DchrgLimit = (float)pBatCanMsgRx->id30F.iDchaLimn;
				BatInfo[BattIdx].RgnLimit = (float)pBatCanMsgRx->id30F.iRgnLimn;
				BatInfo[BattIdx].DcCurrent = (float)pBatCanMsgRx->id30F.iBattCurr;
				BatInfo[BattIdx].ChrgLimit = (float)pBatCanMsgRx->id30F.iChrgLimn;
				break;

			case 0x31B: //From Master battery
			case 0x31C: //From Slave battery
				BattIdx = Id - 0x31B;
				BatInfo[BattIdx].DCVolt = (float)pBatCanMsgRx->id31B.uBattTerminal * 0.1;
				break;

			case 0x507:	//From Slave (WHYYYYYYYYYY?????)
			case 0x508:	//Summarized info from Master
			case 0x509: //From Master
				BattIdx = (Id == 0x509) ? BAT_IDX_MASTER : (Id==0x507) ? BAT_IDX_SLAVE : BAT_IDX_SUMMERY;
				BatInfo[BattIdx].Soc = pBatCanMsgRx->id507.ratBattSoc;
				BatInfo[BattIdx].CapRemain = (float)pBatCanMsgRx->id507.qBattSoc * 0.001;
				break;

			default :
				break;

		}
	}
}

void Bat_InfoReset(void)
{
	for(uint8_t i = 0; i < BAT_IDX_NUMBER; i++)
	{
		memset(&BatInfo[i], 0, sizeof(BatInfo_t));
	}
	BatCtrl.TimerPrescaler = 0;
}

void Bat_Do100HzLoop (void)
{

	if((BatCtrl.TimerPrescaler % 5) == 0)
	{
		Bat_Id31E_50_INV_Send();
	}

	BatCtrl.TimerPrescaler++;

	if(BatCtrl.TimerPrescaler >= 100)
	{
		BatCtrl.TimerPrescaler = 0;
	}
}



void Bat_CanHandleLoad(ExtranetCANStation_t *pIn)
{
	BatCtrl.pExCANStation = pIn;
}

BatMainSM_e Bat_MainSMGet (Bat_Idx Idx)
{
	return BatInfo[Idx].MainSM;
}

Bat_stBattPreChrgSt_e Bat_PrchSMGet (Bat_Idx Idx)
{
	return BatInfo[Idx].PrchSM;
}

uint8_t Bat_SocGet (Bat_Idx Idx)
{
	return BatInfo[Idx].Soc;
}

uint8_t Bat_ShutdownReqFlgGet(Bat_Idx Idx)
{
	return BatInfo[Idx].Flags.Bit.ShutdownReq;
}

float Bat_DchrgCurrentLimitGet (Bat_Idx Idx)
{
	return BatInfo[Idx].DchrgLimit;
}

void Bat_InfoGet(void* pOut, BatInfoIdx_e InfoIdxIn, Bat_Idx BatIdxIn)
{
	if(pOut)
	{
		switch(InfoIdxIn)
		{
			case BAT_INFO_IDX_DC_VOLT_NOW:
				*(float*)pOut = BatInfo[BatIdxIn].DCVolt;
				break;

			case BAT_INFO_IDX_DC_CURR_NOW:
				*(float*)pOut = BatInfo[BatIdxIn].DcCurrent;
				break;

			case BAT_INFO_IDX_CAPACITY_REMAIN:
				*(float*)pOut = BatInfo[BatIdxIn].CapRemain;
				break;

			case BAT_INFO_IDX_DISCHARGE_CURR_LIMIT:
				*(float*)pOut = BatInfo[BatIdxIn].DchrgLimit;
				break;

			case BAT_INFO_IDX_CHARGE_CURR_LIMIT:
				*(float*)pOut = BatInfo[BatIdxIn].ChrgLimit;
				break;

			case BAT_INFO_IDX_REGEN_CURR_LIMIT:
				*(float*)pOut = BatInfo[BatIdxIn].RgnLimit;
				break;

			case BAT_INFO_IDX_MAIN_SM:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].MainSM;
				break;

			case BAT_INFO_IDX_PRCH_SM:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].PrchSM;
				break;

			case BAT_INFO_IDX_FLAG_ALARM:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Flags.Bit.Alarm;
				break;

			case BAT_INFO_IDX_FLAG_WARNING:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Flags.Bit.Warn;
				break;

			case BAT_INFO_IDX_FLAG_SHUTDOWN_REQ:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Flags.Bit.ShutdownReq;
				break;

			case BAT_INFO_IDX_FLAG_CHARGE_BUSY:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Flags.Bit.ChrgBsy;
				break;

			case BAT_INFO_IDX_FLAG_BALANCE_BUSY:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Flags.Bit.BalBsy;
				break;

			case BAT_INFO_IDX_SOC:
				*(uint8_t*)pOut = BatInfo[BatIdxIn].Soc;
				break;
			default:
				break;
		}
	}
}
