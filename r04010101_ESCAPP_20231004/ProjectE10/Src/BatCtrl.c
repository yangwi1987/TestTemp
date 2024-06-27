#include <BatCtrl.h>
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "Protocol.h"

BatStation_t BatStation = BAT_STATION_DEFAULT;
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

void Bat_Id31E_50_INV_Send(void)
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
    pTxMsg->Byte0.Bit.stInvr = 0;     // Todo: load inverter state from Inveter station
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
}

void Bat_CanMsgLoad(uint32_t Id, uint8_t *pData)
{
	BatCanMsgRx_u *pBatCanMsgRx;

	pBatCanMsgRx = (BatCanMsgRx_u*)pData;
	Bat_ArrayToSmallEndian(pData, 8);
	uint8_t BattIdx = 0;

	if(MsgChksmXorBuild(pData,8) == 0)
//	if(1)
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

			case 0x30F: //From Master battery
			case 0x318:	//From Slave battery
			case 0x31A:	//From Summarized info from Master battery
				BattIdx = (Id==0x30F) ? BAT_IDX_MASTER : (Id == 0x318)? BAT_IDX_SLAVE:BAT_IDX_SUMMERY;
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
			case 0x508:	//From Master
			case 0x509: //Summarized info from Master
				BattIdx = (Id==0x508) ? BAT_IDX_MASTER : (Id==0x507) ? BAT_IDX_SLAVE : BAT_IDX_SUMMERY;
				BatInfo[BattIdx].Soc = pBatCanMsgRx->id507.ratBattSoc;
				BatInfo[BattIdx].CapRemain = (float)pBatCanMsgRx->id507.qBattSoc * 0.001;
				break;

			default :
				break;

		}
	}
}




void Bat_Do100HzLoop (void)
{
	static uint32_t TimeCntPrescalor = 0;


	if((TimeCntPrescalor % 5) == 0)
	{
		Bat_Id31E_50_INV_Send();
	}

	TimeCntPrescalor++;

	if(TimeCntPrescalor >= 100)
	{
		TimeCntPrescalor = 0;
	}
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

float Bat_DchrgCurrentLimitGet (Bat_Idx Idx)
{
	return BatInfo[Idx].DchrgLimit;
}

void Bat_CanHandleLoad(ExtranetCANStation_t *pIn)
{
	BatCtrl.pExCANStation = pIn;
}
