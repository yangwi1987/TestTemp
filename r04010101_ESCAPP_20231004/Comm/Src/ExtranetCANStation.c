/*
 * ExtranetCANStation.c
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#include "ExtranetCANStation.h"

const CanModuleConfig_t CANModuleConfigExtra =
{
	//	ItSelect;					ItActivateItem; 		uint32_t ItLineNumber;	uint32_t RxFifoSrc;
		FDCAN_IT_GROUP_RX_FIFO0,	FDCAN_IT_LIST_RX_FIFO0,	FDCAN_INTERRUPT_LINE0,	FDCAN_RX_FIFO0
};

void ExtranetCANStation_Init( ExtranetCANStation_t *v, const CANProtocol *p, FDCAN_HandleTypeDef *u)
{
	HAL_FDCAN_Stop(v->phfdcan);
	v->Enable = 0;
	v->phfdcan = u;
	v->pProtocol = p;
	v->PlcCnt1 = 0;
	//Clear all data in Queue including rear front value and data in buffer
	v->RxQ.FlushQ( &v->RxQ );
	v->TxQ.FlushQ( &v->TxQ );
	//Setup CAN ID filter from ID config table
	v->DriveSetup.Init(u,&v->DriveSetup);
	//start FDCAN module
	HAL_FDCAN_Start(v->phfdcan);
}

uint8_t ExtranetCANStation_ReadRxFIFO( ExtranetCANStation_t *v, uint32_t lFifoN )
{
	uint32_t lLength = 0;
	uint8_t lIdx = 0;
	FDCAN_RxHeaderTypeDef lRxHead;
	uint8_t lStatus = 0;
	// save all the Rx data to Rx queue from FDCAN1/2/3 modules
	for( lIdx = 0; lIdx < 3; lIdx++ )
	{
		if( HAL_FDCAN_GetRxFifoFillLevel( v->phfdcan, lFifoN ) != 0 )
		{
			HAL_FDCAN_GetRxMessage( v->phfdcan, lFifoN, &lRxHead, v->NowCANDataRx.Data );
			lLength = lRxHead.DataLength >> 16;
			v->NowCANDataRx.Size = (uint8_t)lLength;
			if( lRxHead.IdType == FDCAN_STANDARD_ID )
			{
				v->NowCANDataRx.ID.StdFrame.XTD = 0;
				v->NowCANDataRx.ID.StdFrame.StdID = lRxHead.Identifier & 0x7FF;
			}
			else
			{
				v->NowCANDataRx.ID.ExtFrame.XTD = 1;
				v->NowCANDataRx.ID.ExtFrame.ExtID =  lRxHead.Identifier;
			}
			lStatus = v->RxQ.EnQ( &v->RxQ, &v->NowCANDataRx );
		}
	}
	return lStatus;
}

uint8_t ExtranetCANStation_RxHandlePacket( ExtranetCANStation_t *v )
{

}

uint8_t ExtranetCANStation_WriteTxFIFO( ExtranetCANStation_t *v )
{
	uint32_t lLength = 0;
	uint8_t lStatus = 0;
	// Check if Tx Queue Empty
	if( v->TxQ.DeQ( &v->TxQ,&v->NowCANDataTx ) != QUEUE_EMPTY )
	{
		// Fill in Tx header from Tx queue data
		lLength = (uint32_t)v->NowCANDataTx.Size;
		v->TxHead.ErrorStateIndicator=0;
		v->TxHead.DataLength = ( lLength << 16 );
		if( v->NowCANDataTx.ID.StdFrame.XTD == 0 )
		{
			v->TxHead.IdType = FDCAN_STANDARD_ID;
			v->TxHead.Identifier = v->NowCANDataTx.ID.StdFrame.StdID;
		}
		else
		{
			v->TxHead.IdType = FDCAN_EXTENDED_ID;
			v->TxHead.Identifier = v->NowCANDataTx.ID.ExtFrame.ExtID;
		}

		// Send message
		HAL_FDCAN_AddMessageToTxFifoQ( v->phfdcan, &v->TxHead, v->NowCANDataTx.Data);
		lStatus = QUEUE_OK;
	}
	else
	{
		lStatus = QUEUE_EMPTY;
	}
	return lStatus;
}

uint8_t ExtranetCANStation_TxHandlePacket( ExtranetCANStation_t *v, uint32_t lDIn )
{
	uint8_t lStatus=0;
	STRUCT_CAN_DATA CANDataTemp = CAN_DATA_DEFAULT;
	lStatus = v->pProtocol->TxTranslate( lDIn, CANDataTemp.Data, &v->TxInfo, &v->RxInfo );
	if( lStatus == ID_MATCH )
	{
		CANDataTemp.ID.StdFrame.StdID = lDIn;
		CANDataTemp.ID.StdFrame.XTD = 0;
		CANDataTemp.Size = 8;
		lStatus = v->TxQ.EnQ( &v->TxQ, &CANDataTemp );
	}
	return lStatus;
}

void ExtranetCANStation_DoPLCLoop( ExtranetCANStation_t *v )
{
	uint32_t lIdin = 0;

	//	Enqueue periodic transmit data
	if(v->PlcCnt1>=v->pProtocol->TxPeriodMs )
	{
		v->PlcCnt1=0;
	}
	if( v->PlcCnt1 < v->pProtocol->PeriodicTxIDNumber )
	{
		lIdin = *( v->pProtocol->PeriodicTxIDTable + v->PlcCnt1 );
		v->TxHandlePacket( v, lIdin );
	}
	v->PlcCnt1++;
	v->RxHandlePacket(v);
	v->WriteTxFIFO(v);
}

void ExtranetCANStation_DisableStateReset( ExtranetCANStation_t *v )
{
	if( (v->Enable == DISABLE) && ( v->ForceDisable == DISABLE ))
	{
		if( v->DisableResetCnt >= DISABLE_RST_TIMEOUT )
		{
			v->Enable = 1;
			v->DisableResetCnt = 0;
			v->KeepDisableFlg = 0;
		}
		else
		{
			v->Enable = 0;
			v->DisableResetCnt++;
			if( v->KeepDisableFlg ==  1 )
			{
				v->DisableResetCnt = 0;
				v->KeepDisableFlg = 0;
			}
		}
	} else;
}
