/*
 * UdsNetworkLayer.c
 *
 *  Created on: 2020年6月3日
 *      Author: Will.Yang.CYY
 */
#include "UdsNetworkLayer.h"
#include "string.h"

/*
 * pData : pointer of  buffer contains data to be send
 */


const CanModuleConfig_t CANModuleConfigIntra =
{
	//	ItSelect;					ItActivateItem; 		uint32_t ItLineNumber;	uint32_t RxFifoSrc;
		FDCAN_IT_GROUP_RX_FIFO1,	FDCAN_IT_LIST_RX_FIFO1,	FDCAN_INTERRUPT_LINE1,	FDCAN_RX_FIFO1
};





const CanIdConfig_t LscCanIdTableIntra[CAN_ID_CONFIG_ARRAY_SIZE] ={
//	Id1,						      Id2,				        	{FilterType,IdType,ConfigUsage,Reserved}
	{UDS_RX_ID_LSC_START,		      UDS_RX_ID_LSC_END,	    	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,	1,	0}}},
	{UDS_RX_ID_BRP_FUNCTIONAL_START,  UDS_RX_ID_BRP_FUNCTIONAL_END,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,	1,	0}}},
	{UDS_RX_ID_BRP_PHYSICAL_START,	  UDS_RX_ID_BRP_PHYSICAL_END,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,	1,	0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,1,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,1,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,1,0}}},
};



void NetworkLayer_Init(NetworkCtrl_t *v , FDCAN_HandleTypeDef *p )
{

	HAL_FDCAN_Stop(p);
//	CANDrive_ModuleConfig(p ,v->pModuleConfig);
//
//	CANDrive_IdConfig( p, v->pIdConfigTable);
	v->DriveSetup.Init( p, &v->DriveSetup );
	v->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	v->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	v->TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	v->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	v->TxHeader.IdType = FDCAN_STANDARD_ID;
	v->TxHeader.Identifier = 0x308;
	v->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	v->STminReq = 1;
	v->BsReq = 10;
	v->pCanHandle = p;
	v->Rx.Status = Rx_Idle;
	v->Tx.Status = Tx_Idle;
	v->Rx.DataSize = MAX_BUFFER_SIZE;
	v->Tx.DataSize = MAX_BUFFER_SIZE;
	v->STCounter = 0;

	HAL_FDCAN_Start(p);



}
void NetworkLayer_TxDataHandle (NetworkCtrl_t *v)	//in PLC Loop
{
	PDU_u pContainer ;
	uint16_t i = 0;
	uint16_t u16Temp = 0;

	if( (v->Rx.CanId >=UDS_RX_ID_LSC_START ) && (v->Rx.CanId <= UDS_RX_ID_LSC_END ))
	{
		v->TxHeader.Identifier = UDS_TX_ID_LSC;
	}
	else if( ((v->Rx.CanId >=UDS_RX_ID_BRP_FUNCTIONAL_START ) && (v->Rx.CanId <= UDS_RX_ID_BRP_FUNCTIONAL_END )) || \
			 ((v->Rx.CanId >=UDS_RX_ID_BRP_PHYSICAL_START ) && (v->Rx.CanId <= UDS_RX_ID_BRP_PHYSICAL_END )))
	{
		v->TxHeader.Identifier = UDS_TX_ID_BRP;
	}

	if( v->Tx.Status == Tx_Request)
	{
		pContainer.Default.PCI.All = 0;
		for( i = 0; i < 7; i++ )
		{
			pContainer.Default.Datas[i]=0;
		}
		if( v->Tx.LengthTotal < 8 )
		{
			pContainer.SF.PCI.Bits.PCIType = PCI_SF;
			pContainer.SF.PCI.Bits.DLC = v->Tx.LengthTotal;
			for( i = 0; i < v->Tx.LengthTotal; i++ )
			{
				pContainer.SF.Datas[i] = v->Tx.Data[i];
			}
			if ( v->TxHeader.Identifier == UDS_TX_ID_BRP )
			{
			    for( ; i < 7; i++ )
			    {
			    	pContainer.SF.Datas[i] = UDS_TX_PADDING_CODE;
			    }
			}
			HAL_FDCAN_AddMessageToTxFifoQ(v->pCanHandle, &v->TxHeader, (uint8_t*)&pContainer);
			v->Tx.Result = N_OK;
			v->Tx.Status = Tx_Complete;
		}
		else
		{
			pContainer.FF.PCI.Bits.PCIType = PCI_FF;
			pContainer.FF.PCI.Bits.DLC_L = v->Tx.LengthTotal & 0xFF;
			pContainer.FF.PCI.Bits.DLC_H = v->Tx.LengthTotal >> 8;
			v->Tx.LengthHandled = 0;
			v->Tx.SnNow = 1;
			for( i = 0; i < 6; i++ )
			{
				pContainer.FF.Datas[i] = v->Tx.Data[i];
			}
			HAL_FDCAN_AddMessageToTxFifoQ(v->pCanHandle, &v->TxHeader, (uint8_t*)&pContainer);
			v->Tx.LengthHandled += 6;
			v->Tx.Status = Tx_Processing;
			v->Tx.FlowCtrlFlag = FlowCtrl_WaitforFC;
			v->Tx.CFCnt = 0;
			v->Tx.TimeoutCnt = 0;
		}
	}
	else if(v->Tx.Status == Tx_Processing)
	{
		if( v->Tx.FlowCtrlFlag == FlowCtrl_FCReceived )
		{
			pContainer.CF.PCI.Bits.PCIType = PCI_CF;
			pContainer.CF.PCI.Bits.SN = v->Tx.SnNow;
			v->Tx.SnNow++;
			u16Temp = v->Tx.LengthTotal - v->Tx.LengthHandled ;
			u16Temp = ( u16Temp >7 )? 7 : u16Temp;
			for( i = 0; i < u16Temp; i++ )
			{
				pContainer.CF.Datas[i] = v->Tx.Data[i+v->Tx.LengthHandled];
			}
			if (v->TxHeader.Identifier == UDS_TX_ID_BRP)
			{
			    for( ; i < 7; i++ )
			    {
			    	pContainer.CF.Datas[i] = UDS_TX_PADDING_CODE;
			    }
			}
			HAL_FDCAN_AddMessageToTxFifoQ(v->pCanHandle, &(v->TxHeader), (uint8_t*)&pContainer);
			v->Tx.LengthHandled += u16Temp;
			if( v->Tx.LengthHandled >= v->Tx.LengthTotal)
			{
				//all data transmitted
				v->Tx.Status = Tx_Complete;
				v->Tx.Result = N_OK;
				v->Tx.FlowCtrlFlag = FlowCtrl_Idle;
			}
			else
			{
				if( v->BsCmd != 0 )
				{
					v->Tx.CFCnt++;
					if( v->Tx.CFCnt >= v->BsCmd )
					{
						v->Tx.FlowCtrlFlag = FlowCtrl_WaitforFC;
						v->Tx.CFCnt = 0;
					}
				}
			}
		}
		else
		{
			v->Tx.TimeoutCnt++;
			if( v->Tx.TimeoutCnt > 10000 )
			{
				v->Tx.Result = N_TIMEOUT_Bs;
				v->Tx.Status = Tx_Error;
			}
		}
	}
	else
	{
//		Undefined status value ,report error and do nothing
		v->Tx.Status = Tx_Idle;
		v->Tx.Result = N_ERROR;
	}

}

void NetworkLayer_RxDataHandle( NetworkCtrl_t *v ) //in PLC Loop
{
	PDU_u pContainer ;
	uint16_t i = 0;
	uint8_t j = 0;
	uint8_t PciTemp=0;
	int16_t		s16Temp=0;
	uint16_t FFDlc=0;
	FDCAN_RxHeaderTypeDef RxHeader;

	for( j = 0; j < 3; j++ )	//Scan all Buffer of RXFIFO1
	{
		if( HAL_FDCAN_GetRxFifoFillLevel(v->pCanHandle,FDCAN_RX_FIFO1) != 0 )
		{
			HAL_FDCAN_GetRxMessage( v->pCanHandle, FDCAN_RX_FIFO1, &RxHeader, (uint8_t*)&pContainer);
			PciTemp = pContainer.Default.PCI.Bits.PCIType;
			if( v->Rx.Status == Rx_Idle )
			{
				if( PciTemp == PCI_SF)
				{
					if( ( pContainer.SF.PCI.Bits.DLC <= 7 ) && ( pContainer.SF.PCI.Bits.DLC > 0 ) )
					{
						v->Rx.LengthTotal = pContainer.SF.PCI.Bits.DLC;
						for( i = 0; i < v->Rx.LengthTotal; i++ )
						{
							v->Rx.Data[i] = pContainer.SF.Datas[i];
						}
						v->Rx.Result = N_OK;
						v->Rx.Status  = Rx_Complete;
						v->Rx.CanId = RxHeader.Identifier;
					}
					else
					{
						v->Rx.Result = N_UNEXP_PDU;
						v->Rx.Status  = Rx_Idle;
					}
				}
				else if(PciTemp == PCI_FF)
				{
					FFDlc = pContainer.FF.PCI.Bits.DLC_H<<8;
					FFDlc += pContainer.FF.PCI.Bits.DLC_L;

					if( FFDlc <= v->Rx.DataSize)
					{

						if( FFDlc > 7 )
						{
							v->Rx.LengthTotal = FFDlc;
							v->Rx.LengthHandled =0;
							for( i = 0; i < 6; i++ )
							{
								v->Rx.Data[i] = pContainer.FF.Datas[i];
							}
							v->Rx.SnBefore = 0;
							v->Rx.Result = N_OK;
							v->Rx.LengthHandled+=6;
							v->Rx.Status = Rx_Processing;
							v->Rx.TimeoutCnt =0;
							v->FlowCtrlSendReq(v,FlowStatus_ContinueToSend);
						}
						else
						{
							//wrong data length, ignore received data
							v->Rx.Result = N_UNEXP_PDU;
							v->Rx.Status = Rx_Idle;
							v->FlowCtrlSendReq(v,FlowStatus_Overflow);
						}
					}
					else
					{
						v->Rx.Result = N_UNEXP_PDU;
						v->Rx.Status = Rx_Idle;
					}
				}
				else if(PciTemp == PCI_FC )
				{
					if( v->Tx.FlowCtrlFlag == FlowCtrl_WaitforFC )
					{
						NetworkLayer_ChangeParamRequest(v,pContainer.FC.STMin,pContainer.FC.BlockSize);
						v->Tx.FlowCtrlFlag = FlowCtrl_FCReceived;
					}
					else
					{
						if(v->Tx.Status == Tx_Processing)
						{
							if( pContainer.FC.PCI.Bits.FS == FlowStatus_Wait )
							{
								v->Tx.FlowCtrlFlag = FlowCtrl_WaitforFC;
								v->Tx.TimeoutCnt = 0;
							}
							else if(pContainer.FC.PCI.Bits.FS == FlowStatus_Overflow)
							{
								v->Tx.Status = Tx_Idle;
								v->Tx.Result = N_WFT_OVERN;
							}
							else
							{
								//do nothing
							}
						}
						else
						{
							v->Rx.Result = N_UNEXP_PDU;
							v->Rx.Status = Rx_Idle;
						}
					}

				}
				else
				{
					v->Rx.Result = N_UNEXP_PDU;
					v->Rx.Status = Rx_Idle;
				}
			}
			else if( v->Rx.Status == Rx_Processing )
			{
				v->Rx.SnNow = ( pContainer.CF.PCI.Bits.SN == 0 )? 16 : pContainer.CF.PCI.Bits.SN;
				if( ( v->Rx.SnNow - v->Rx.SnBefore ) == 1 )
				{
					if( v->Rx.LengthTotal > v->Rx.LengthHandled )
					{
						v->Rx.SnBefore = pContainer.CF.PCI.Bits.SN;		// SnBefore will loop between 0x00~0x0F(0~15)
						v->Rx.TimeoutCnt =0;
						s16Temp = v->Rx.LengthTotal - v->Rx.LengthHandled;
						s16Temp = ( s16Temp > 7 )? 7 : s16Temp;
						for( i = 0; i < s16Temp; i++ )
						{
							v->Rx.Data[i+v->Rx.LengthHandled] = pContainer.CF.Datas[i];
						}
						v->Rx.LengthHandled += s16Temp;

						if( v->Rx.LengthHandled == v->Rx.LengthTotal)
						{
							v->Rx.CFCnt = 0;
							v->Rx.SnNow =0;
							v->Rx.SnBefore = 0;
							v->Rx.Result = N_OK;
							v->Rx.Status = Rx_Complete;
							v->Rx.CanId = RxHeader.Identifier;
						}
						else
						{
							if( v->BsReq != 0) //with block size limitation,send flowctrl-frame when received frame number achieved block size limit
							{
								v->Rx.CFCnt++;
								if( v->Rx.CFCnt >= v->BsReq )
								{
									v->Rx.CFCnt=0;
									v->FlowCtrlSendReq(v,FlowStatus_ContinueToSend);
								}
							}
						}
					}
					else
					{
						//unexpect PDU
						v->Rx.Status = Rx_Idle;
						v->Rx.Result = N_ERROR;
					}
				}
				else
				{
					//wrong data serial number ,abort reception
					v->Rx.Result = N_WRONG_SN;
					v->Rx.Status = Rx_Idle;
				}
			}
			else if(v->Rx.Status == Rx_Reading )
			{
				v->Rx.Result = N_BUFFER_OVFLW;
				v->FlowCtrlSendReq( v, FlowStatus_Overflow );
			}
		}
	}
}


void NetworkLayer_FlowCtrlSendReq( NetworkCtrl_t *v,uint8_t status )
{
	uint8_t Txdata[8]={0,0,0,0,0,0,0,0};
	status &= 0x0F;
	Txdata[0] = 0x30 + status;
	Txdata[1] = v->BsReq;
	Txdata[2] = v->STminReq;
	if( (v->Rx.CanId >=UDS_RX_ID_LSC_START ) && (v->Rx.CanId <= UDS_RX_ID_LSC_END ))
	{
		v->TxHeader.Identifier = UDS_TX_ID_LSC;
	}
	else if( ((v->Rx.CanId >=UDS_RX_ID_BRP_FUNCTIONAL_START ) && (v->Rx.CanId <= UDS_RX_ID_BRP_FUNCTIONAL_END )) || \
			 ((v->Rx.CanId >=UDS_RX_ID_BRP_PHYSICAL_START ) && (v->Rx.CanId <= UDS_RX_ID_BRP_PHYSICAL_END )))
	{
		v->TxHeader.Identifier = UDS_TX_ID_BRP;
	}
	HAL_FDCAN_AddMessageToTxFifoQ(v->pCanHandle, &v->TxHeader, Txdata);
}

uint8_t NetworkLayer_ChangeParamRequest(NetworkCtrl_t *v ,uint8_t lStminCmd, uint8_t lBsCmd  )
{
	if( lStminCmd > FLOW_CTRL_STM_MIN_LIMIT )
	{
		return ChangeParam_Wrong_Value;
	}

	if( lBsCmd > FLOW_CTRL_BS_LIMIT )
	{
		return ChangeParam_Wrong_Value;
	}

	if( v->Rx.Status == Rx_Processing )
	{
		return ChangeParam_RX_ON;
	}

	v->STminCmd = lStminCmd;
	v->BsCmd = lBsCmd;
	return ChangeParam_N_OK;
}



