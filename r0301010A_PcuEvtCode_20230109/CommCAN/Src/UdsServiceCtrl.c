/*
 * UdsServiceCtrl.c
 *
 *  Created on: 2020年5月28日
 *      Author: Will.Yang.CYY
 */

#include "UdsServiceCtrl.h"
#include "string.h"



void UdsServiceCtrl_SendDataPeriodically (NetWorkService_t *p, LinkLayerCtrlUnit_t *pTx, uint32_t ParamID)
{
	uint32_t TempU32 = 0;
	uint8_t * pPt = (uint8_t*)&TempU32;
	uint8_t i=0;
	static uint16_t IntervalCnt =1;
	if((p->PeriodUpdateCtrl.EnableFlag !=0) && (p->PeriodUpdateCtrl.StartFlag !=0))
	{
		if((IntervalCnt++ >= p->PeriodUpdateCtrl.IntervalInMs) && (pTx->Status != Tx_Request))
		{
			IntervalCnt = 1;

			pTx->Data[0] = SID_READ_DATA_BY_ID + POSITIVE_RESPONSE_OFFSET;

			TempU32 = ParamID;
			ByteSwap((uint32_t*)&TempU32,2);
			pPt = (uint8_t*)&TempU32;
			for(i=0;i<2;i++)
			{
				pTx->Data[1+i] = *(pPt+i);
			}
			TempU32 = p->AccessParam( 0, (uint16_t)ParamID, 0, PARAM_READ, NULL);
			ByteSwap((uint32_t*)&TempU32,2);
			pPt = (uint8_t*)&TempU32;
			for(i=0;i<2;i++)
			{
				pTx->Data[3+i] = *(pPt+i);
			}
			pTx->LengthTotal = 5;
			pTx->Status = Tx_Request;
		}
	}
	else
	{
		IntervalCnt=0;
	}
}

void UdsServiceCtrl_SendDataPeriodicallyNoSwap (NetWorkService_t *p, LinkLayerCtrlUnit_t *pTx, uint32_t ParamID)
{
	uint32_t TempU32 = 0;
	uint8_t * pPt = (uint8_t*)&TempU32;
	uint8_t i=0;
	static uint16_t IntervalCnt =1;
	if((IntervalCnt++ >= p->PeriodUpdateCtrl.IntervalInMs) && (pTx->Status != Tx_Request))
	{
		IntervalCnt = 1;

		pTx->Data[0] = SID_READ_DATA_BY_ID + POSITIVE_RESPONSE_OFFSET;

		TempU32 = ParamID;
		ByteSwap((uint32_t*)&TempU32,2);
		pPt = (uint8_t*)&TempU32;
		for(i=0;i<2;i++)
		{
			pTx->Data[1+i] = *(pPt+i);
		}
		TempU32 = p->AccessParam( 0, (uint16_t)ParamID, 0, PARAM_READ, NULL);
		//ByteSwap((uint32_t*)&TempU32,2);
		pPt = (uint8_t*)&TempU32;
		for(i=0;i<4;i++)
		{
			pTx->Data[3+i] = *(pPt+i);
		}
		pTx->LengthTotal = 7;
		pTx->Status = Tx_Request;
	}
}

void UdsServiceCtrl_NegativeRspReq( LinkLayerCtrlUnit_t *pTx, uint8_t Sid, uint8_t Nrc )
{
	pTx->Data[0] = 0x7F;
	pTx->Data[1] = Sid;
	pTx->Data[2] = Nrc;
	pTx->LengthTotal = 3;
	pTx->Status = Tx_Request;
}

void UdsServiceCtrl_ReadDataRegionF( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, uint16_t ParamID )
{
	uint8_t i = 0;
	uint16_t UDSDataBuf[MAX_UDS_DATA_BUF] = {0};
	uint8_t *pPt;
	uint16_t DIDDataLength; // the length of DID data(bytes), excluding first 3 bytes(SID, ParamID)

	switch( ParamID )
	{
		case 0x089:	// HW Version Read
			pPt = (uint8_t*)&HWVerNumber;
			DIDDataLength = HW_VER_NUM_IDX;
			break;

		case 0x187:	// Part Number Read
			pPt = (uint8_t*)&BomNumber;
			DIDDataLength = PART_NUM_IDX;
			break;

		case 0x189:	// Software Version Number Read
			pPt = (uint8_t*)&FWVerNumber;
			DIDDataLength = SW_VER_NUM_IDX;
			break;

		case 0x18A:	// System Supplier Identifier Read
			pPt = (uint8_t*)&SystemSupplyerID;
			DIDDataLength = SYS_SUP_ID_IDX;
			break;

		case 0x18C:	// Serial Number Data Identifier Read
			for( i = 0; i < (SERIAL_NUM_IDX >> 1); i++ )
			{
				UDSDataBuf[i] = DriveParams.PCUParams.PCUSNCode[i];
				ByteSwap( (uint32_t*)&UDSDataBuf[i], 2 );
			}
			pPt = (uint8_t*)&UDSDataBuf[0];
			DIDDataLength = SERIAL_NUM_IDX;
			break;

		case 0x191:	// Read motor serial number of axis 1
			for( i = 0; i < (MOT_SERIAL_NUM_IDX >> 1); i++ )
			{
				UDSDataBuf[i] = 0;
				ByteSwap( (uint32_t*)&UDSDataBuf[i], 2 );
			}
			pPt = (uint8_t*)&UDSDataBuf[0];
			DIDDataLength = MOT_SERIAL_NUM_IDX;
			break;

		case 0x193:	// Read motor hardware version of axis 1

			UDSDataBuf[0] = 0;
			ByteSwap( (uint32_t*)&UDSDataBuf[0], 2 );

			UDSDataBuf[1] = 0;
			ByteSwap( (uint32_t*)&UDSDataBuf[1], 2 );

			pPt = (uint8_t*)&UDSDataBuf[0];
			DIDDataLength = MOT_HW_VER_NUM_IDX;
			break;

		case 0x195:	// // Read motor software version of axis 1
			pPt = (uint8_t*)&PSBVerNumber;
			DIDDataLength = MOT_SW_VER_NUM_IDX;
			break;
	}

	for( i = 0; i < DIDDataLength; i++ )
	{
		pTx->Data[i + 3] = *(pPt + i);
	}
	pTx->LengthTotal = (DIDDataLength + 3);
}

void UdsServiceCtrl_ServiceHandler( NetWorkService_t *p ,NetworkCtrl_t *v  )
{
	uint8_t lSID = 0;
	if( v->Rx.Status == Rx_Complete)
	{
		v->Rx.Status = Rx_Reading;
		lSID = v->Rx.Data[0];
		switch (lSID)
		{
			case SID_DIAGNOSTIC_SESSION_CONTROL:
			{
				UdsServiceCtrl_SessionControl( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_ECUReset :
			{
				UdsServiceCtrl_ECUReset( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_SECURITY_ACCESS :
			{
				UdsServiceCtrl_SecurityAccess( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_COMMUNICATION_CTRL:
			{
				UdsServiceCtrl_CommunicationCtrl( &v->Rx, &v->Tx, &ExtranetCANStation);
				break;
			}
			case SID_READ_DATA_BY_ID :
			{
				UdsServiceCtrl_ReadDataByID( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_REQUEST_DOWNLOAD :
			{
				UdsServiceCtrl_RequestDownload( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_TRANSFER_DATA :
			{
				UdsServiceCtrl_TransferData( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_TEST_PRESENT:
			{
				UdsServiceCtrl_TesterPresent( &v->Rx, &v->Tx, &ExtranetCANStation);
				break;
			}
			case SID_REQUEST_TRANSFER_EXIT:
			{
				UdsServiceCtrl_RequestTransferExit( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_WRITE_DATA_BY_ID_LSC:
			{
				UdsServiceCtrl_WriteDataByIDLsc( p, &v->Rx, &v->Tx );
				break;
			}
			case SID_RESPONSE_ON_EVENT:
			{
				break;
			}
			default :
			{
				p->NegativeRspReq( &v->Tx, v->Rx.Data[0], NRC_serviceNotSupported );
				break;
			}
		}
		v->Rx.Status = Rx_Idle;

	}
}

void UdsServiceCtrl_ReadDataByID( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	Union_LscParamID RxDataParamID;
	uint16_t TargetID = 0, ParamID = 0;
	int32_t pTemp = 0;
	uint8_t i = 0;
	uint8_t *pPt;
	uint8_t Result = PARAM_ACCESS_SUCCESS;
	uint8_t ReplyResult;

	RxDataParamID.All = (pRx->Data[1]<<8)+pRx->Data[2];
	TargetID = RxDataParamID.Bits.TargetID;
	ParamID = RxDataParamID.Bits.ParamID;

	switch( TargetID )
	{
		case 0xF: // read target is Flash
			// todo add check DID_ACCESS_SUCCESS

			// prepare SID, ParamID
			for( i = 0; i < 3; i++ )
			{
				pTx->Data[i] = pRx->Data[i];
			}

			// assign data from specific data address and set length of total data.
			UdsServiceCtrl_ReadDataRegionF( p, pRx, pTx, ParamID );
			pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
			pTx->Status = Tx_Request;
			break;

		case 0x2: // read target is AxisID 2
			pTemp = p->AccessParam( 2, ParamID, &pTemp, PARAM_READ, &Result );
			if( Result == PARAM_ACCESS_SUCCESS )
			{
				ByteSwap( (uint32_t*)&pTemp, 2 );
				pPt = (uint8_t*)&pTemp;
				for( i = 0; i < 3; i++ )
				{
					pTx->Data[i] = pRx->Data[i];
				}
				for( i = 0; i < 2; i++ )
				{
					pTx->Data[i + 3] = *(pPt + i);
				}
				pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
				pTx->LengthTotal = 5;
				pTx->Status = Tx_Request;
			}
			else
			{
				if( Result == PARAM_ACCESS_FAIL_NO_AUTH )
				{
					ReplyResult = NRC_SecurityAccessDenied;
				}
				else
				{
					ReplyResult = NRC_RequestOutOfRange;
				}
				p->NegativeRspReq( pTx, SID_READ_DATA_BY_ID, ReplyResult );
			}
			break;

		case 0x1: // read target is AxisID 1
		default:  // The default is AxisID 1
			pTemp = p->AccessParam( 1, ParamID, &pTemp, PARAM_READ, &Result );
			if( Result == PARAM_ACCESS_SUCCESS )
			{
				ByteSwap( (uint32_t*)&pTemp, 2 );
				pPt = (uint8_t*)&pTemp;
				for( i = 0; i < 3; i++ )
				{
					pTx->Data[i] = pRx->Data[i];
				}
				for( i = 0; i < 2; i++ )
				{
					pTx->Data[i + 3] = *(pPt + i);
				}
				pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
				pTx->LengthTotal = 5;
				pTx->Status = Tx_Request;
			}
			else
			{
				if( Result == PARAM_ACCESS_FAIL_NO_AUTH )
				{
					ReplyResult = NRC_SecurityAccessDenied;
				}
				else
				{
					ReplyResult = NRC_RequestOutOfRange;
				}
				p->NegativeRspReq( pTx, SID_READ_DATA_BY_ID, ReplyResult );
			}
			break;
	}
}

void UdsServiceCtrl_WriteDataByIDLsc( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	Union_LscParamID ParamID;
	uint16_t lParamID = 0;
	int32_t pTemp=0;
	int32_t retValue=0;
	uint8_t i=0;
	uint8_t * pPt;
	uint8_t Result = PARAM_ACCESS_SUCCESS;
	uint8_t ReplyResult;

	ParamID.All = (pRx->Data[1]<<8)+pRx->Data[2];
	lParamID = ParamID.Bits.ParamID;

	pPt = (uint8_t*)&pTemp;
	for(i=0;i<2;i++)
	{
		*(pPt+i)=pRx->Data[3+i];
	}
	ByteSwap((uint32_t*)&pTemp,2);
	retValue = p->AccessParam( 1, lParamID, &pTemp, PARAM_WRITE, &Result );

	if(Result == PARAM_ACCESS_SUCCESS)
	{

		for(i=0;i<3;i++)
		{
			pTx->Data[i] = pRx->Data[i] ;
		}

		ByteSwap((uint32_t*)&retValue,2);
		pPt = (uint8_t*)&retValue;
		for(i=0;i<2;i++)
		{
			pTx->Data[3+i] = *(pPt+i);
		}

		pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
		pTx->LengthTotal = 5;
		pTx->Status = Tx_Request;
	}
	else
	{
		if(Result == PARAM_ACCESS_FAIL_NO_AUTH)
		{
			ReplyResult = NRC_SecurityAccessDenied;
		}
		else
		{
			ReplyResult = NRC_RequestOutOfRange;
		}
		p->NegativeRspReq(pTx,SID_WRITE_DATA_BY_ID_LSC,ReplyResult);
	}
}
void UdsServiceCtrl_SessionControl( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	uint8_t lSessionCMD = 0;
	lSessionCMD = pRx->Data[1];

	switch ( lSessionCMD )
	{
	case Session_0x01_Default:
		p->pParamMgr->Session = Session_0x01_Default;
		break;
	case Session_0x02_Programming:
		p->pParamMgr->Session = Session_0x02_Programming;
		break;
	case Session_0x03_ExtendedDiagnostic:
		p->pParamMgr->Session = Session_0x03_ExtendedDiagnostic;
		break;
	case Session_0x04_SafetySystemDiagnostic:
		p->pParamMgr->Session = Session_0x04_SafetySystemDiagnostic;
		break;
	case Session_0x40_VehicleManufacturerSpecific:
		p->pParamMgr->Session = Session_0x40_VehicleManufacturerSpecific;
		break;
	case Session_0x60_SystemSupplierSpecific:
		p->pParamMgr->Session = Session_0x60_SystemSupplierSpecific;
		break;
	default:
		p->pParamMgr->Session = Session_0x01_Default;
		break;
	}

	pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
	pTx->Data[1] = pRx->Data[1];
	pTx->Data[2] = 0;
	pTx->Data[3] = 0x32;
	pTx->Data[4] = 0x01;
	pTx->Data[5] = 0xF4;
	pTx->LengthTotal=6;
	pTx->Status = Tx_Request;
}

void UdsServiceCtrl_SecurityAccess( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	p->pSecurityCtrl->SubFuncIn = pRx->Data[1];
	p->pSecurityCtrl->SubFuncCheck ( p->pSecurityCtrl, &pRx->Data[2], &pTx->Data[2] );

	if ( p->pSecurityCtrl->SecureState == UDS_SECURE_STATE_SEND_SEED )
	{
		pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
		pTx->Data[1] = p->pSecurityCtrl->SubFuncIn;
		pTx->LengthTotal = 2 + UDS_SECURE_SEED_SIZE;
		pTx->Status = Tx_Request;
		p->pSecurityCtrl->SecureState = UDS_SECURE_STATE_WAIT_KEY;
	}
	else if ( (p->pSecurityCtrl->SecureState == UDS_SECURE_STATE_CHECKED) && (p->pSecurityCtrl->SecureResult == UDS_SECURE_RESULT_SUCCESS) )
	{
		p->pParamMgr->Authority = p->pSecurityCtrl->SecureLvNow;
		pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
		pTx->Data[1] = p->pSecurityCtrl->SubFuncIn;
		pTx->LengthTotal = 2;
		pTx->Status = Tx_Request;
	}
	else
	{
		switch ( p->pSecurityCtrl->SecureResult )
		{
			case UDS_SECURE_RESULT_FAIL_WRONG_SEQ:
				p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_RequestSequenceError );
				break;
			case UDS_SECURE_RESULT_FAIL_WRONG_KEY:
				p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_InvalidKey );
				break;
			case UDS_SECURE_RESULT_FAIL_SUB_FUNCTION_VALUE_MISMATCH:
				p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_ConditionsNotCorrect );
				break;
			case UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL:
				p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_SubFunctionNotSupported );
				break;
			default:
				p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_ConditionsNotCorrect );
				break;
		}
	}

}

void UdsServiceCtrl_ECUReset(NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
	pTx->Data[1] = pRx->Data[1];
	pTx->LengthTotal = 2;
	pTx->Status = Tx_Request;
}

void UdsServiceCtrl_CommunicationCtrl(LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ExtranetCANStation_t *pOp)
{
	/*
	 * Local Declare
	 */
	Union_CommunicationCtrlParamSunFunc ParamID;
	ParamID.All = pRx->Data[1];

	/*
	 * Process Code
	 */
	if( ParamID.Bits.FuncID == enableRxAndTx )
	{
		pOp->Enable = CommunicationEnable;
	}
	else if( ParamID.Bits.FuncID == enableRxAndDisableTx )
	{
		pOp->Enable = CommunicationDisable;
	}
	else if( ParamID.Bits.FuncID == disableRxAndEnableTx )
	{
		pOp->Enable = CommunicationEnable;
	}
	else if( ParamID.Bits.FuncID == disableRxAndTx )
	{
		pOp->Enable = CommunicationDisable;
	}
	else;

	/*
	 * Response Code
	 */
	if( ParamID.Bits.RspState == CommunicationRspOff );
	else
	{
		pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
		pTx->Data[1] = pRx->Data[1];
		pTx->Data[2] = pRx->Data[2];
		pTx->LengthTotal = 3;
		pTx->Status = Tx_Request;
	}
}

void UdsServiceCtrl_TesterPresent(LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ExtranetCANStation_t *pOp)
{
	uint8_t ParamID = pRx->Data[1];

	if( ParamID == 0x80)
	{
		pOp->KeepDisableFlg = CommunicationEnable;
	}
	else
	{
		pOp->KeepDisableFlg = CommunicationDisable;
	}
}

void UdsServiceCtrl_RequestDownload( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx )
{

	uint8_t i = 1;
	uint32_t lStartAddress = 0;
	uint32_t lDataLengthInByte = 0;
	uint16_t lByteNumInOneBlockOut = 0;
	DataTransferCtrlInform_t *pt;
	uint8_t CheckResult=0;

	pt =(DataTransferCtrlInform_t*)&(pRx->Data[1]);

	for( i = 0; i < pt->SizeCode.AddressSize; i++)
	{
		*((uint8_t*)&lStartAddress+i) = pRx->Data[3+i];
	}
	for( i = 0; i < pt->SizeCode.LengthSize; i++)
	{
		*((uint8_t*)&lDataLengthInByte+i) = pRx->Data[3+pt->SizeCode.AddressSize+i];
	}
	ByteSwap( &lStartAddress, pt->SizeCode.AddressSize );
	ByteSwap( &lDataLengthInByte, pt->SizeCode.LengthSize );
	p->TransferCtrl.pStartMemAddr = (uint8_t*)lStartAddress;
	p->TransferCtrl.LengthTotal = lDataLengthInByte;
	p->TransferCtrl.LengthHandled = 0;
	p->TransferCtrl.Inform.FormatCode.All = pt->FormatCode.All;
	p->TransferCtrl.Inform.SizeCode.All = pt->SizeCode.All;
	p->TransferCtrl.Operation = TransferCtrlOperation_DownloadReq;
	p->TransferCtrl.LoadStatus = TransferCtrlLoad_Processing;
	p->TransferCtrl.BlockSequenceCounter = 0;
	pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
	pTx->Data[1] = 0x20;
	lByteNumInOneBlockOut = p->TransferCtrl.MaxNumberOfBlockLength;
/*
 * User Code Begin
 */
	if( p->TransferCtrl.TargetAddressAvailableCheck != 0 )
	{
		CheckResult = p->TransferCtrl.TargetAddressAvailableCheck(p->TransferCtrl.pStartMemAddr, p->TransferCtrl.LengthTotal );
	}
/*
 * User Code End
 */
	if( CheckResult == 0 )
	{
		ByteSwap((uint32_t*)&lByteNumInOneBlockOut,2);
		for( i = 0; i < 2; i++ )
		{
			pTx->Data[2+i]=*((uint8_t*)&lByteNumInOneBlockOut+i);
		}
		pTx->LengthTotal=4;
		pTx->Status = Tx_Request;
	}
	else
	{
		p->NegativeRspReq( pTx, SID_REQUEST_DOWNLOAD, NRC_IncorrectMessageLengthOrInvalidFormat );
	}
}
void UdsServiceCtrl_TransferData( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx )
{
//	uint16_t i = 0;
	uint16_t lByteRxceived=0;
	uint8_t BlockSequenceCounterNow=0;
//	uint8_t *pDest;
	uint8_t *pSrc;

	if( p->TransferCtrl.Operation == TransferCtrlOperation_DownloadReq )
	{

		BlockSequenceCounterNow = pRx->Data[1];
		if( BlockSequenceCounterNow == ( p->TransferCtrl.BlockSequenceCounter + 1 ) )
		{
			lByteRxceived= pRx->LengthTotal - 2 ;	//SID(1byte) + SequenceBlockCnt(1Byte) = 2byte

			if( ( p->TransferCtrl.LengthHandled + lByteRxceived ) <= p->TransferCtrl.LengthTotal )
			{
/*
 * User Code Begin
 */
				pSrc = (uint8_t*)&pRx->Data[2];
				if( p->TransferCtrl.WriteToFlash != 0)
				{
					p->TransferCtrl.WriteToFlash( pSrc , p->TransferCtrl.pStartMemAddr + p->TransferCtrl.LengthHandled , lByteRxceived );
				}
/*
 * User Code End
 */
				p->TransferCtrl.LengthHandled += lByteRxceived;
				pTx->Data[0] = SID_TRANSFER_DATA + POSITIVE_RESPONSE_OFFSET;
				pTx->Data[1] = BlockSequenceCounterNow;
				p->TransferCtrl.BlockSequenceCounter = BlockSequenceCounterNow;
				pTx->LengthTotal = 2;
				pTx->Status = Tx_Request;
			}
			else
			{
				p->TransferCtrl.LoadStatus = TransferCtrlLoad_Idle;
				p->TransferCtrl.Operation = TransferCtrlOperation_Idle;
				p->NegativeRspReq( pTx,SID_TRANSFER_DATA,NRC_IncorrectMessageLengthOrInvalidFormat );
			}
		}
		else
		{
			//wrong block sequence counter value,the data might be missing or incorrect,abort receive/transmit
			p->TransferCtrl.LoadStatus = TransferCtrlLoad_Idle;
			p->TransferCtrl.Operation = TransferCtrlOperation_Idle;
			p->NegativeRspReq( pTx,SID_TRANSFER_DATA,NRC_RequestSequenceError );
		}
	}
	else
	{
		p->TransferCtrl.LoadStatus = TransferCtrlLoad_Idle;
		p->TransferCtrl.Operation = TransferCtrlOperation_Idle;
		p->NegativeRspReq( pTx,SID_TRANSFER_DATA,NRC_WrongBlockSequenceCounter );
	}
}
void UdsServiceCtrl_RequestTransferExit( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx )
{
	uint16_t i = 0;
	uint16_t lChecksum = 0;
	if( p->TransferCtrl.LengthTotal == p->TransferCtrl.LengthHandled )
	{
/*
 * User Code Begin
 */
		if( p->TransferCtrl.ChecksumCalculate != 0 )
		{
			lChecksum = p->TransferCtrl.ChecksumCalculate( p->TransferCtrl.pStartMemAddr, p->TransferCtrl.LengthTotal);
		}
/*
 * User Code End
 */
		for(i = 0; i < 5; i++ )
		{
			pTx->Data[i] = pRx->Data[i];
		}
		pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
		pTx->LengthTotal = 5;
		pTx->Status = Tx_Request;
	}
	else
	{
		p->NegativeRspReq( pTx,SID_REQUEST_TRANSFER_EXIT,NRC_IncorrectMessageLengthOrInvalidFormat );
	}
	p->TransferCtrl.LoadStatus = TransferCtrlLoad_Idle;
	p->TransferCtrl.Operation = TransferCtrlOperation_Idle;
}
void UdsServiceCtrl_ResponseOnEvent (NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx )
{
	if(pRx->Data[1] == onTimerInterrupt )
	{

	}
}
void TransferCtrl_Init(DataTransferCtrl_t *v, uint16_t BufferSize )
{
	v->Sid = 0 ;
	v->MaxNumberOfBlockLength = MAX_BUFFER_SIZE;
	v->Operation = TransferCtrlOperation_Idle;
	v->LoadStatus = TransferCtrlLoad_Idle;
	v->BlockSequenceCounter = 0;
	v->LengthTotal = 0;
	v->LengthHandled = 0;
	v->BuffSize = BufferSize;
	v->Inform.SizeCode.All = 0;
	v->Inform.FormatCode.All = 0;
	v->WriteToFlash = 0;
	v->ReadFromFlash = 0;
	v->ChecksumCalculate =0;
	v->TargetAddressAvailableCheck=0;
}

void UdsServiceCtrl_DoPLC( NetWorkService_t *v )
{
	v->NetWork.RxDataHandle( &v->NetWork );

	v->ServiceHandler( v, &v->NetWork );
	v->SendDataPeriodically( v, &v->NetWork.Tx, v->PeriodUpdateCtrl.ParamId);
	if( v->NetWork.STCounter++ >= v->NetWork.STminCmd )
	{
		v->NetWork.STCounter = 0;
		v->NetWork.TxDataHandle( &v->NetWork );
	}

	if( v->EnableAutoSend == 1 )
	{
		UdsServiceCtrl_SendDataPeriodicallyNoSwap( v, &v->NetWork.Tx, v->PeriodUpdateCtrl.ParamId);
	}

}

void UdsServiceCtrl_Init ( NetWorkService_t *v, FDCAN_HandleTypeDef *p, UdsSecurityAccessCtrl_t *u )
{
//	v->NetWork.pModuleConfig = m;
//	v->NetWork.pIdConfigTable = t;
	v->NetWork.Init(&v->NetWork,p);
	v->pSecurityCtrl = u;
	v->TransferCtrl.init(&v->TransferCtrl, v->NetWork.Rx.DataSize);
	v->pSecurityCtrl->Init ( v->pSecurityCtrl );
}
