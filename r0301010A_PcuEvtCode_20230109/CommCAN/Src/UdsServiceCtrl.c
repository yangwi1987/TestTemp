/*
 * UdsServiceCtrl.c
 *
 *  Created on: 2020年5月28日
 *      Author: Will.Yang.CYY
 */

#include "UdsServiceCtrl.h"
#include "string.h"
#include "RcUartComm.h"


__STATIC_FORCEINLINE void UdsServiceCtrlBRP_DSC( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_ER( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_SA( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_TP( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_RDBI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
//__STATIC_FORCEINLINE void UdsServiceCtrlBRP_WDBI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_CDTCI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_RDTCI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m );

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

EnumUdsBRPNRC UdsServiceCtrl_ReadDataRegionF( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, uint16_t ParamID )
{
	uint8_t i = 0;
	uint16_t UDSDataBuf[MAX_UDS_DATA_BUF] = {0};
	uint8_t *pPt;
	uint16_t DIDDataLength = 0; // the length of DID data(bytes), excluding first 3 bytes(SID, ParamID)
	EnumUdsBRPNRC NrcRet = NRC_0x00_PR;

	switch( ParamID )
	{
		case 0x089:	// HW Version Read
			pPt = (uint8_t*)&HWVerNumber;
			DIDDataLength = HW_VER_NUM_IDX;
			break;
		case 0x186:	// Active Diagnostic Session
			pPt = (uint8_t*)&(p->pParamMgr->Session);
			DIDDataLength = 1;
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

		case 0x1F2:
			
			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RF_FW_VERSION) !=0)
			{
				pPt = (uint8_t*)RCCommCtrl.RFFwVer;
				DIDDataLength = RC_COMM_RF_FW_VER_SIZE;
			}
			else
			{
				NrcRet = NRC_0x22_CNC;
			}

			break;

		case 0x1F3:

			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RF_SN) !=0)
			{
				pPt = (uint8_t*)RCCommCtrl.RFSN;
				DIDDataLength = RC_COMM_RF_SN_SIZE;
			}
			else
			{
				NrcRet = NRC_0x22_CNC;
			}

			break;

		case 0x1F4:

			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RC_FW_VERSION) != 0)
			{
				pPt = (uint8_t*)RCCommCtrl.RCFwVer;
				DIDDataLength = RC_COMM_RC_FW_VER_SIZE;
			}
			else
			{
				NrcRet = NRC_0x22_CNC;
			}

			break;

		case 0x1F5:

			if((RCCommCtrl.RcInfoQueryCompleteFlag & RC_COMM_RC_INFO_QUERY_COMPLETE_FLAG_MASK_RC_SN) != 0)
			{
				pPt = (uint8_t*)RCCommCtrl.RCSN;
				DIDDataLength = RC_COMM_RC_SN_SIZE;
			}
			else
			{
				NrcRet = NRC_0x22_CNC;
			}

			break;

		default :
			NrcRet = NRC_0x31_ROOR;
			break;
	}

	if(NrcRet == NRC_0x00_PR)
	{
		for( i = 0; i < DIDDataLength; i++ )
		{
			pTx->Data[i + 3] = *(pPt + i);
		}
		pTx->LengthTotal = (DIDDataLength + 3);
	}
	
	return NrcRet;
}

void UdsServiceCtrl_ServiceHandler( NetWorkService_t *p ,NetworkCtrl_t *v  )
{
	uint8_t lSID = 0;
	if( v->Rx.Status == Rx_Complete)
	{
		v->Rx.Status = Rx_Reading;
		p->SessionCNT = 0;
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
    	    	if ( p->pParamMgr->Session != Session_0x01_Default )
    	    	{
    				UdsServiceCtrl_SecurityAccess( p, &v->Rx, &v->Tx );
    	    	}
    	    	else
    	    	{
    	    		p->NegativeRspReq( &v->Tx, v->Rx.Data[0], NRC_serviceNotSupportedInActiveSession );
    	    	}
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

			ReplyResult = UdsServiceCtrl_ReadDataRegionF( p, pRx, pTx, ParamID );

			if( ReplyResult == NRC_0x00_PR)
			{
				// assign data from specific data address and set length of total data if access success
				pTx->Data[0] += POSITIVE_RESPONSE_OFFSET;
				pTx->Status = Tx_Request;
			}
			else
			{
				// invalid service request,  report NRC to client
				p->NegativeRspReq( pTx, SID_READ_DATA_BY_ID, ReplyResult );
			}
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

	p->pParamMgr->Security = p->pSecurityCtrl->SecureLvNow;

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
		p->pParamMgr->NextSession = Session_0x01_Default;
		p->SessionCNTEnable = 0;
		p->SessionCNT = 0;
		break;
	case Session_0x02_Programming:
		p->pParamMgr->NextSession = Session_0x02_Programming;
		break;
	case Session_0x03_ExtendedDiagnostic:
		p->pParamMgr->NextSession = Session_0x03_ExtendedDiagnostic;
		p->SessionCNTEnable = 1;
		break;
	case Session_0x04_SafetySystemDiagnostic:
		p->pParamMgr->NextSession = Session_0x04_SafetySystemDiagnostic;
		p->SessionCNTEnable = 1;
		break;
	case Session_0x40_VehicleManufacturerSpecific:
		p->pParamMgr->NextSession = Session_0x40_VehicleManufacturerSpecific;
		p->SessionCNTEnable = 1;
		break;
	case Session_0x60_SystemSupplierSpecific:
		p->pParamMgr->NextSession = Session_0x60_SystemSupplierSpecific;
		p->SessionCNTEnable = 1;
		break;
	default:
		p->pParamMgr->NextSession = Session_0x01_Default;
		p->SessionCNTEnable = 0;
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
	if ( p->pSecurityCtrl->IsFalseAccessExceedLimit == TRUE )
	{
		p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_requiredTimeDelayNotExpired );
	}
	else
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
        		case UDS_SECURE_RESULT_FAIL_FALSE_ACCESS_EXCEED_LIMIT:
	    			p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_exceededNumberOfAttempts );
        			break;
	    		default:
	    			p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_ConditionsNotCorrect );
	    			break;
	    	}
	    }
	}
}

void UdsServiceCtrl_ECUReset(NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx)
{
	if ( p->ServiceCtrlBRP.ServoOnOffState == 1)
	{
		p->NegativeRspReq ( pTx, SID_SECURITY_ACCESS, NRC_ConditionsNotCorrect );
	}
	else
	{
		p->ECUSoftResetEnable = 1;
		pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
		pTx->Data[1] = pRx->Data[1];
		pTx->LengthTotal = 2;
		pTx->Status = Tx_Request;
	}
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

	if( (v->NetWork.Rx.CanId >=UDS_RX_ID_BRP_FUNCTIONAL_START ) && (v->NetWork.Rx.CanId <= UDS_RX_ID_BRP_FUNCTIONAL_END ))
	{
        v->ServiceCtrlBRP.ServiceHandler_Functional( v, &v->NetWork, &v->ServiceCtrlBRP );
	}
	else if( (v->NetWork.Rx.CanId >=UDS_RX_ID_BRP_PHYSICAL_START ) && (v->NetWork.Rx.CanId <= UDS_RX_ID_BRP_PHYSICAL_END ))
	{
        v->ServiceCtrlBRP.ServiceHandler_Physical( v, &v->NetWork, &v->ServiceCtrlBRP );
	}
	else
	{
     	    v->ServiceHandler( v, &v->NetWork );
	}
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

	// LSC original session
	if ( v->SessionCNTEnable )
	{
		if ( v->SessionCNT >= MF_SESSION_MAX_MS ) // todo modify session time out second
		{
			v->pParamMgr->NextSession = Session_0x01_Default;
		    v->SessionCNTEnable = 0;
		    v->SessionCNT = 0;
		    v->pSecurityCtrl->SecureLvNow = SECURITY_LEVEL_0;
		}
		else
		{
			v->SessionCNT++;
		}
	}

	// BRP session
	if ( v->ServiceCtrlBRP.BRPSessionCNTEnable )
	{
		if ( v->ServiceCtrlBRP.BRPSessionCNT >= P2_STAR_SERVER_MAX_MS )
		{
			v->ServiceCtrlBRP.DiagnosticSession = Session_0x01_DS;
		    v->ServiceCtrlBRP.BRPSessionCNTEnable = 0;
		    v->ServiceCtrlBRP.BRPSessionCNT = 0;
		    v->pSecurityCtrl->SecureLvNow = SECURITY_LEVEL_0;
		}
		else
		{
			v->ServiceCtrlBRP.BRPSessionCNT++;
		}
	}

	if ( v->pSecurityCtrl->IsFalseAccessExceedLimit == TRUE )
	{
		if ( v->ServiceCtrlBRP.FalseAccessExceedLimitCNT >= FALSE_ACCESS_EXCEED_LIMIT_DELAY_TIME_MS )
		{
			v->pSecurityCtrl->IsFalseAccessExceedLimit = FALSE;
			v->ServiceCtrlBRP.FalseAccessExceedLimitCNT = 0;
		}
		else
		{
            v->ServiceCtrlBRP.FalseAccessExceedLimitCNT++;
		}
	}

}

void UdsServiceCtrl_Init ( NetWorkService_t *v, FDCAN_HandleTypeDef *p, UdsSecurityAccessCtrl_t *u )
{
//	v->NetWork.pModuleConfig = m;
//	v->NetWork.pIdConfigTable = t;
	v->NetWork.Init(&v->NetWork,p);
	v->pSecurityCtrl = u;
	v->ServiceCtrlBRP.pSecurityCtrl = u;
	v->TransferCtrl.init(&v->TransferCtrl, v->NetWork.Rx.DataSize);
	v->pSecurityCtrl->Init ( v->pSecurityCtrl );
}
/*
 * Functions for BRP UDS ===========================================================================================================================================
 */


void UdsServiceCtrlBRP_ServiceHandler_Functional( NetWorkService_t *p, NetworkCtrl_t *v, ServiceCtrlBRP_t *m )
{

    if( v->Rx.Status == Rx_Complete)
    {
        EnumUdsServiceBRPIdentifier lSID = 0;
	    m->BRPSessionCNT = 0;
    	v->Rx.Status = Rx_Reading;
        m->Response_Code = NRC_0x10_GR;
        m->SuppressPosRspMsgIndicationBit = FALSE;
    	lSID = v->Rx.Data[0];
    	switch (lSID)
    	{
    	    case SID_0x3E_TP_with_SF:
    	    {
    	    	if ( m->DiagnosticSession == Session_0x01_DS ) //|| ( m->DiagnosticSession == Session_0x02_PRGS ))
    	    	{
    	    	    UdsServiceCtrlBRP_TP( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    		    break;
    	    }
    	    case SID_0x22_RDBI_without_SF:
    	    {
    	    	if ( m->DiagnosticSession == Session_0x01_DS ) //|| ( m->DiagnosticSession == Session_0x02_PRGS ))
    	    	{
    	    	    UdsServiceCtrlBRP_RDBI( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    		    break;
    	    }
    	    case SID_0x14_CDTCI_without_SF:
    	    {
    	    	if ( m->DiagnosticSession == Session_0x01_DS )
    	    	{
    	    	    UdsServiceCtrlBRP_CDTCI( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    		    break;
    	    }
    	    case SID_0x19_RDTCI_with_SF:
    	    {
    	    	if ( m->DiagnosticSession == Session_0x01_DS )
    	    	{
    	    	    UdsServiceCtrlBRP_RDTCI( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    		    break;
    	    }
    	    default:
    		{
        		m->Response_Code = NRC_0x11_SNS;
    	    	break;
  		    }
     	}
      	v->Rx.Status = Rx_Idle;
    	if ( m->Response_Code == NRC_0x00_PR )
    	{
    		if ( m->SuppressPosRspMsgIndicationBit == TRUE )
    		{
    		    v->Tx.Status = Tx_Idle;
    		}
    	    else
    	    {
    		    v->Tx.Status = Tx_Request;
    	    }
    	}
    	else if (( m->Response_Code == NRC_0x11_SNS ) || ( m->Response_Code == NRC_0x12_SFNS ) || ( m->Response_Code == NRC_0x7F_SNSIAS )\
    		    || ( m->Response_Code == NRC_0x7E_SFNSIAS ) || ( m->Response_Code == NRC_0x31_ROOR ))
    	{
    	    v->Tx.Status = Tx_Idle;
    	}
    	else
    	{
    		v->Tx.Data[0] = 0x7F;
    		v->Tx.Data[1] = lSID;
    		v->Tx.Data[2] = m->Response_Code;
    		v->Tx.LengthTotal = 3;
    		v->Tx.Status = Tx_Request;
    	}
    }

}

void UdsServiceCtrlBRP_ServiceHandler_Physical( NetWorkService_t *p, NetworkCtrl_t *v, ServiceCtrlBRP_t *m  )
{

    if( v->Rx.Status == Rx_Complete)
    {
        EnumUdsServiceBRPIdentifier lSID = 0;
	    m->BRPSessionCNT = 0;
    	v->Rx.Status = Rx_Reading;
        m->Response_Code = NRC_0x10_GR;
        m->SuppressPosRspMsgIndicationBit = FALSE;
    	lSID = v->Rx.Data[0];
    	switch (lSID)
    	{
            case SID_0x10_DSC_with_SF:
            {
            	UdsServiceCtrlBRP_DSC( &v->Rx, &v->Tx, m );
    	        break;
            }
            case SID_0x11_ER_with_SF:
            {
            	UdsServiceCtrlBRP_ER( &v->Rx, &v->Tx, m );
    	        break;
            }
            case SID_0x27_SA_with_SF:
            {
    	    	if ( m->DiagnosticSession == Session_0x03_EXTDS ) //|| ( m->DiagnosticSession == Session_0x02_PRGS ))
    	    	{
    	    	    UdsServiceCtrlBRP_SA( &v->Rx, &v->Tx, m );
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    	        break;
            }
            case SID_0x3E_TP_with_SF:
            {
            	UdsServiceCtrlBRP_TP( &v->Rx, &v->Tx, m );
    	        break;
            }
            case SID_0x22_RDBI_without_SF:
            {
            	UdsServiceCtrlBRP_RDBI( &v->Rx, &v->Tx, m);
    	        break;
            }
//            case SID_0x2E_WDBI_without_SF:
//            {
//            	UdsServiceCtrlBRP_WDBI( &v->Rx, &v->Tx, m);
//    	        break;
//            }
            case SID_0x14_CDTCI_without_SF:
            {
    	    	if (( m->DiagnosticSession == Session_0x01_DS ) || ( m->DiagnosticSession == Session_0x03_EXTDS ))
    	    	{
    	    	    UdsServiceCtrlBRP_CDTCI( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    	        break;
            }
            case SID_0x19_RDTCI_with_SF:
            {
    	    	if (( m->DiagnosticSession == Session_0x01_DS ) || ( m->DiagnosticSession == Session_0x03_EXTDS ))
    	    	{
    	    	    UdsServiceCtrlBRP_RDTCI( &v->Rx, &v->Tx, m);
    	    	}
    	    	else
    	    	{
    	    		m->Response_Code = NRC_0x7F_SNSIAS;
    	    	}
    	        break;
            }
//            case SID_0x2F_IOCBI_without_SF:
//            {
//            	UdsServiceCtrlBRP_IOCBI( &v->Rx, &v->Tx, m);
//	            break;
//            }
//            case SID_0x31_RC_with_SF:
//            {
//            	UdsServiceCtrlBRP_RC( &v->Rx, &v->Tx, m);
//	            break;
//            }
//            case SID_0x34_RD_without_SF:
//            {
//            	m->Response_Code = NRC_0x7F_SNSIAS;
//	            break;
//            }
//            case SID_0x36_TD_without_SF:
//            {
//            	m->Response_Code = NRC_0x7F_SNSIAS;
//	            break;
//            }
//            case SID_0x37_RTE_without_SF:
//            {
//            	m->Response_Code = NRC_0x7F_SNSIAS;
//	            break;
//            }
            default:
    	    {
    	    	m->Response_Code = NRC_0x11_SNS;
            	break;
    	    }

	    }
	    v->Rx.Status = Rx_Idle;

	    if ( m->Response_Code == NRC_0x00_PR )
	    {
	    	if ( m->SuppressPosRspMsgIndicationBit == TRUE )
	    	{
	    	    v->Tx.Status = Tx_Idle;
	    	}

	        else
	        {
	    	    v->Tx.Status = Tx_Request;
	        }
	    }
	    else
	    {
	    	v->Tx.Data[0] = 0x7F;
	    	v->Tx.Data[1] = lSID;
	    	v->Tx.Data[2] = m->Response_Code;
	    	v->Tx.LengthTotal = 3;
	    	v->Tx.Status = Tx_Request;
	    }
    }
}
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_DSC( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{
	if ( pRx->LengthTotal == 2)
	{
		Union_UdsDataParameter DataParameter;
		DataParameter.All = pRx->Data[1];
	    m->SuppressPosRspMsgIndicationBit = DataParameter.Bits.SuppressPosRspMsgIndicationBit;
		switch (DataParameter.Bits.SunFunctionType)
		{
		    case Session_0x01_DS:
		    {
		    	m->DiagnosticSession = Session_0x01_DS;
		    	m->BRPSessionCNTEnable = 0;
		    	m->BRPSessionCNT = 0;
		    	m->pSecurityCtrl->SecureLvNow = DEFAULT_SECURITY_LEVEL; // todo Para default security level
		    	m->Response_Code = NRC_0x00_PR;
		    	break;
		    }
		    case Session_0x02_PRGS:
		    {
		    	if ( m->ServoOnOffState == 1)
		    	{
		        	m->Response_Code = NRC_0x22_CNC;
		    	}
		    	else
		    	{
		    		BootAppTrig = BOOT_ENA;
		    		m->pSecurityCtrl->SecureLvNow = DEFAULT_SECURITY_LEVEL; // todo Para default security level
		    		m->Response_Code = NRC_0x00_PR;
		    	}
		    	break;
		    }
		    case Session_0x03_EXTDS:
		    {
		    	m->DiagnosticSession = Session_0x03_EXTDS;
		    	m->BRPSessionCNTEnable = 1;
		    	m->pSecurityCtrl->SecureLvNow = DEFAULT_SECURITY_LEVEL; // todo Para default security level
		    	m->Response_Code = NRC_0x00_PR;
		    	break;
		    }
		    default:
		    {
                m->Response_Code = NRC_0x12_SFNS;
		    }
		}
	}
	else
	{
		m->Response_Code = NRC_0x13_IMLOIF;
	}

	if ( m->Response_Code == NRC_0x00_PR )
	{
	    pTx->Data[0] = pRx->Data[0]+POSITIVE_RESPONSE_OFFSET;
	    pTx->Data[1] = pRx->Data[1];
	    pTx->Data[2] = ( P2_SERVER_MAX >> 8 );
	    pTx->Data[3] = ( P2_SERVER_MAX & 0xFF );
	    pTx->Data[4] = ( P2_STAR_SERVER_MAX >> 8 );
	    pTx->Data[5] = ( P2_STAR_SERVER_MAX & 0xFF );
	    pTx->LengthTotal=6;
	}
	else
	{
        ;;/*Do nothing*/
	}
}

__STATIC_FORCEINLINE void UdsServiceCtrlBRP_ER( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{

	if ( pRx->LengthTotal == 2)
	{
	    Union_UdsDataParameter DataParameter;
	    DataParameter.All = pRx->Data[1];
	    m->SuppressPosRspMsgIndicationBit = DataParameter.Bits.SuppressPosRspMsgIndicationBit;
	    switch (DataParameter.Bits.SunFunctionType)
	    {
	        case Soft_Reset:
	        {
		    	if ( m->ServoOnOffState == 1)
		    	{
		        	m->Response_Code = NRC_0x22_CNC;
		    	}
		    	else
		    	{
		    		m->BRPECUSoftResetEnable = 1;
	        	    m->Response_Code = NRC_0x00_PR;
	        	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
	        	    pTx->Data[1] = pRx->Data[1];
	        	    pTx->LengthTotal = 2;
		    	}
	        	break;
	        }
	        default:
	        {
	        	m->Response_Code = NRC_0x12_SFNS;
	        	break;
	        }

	    }
	}
	else
	{
		m->Response_Code = NRC_0x13_IMLOIF;
	}

}
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_SA( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{
	if ( pRx->LengthTotal == 2 + UDS_SECURE_SEED_SIZE ) /*To be discuss with BRP*/
	{
	    Union_UdsDataParameter DataParameter;
	    DataParameter.All = pRx->Data[1];
	    if (( DataParameter.Bits.SuppressPosRspMsgIndicationBit == TRUE ) && (( DataParameter.Bits.SunFunctionType & 0x01 ) == 1 ))
	    {
	    	m->Response_Code = NRC_0x22_CNC;
	    }
	    else if (( pRx->Data[2] || pRx->Data[3] ) && (( DataParameter.Bits.SunFunctionType & 0x01 ) == 1 ))
	    {
	    	m->Response_Code = NRC_0x31_ROOR;
	    }
	    else
	    {
	    	if ( m->pSecurityCtrl->IsFalseAccessExceedLimit == TRUE )
	    	{
	    		m->Response_Code = NRC_0x37_RTDNE;
	    	}
	    	else
	    	{
	            m->SuppressPosRspMsgIndicationBit = DataParameter.Bits.SuppressPosRspMsgIndicationBit;
	            m->pSecurityCtrl->SubFuncIn = DataParameter.Bits.SunFunctionType;
	            m->pSecurityCtrl->SubFuncCheck ( m->pSecurityCtrl, &pRx->Data[2], &pTx->Data[2] );

	            if ( m->pSecurityCtrl->SecureState == UDS_SECURE_STATE_SEND_SEED )
	            {
	            	pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
	            	pTx->Data[1] = m->pSecurityCtrl->SubFuncIn;
	            	pTx->LengthTotal = 2 + UDS_SECURE_SEED_SIZE;
	                m->Response_Code = NRC_0x00_PR;
	            	m->pSecurityCtrl->SecureState = UDS_SECURE_STATE_WAIT_KEY;
	            }
	            else if ( (m->pSecurityCtrl->SecureState == UDS_SECURE_STATE_CHECKED) && (m->pSecurityCtrl->SecureResult == UDS_SECURE_RESULT_SUCCESS) )
	            {
	            	pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
	            	pTx->Data[1] = m->pSecurityCtrl->SubFuncIn;
	            	pTx->LengthTotal = 2;
	                m->Response_Code = NRC_0x00_PR;
	            }
	            else
	            {
	            	switch ( m->pSecurityCtrl->SecureResult )
	            	{
	            		case UDS_SECURE_RESULT_FAIL_WRONG_SEQ:
	            			m->Response_Code = NRC_0x24_RSE;
	            			break;
	            		case UDS_SECURE_RESULT_FAIL_WRONG_KEY:
	            			m->Response_Code = NRC_0x35_IK;
	            			break;
	            		case UDS_SECURE_RESULT_FAIL_SUB_FUNCTION_VALUE_MISMATCH:
	            			m->Response_Code = NRC_0x22_CNC;
	            			break;
	            		case UDS_SECURE_RESULT_FAIL_UNSUPPORT_SECURE_LEVEL:
	            			m->Response_Code = NRC_0x12_SFNS;
	            			break;
	            		case UDS_SECURE_RESULT_FAIL_FALSE_ACCESS_EXCEED_LIMIT:
	            			m->Response_Code = NRC_0x36_ENOA;
	            			break;
	            		default:
	            			m->Response_Code = NRC_0x22_CNC;
	            			break;
	            	}
	            }
	        }
	    }
	}
	else
	{
		m->Response_Code = NRC_0x13_IMLOIF;
	}

}

__STATIC_FORCEINLINE void UdsServiceCtrlBRP_TP( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{
	if ( pRx->LengthTotal == 2)
	{
	    Union_UdsDataParameter DataParameter;
	    DataParameter.All = pRx->Data[1];
	    m->SuppressPosRspMsgIndicationBit = DataParameter.Bits.SuppressPosRspMsgIndicationBit;
	    switch (DataParameter.Bits.SunFunctionType)
	    {
	        case NONE:
	        {
	        	m->Response_Code = NRC_0x00_PR;
        	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
        	    pTx->Data[1] = pRx->Data[1];
        	    pTx->LengthTotal = 2;
	        	break;
	        }
	        default:
	        {
	        	m->Response_Code = NRC_0x12_SFNS;
	        	break;
	        }
	    }
	}
	else
	{
		m->Response_Code = NRC_0x13_IMLOIF;
	}

}
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_RDBI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{
	if ( pRx->LengthTotal == 3)
	{
		UdsDIDParameter_e DID = ( pRx->Data[1] << 8 ) + pRx->Data[2];
        m->Response_Code = m->RDBI_Function ( DID, pRx, pTx );
	}
	else
	{
		m->Response_Code = NRC_0x13_IMLOIF;
	}

}
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_CDTCI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{

    if (pRx->LengthTotal == 4 )
    {
    	uint32_t temp_qroupOfDTC = 0;
    	temp_qroupOfDTC = ( pRx->Data[1] << 16 ) + ( pRx->Data[2] << 8 ) + pRx->Data[2];
	    if ( temp_qroupOfDTC == 0x00FFFFFF )
	    {
	    	if ( m->pDTCStation->State == DTC_Process_State_Idle )
	    	{
            m->pDTCStation->State = DTC_Process_State_Clear;
	    	m->Response_Code = NRC_0x78_RCRRP;
	    	}
	    	else
	    	{
		    	m->Response_Code = NRC_0x22_CNC;
	    	}
	    }
	    else
	    {
	    	m->Response_Code = NRC_0x31_ROOR;
	    }
    }
    else
    {
    	m->Response_Code = NRC_0x13_IMLOIF;
    }

}
__STATIC_FORCEINLINE void UdsServiceCtrlBRP_RDTCI( LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ServiceCtrlBRP_t *m )
{
    if (pRx->LengthTotal == 3 )
    {
		Union_UdsDataParameter DataParameter;
		DataParameter.All = pRx->Data[1];
	    m->SuppressPosRspMsgIndicationBit = DataParameter.Bits.SuppressPosRspMsgIndicationBit;
	    if ( m->SuppressPosRspMsgIndicationBit == 0 )
	    {
			switch ( DataParameter.Bits.SunFunctionType )
			{
			    case reportDTCByStatusMask:
			    {
			    	uint8_t respond_number_by_Status_Mask = 0;
			    	for ( uint8_t i = 0; i < DTC_RecordNumber_Total; i++ )
			    	{
			    		if ( *(uint8_t*)&(m->pDTCStation->StatusOfDTC_Realtime[i]) & *(uint8_t*)&(m->pDTCStation->StatusAvailabilityMask) & pRx->Data[2] )
			    		{
                            pTx->Data[ (4 * respond_number_by_Status_Mask) + 3] = m->pDTCStation->DTC_Code[i] >> 8;
                            pTx->Data[ (4 * respond_number_by_Status_Mask) + 4] = m->pDTCStation->DTC_Code[i] & 0xFF;
                            pTx->Data[ (4 * respond_number_by_Status_Mask) + 5] = 0;
                            pTx->Data[ (4 * respond_number_by_Status_Mask) + 6] = *(uint8_t*)&(m->pDTCStation->StatusOfDTC_Realtime[i]);
                            respond_number_by_Status_Mask ++;
			    	    }
			    	}
			    	m->Response_Code = NRC_0x00_PR;
	        	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
	        	    pTx->Data[1] = pRx->Data[1];
	        	    pTx->Data[2] = *(uint8_t*)&(m->pDTCStation->StatusAvailabilityMask);

	        	    pTx->LengthTotal = 3 + ( respond_number_by_Status_Mask * 4 );

			        break;
			    }
			    case reportDTCStoredDataByRecordNumber:
			    {
			    	if ( pRx->Data[2] == 0xFF )
			    	{
                        uint16_t final_Tx_buffer_addr = 2;
    			    	for ( uint8_t i = 0; i < DTC_RecordNumber_Total; i++ )
    			    	{
    			    		if ( m->pDTCStation->DTCRespondPackgeFirst[i].DTCId.DTCStoredDataRecordNumber != 0 )
    			    		{
    			    			m->pDTCStation->DTCRespondPackgeFirst[i].StatusOfDTC = m->pDTCStation->StatusOfDTC_Realtime[i];
    			    			memcpy( &(pTx->Data[ final_Tx_buffer_addr ]), &(m->pDTCStation->DTCRespondPackgeFirst[i]), DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
    			    			final_Tx_buffer_addr = final_Tx_buffer_addr +  DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset;
    			    		}
    			    		else
    			    		{
    			    			continue;
    			    		}
    			    		if ( m->pDTCStation->DTCRespondPackgeLast[i].DTCId.DTCStoredDataRecordNumber != 0 )
    			    		{
    			    			m->pDTCStation->DTCRespondPackgeLast[i].StatusOfDTC = m->pDTCStation->StatusOfDTC_Realtime[i];
    			    			memcpy( &(pTx->Data[ final_Tx_buffer_addr ]), &(m->pDTCStation->DTCRespondPackgeLast[i]), DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
    			    			final_Tx_buffer_addr = final_Tx_buffer_addr +  DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset;
    			    		}
    			    	}

				    	m->Response_Code = NRC_0x00_PR;
		        	    pTx->Data[0] = pRx->Data[0] + POSITIVE_RESPONSE_OFFSET;
		        	    pTx->Data[1] = pRx->Data[1];

		        	    pTx->LengthTotal = final_Tx_buffer_addr;
			    	}
			    	else
			    	{
				    	m->Response_Code = NRC_0x31_ROOR;
			    	}
			    	break;
			    }
			    default:
			    {
			    	m->Response_Code = NRC_0x12_SFNS;
			    }
			}
	    }
	    else
	    {
	    	m->Response_Code = NRC_0x22_CNC;
	    }

    }
    else
    {
    	m->Response_Code = NRC_0x13_IMLOIF;
    }
}
