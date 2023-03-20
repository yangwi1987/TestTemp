/*
 * Protocol.c
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#if BME
#include "Protocol.h"
//uint8_t VcuEnableFlag=0;
/*
 * "ExtranetInformInSystemTableExample" will be stored in system table bin
 * and the pointer will be loaded in drive_ini when memory control module is ready
 */
const CANProtocol ExtranetInformInSystemTableExample=
{
		50,
		7,
		{
			0x500, 0x700, 0x701, 0x702, 0x703,
			0x704 ,0x705, 0x00,  0x00,  0x00,
		},
		(pRxTranslate)BME060CAN_RxDataTranslateV0617,
		(pTxTranslate)BME060CAN_TxDataTranslateV0617,
};


/*
 * "LscCanIdTableExtra" will be stored in system table bin
 */
const CanIdConfig_t LscCanIdTableExtra[CAN_ID_CONFIG_ARRAY_SIZE] =
{
//	Id1,	Id2,	{FilterType,IdType, ConfigUsage,Reserved}
//	{0x151,	0x152,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,0,0}}},
//	{0x360,	0x362,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,0,0}}},
	{0x401,	0x402,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,0,0}}},
	{0x600,	0x603,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_USED,0,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
	{0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
};

StructBMSInformV0617 TestV0617;


int16_t CellTemp[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};


uint8_t BME060CAN_RxDataTranslateV0617( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t )
{
		uint8_t	lStatus=ID_MATCH;
		uint16_t ulTemp;
		int16_t lTemp=0;
		UnionRxDataV0617 *plContainer;
		plContainer = (UnionRxDataV0617*)pDataIn;

		memset( (void*)&TestV0617, 0, sizeof(TestV0617) );

		switch (pIdIn)
		{
			case 0x401 :
			{
				ulTemp = plContainer->ID0x401.ErrorCode_H;
				ulTemp <<= 8 ;
				ulTemp += plContainer->ID0x401.ErrorCode_L;
//				TestV0617._401hBmsErrorCode = ulTemp;
//				TestV0617._401hBmsMainStateMachine = plContainer->ID0x401.BmsMainSM;
//				TestV0617._401hBmsPrchStateMachine = plContainer->ID0x401.BmsPrchSM;
//				TestV0617._401hWakeupReason = plContainer->ID0x401.WakeUpReason;

				v->BMSState.BMS = plContainer->ID0x401.BmsMainSM;
				v->BMSState.FET = plContainer->ID0x401.BmsPrchSM;


				if(t->DebugU8[IDX_BMS_COMM_ENABLE] == 1){
					//BMS communication is available, check BMS prch state
					if( ( v->BMSState.BMS == BMS_MAIN_SM_ACTIVE )&&( v->BMSState.FET == BMS_PRCH_SM_DONE ) ){
						v->PrchCtrlFB.bit.BypassMOS = ENABLE;
					} else {
						v->PrchCtrlFB.bit.BypassMOS = DISABLE;
					}
				}else{
					// BMS communication is not available, bypass the BMS prch state check mechanism
					v->PrchCtrlFB.bit.BypassMOS = ENABLE;
				}
				v->ReceivedCANID |= RECEIVED_BAT_ID_1;
				break;

			}
			case 0x601 :{
				CellTemp[0] = plContainer->DataI16[0];
				CellTemp[1] = plContainer->DataI16[1];
				CellTemp[2] = plContainer->DataI16[2];
				CellTemp[3] = plContainer->DataI16[3];
				break;
			}
			case 0x602 :{
				CellTemp[4] = plContainer->DataI16[0];
				CellTemp[5] = plContainer->DataI16[1];
				CellTemp[6] = plContainer->DataI16[2];
				CellTemp[7] = plContainer->DataI16[3];
				break;
			}
			case 0x603 :{
				CellTemp[8] = plContainer->DataI16[0];
				CellTemp[9] = plContainer->DataI16[1];
				lTemp = 0;
				for( uint8_t i=0; i<16; i++ ){
					lTemp = ( CellTemp[i] > lTemp )? CellTemp[i] : lTemp;
					CellTemp[i] = 0;
				}
				lTemp = lTemp/10;
				v->BatTempNow0P1C = lTemp;
				break;
			}
			default :
			{
				lStatus = ID_NO_MATCH;
				break;
			}
		}
		return lStatus;
}

uint8_t BME060CAN_TxDataTranslateV0617( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r )
{
	uint8_t	lStatus=ID_MATCH;
	UnionTxDataV0617 *plContainer;
	uint8_t lIdx=0;

	for( lIdx=0; lIdx<8; lIdx++ )
	{
		*(pDataIn+lIdx)=0;	//clear input buffer
	}

	plContainer = (UnionTxDataV0617*)pDataIn;
	switch (pIdIn)
	{
		case 0x500:
		{
			plContainer->ID0x500.ShutdownReq = 0x00;
			plContainer->ID0x500.ConnReq = 0x01;
			break;
		}
		case 0x700:
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 0){
				lStatus = ID_NO_MATCH;
				break;
			}

			v->DebugU8[IDX_LOG_SAMPLE_FLAG] = 0;
			plContainer->ID0x700.MotorTemp = (uint8_t)(v->NTCTemp[0]+40);
			plContainer->ID0x700.EscMos1Temp = (uint8_t)(v->NTCTemp[1]+40);
			plContainer->ID0x700.EscMos2Temp = (uint8_t)(v->NTCTemp[2]+40);
			plContainer->ID0x700.EscCapTemp = (uint8_t)(v->NTCTemp[3]+40);
			plContainer->ID0x700.OpCmdAndFinal = r->OutputModeCmd +(v->DebugU8[0] <<4);
//			plContainer->ID0x700.ThrottleRaw = (uint16_t)( 1000.0f * v->Debugf[IDX_THROTTLE_RAW]);
			plContainer->ID0x700.ThrottleRaw = r->ThrottleCmd;
			plContainer->ID0x700.ThrottleFinal = (uint8_t)( 100.0f * v->Debugf[IDX_THROTTLE_FINAL]);
			break;
		}
		case 0x701 :
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 1){
				plContainer->ID0x701.DcVoltU16 = (uint16_t)v->Debugf[IDX_DC_VOLT];
				plContainer->ID0x701.MotorRpmI16 = (int16_t)v->Debugf[IDX_MOTOR_RPM];
				plContainer->ID0x701.SafetySensor = (uint8_t)HAL_GPIO_ReadPin(SAFTYSSR_GPIO_Port, SAFTYSSR_Pin);
				plContainer->ID0x701.EscState = v->PcuStateReport;
				plContainer->ID0x701.BmsMainState = r->BMSState.BMS;
				plContainer->ID0x701.BmsPrchState = r->BMSState.FET;

			}else{
				lStatus = ID_NO_MATCH;
			}

			break;
		}
		case 0x702 :
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 1){
				plContainer->ID0x702.IdCmdI16 = (int16_t)(v->Id_cmd*10.0f);
				plContainer->ID0x702.IqCmdI16 = (int16_t)(v->Iq_cmd*10.0f);
				plContainer->ID0x702.IdFbkI16 = (int16_t)(v->Id_fbk*10.0f);
				plContainer->ID0x702.IqFbkI16 = (int16_t)(v->Iq_fbk*10.0f);
			}else{
				lStatus = ID_NO_MATCH;
			}
			break;
		}
		case 0x703 :
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 1){
				plContainer->ID0x703.AcLimitCmd = (int16_t)(v->Debugf[IDX_AC_LIMIT_CMD]*10.0f);
				plContainer->ID0x703.AcLimitTq = (int16_t)(v->Debugf[IDX_AC_LIMIT_TQ]*10.0f);
				plContainer->ID0x703.DcLimitCmd = (int16_t)(v->Debugf[IDX_DC_LIMIT_CMD]*10.0f);
				plContainer->ID0x703.DcLimitTq = (int16_t)(v->Debugf[IDX_DC_LIMIT_TQ]*10.0f);
			}else{
				lStatus = ID_NO_MATCH;
			}

			break;
		}
		case 0x704 :
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 1){
				plContainer->ID0x704.VdCmdI16 = (int16_t)(v->Debugf[IDX_VD_CMD]*10.0f);
				plContainer->ID0x704.VqCmdI16 = (int16_t)(v->Debugf[IDX_VQ_CMD]*10.0f);
				plContainer->ID0x704.PerformanceTqI16 = (int16_t)(v->Debugf[IDX_PERFROMANCE_TQ]*10.0f);
//				plContainer->ID0x704.Error01[0] =  v->DebugError[0];
//				plContainer->ID0x704.Error01[1] =  v->DebugError[1];
				plContainer->DataI16[3] = (int16_t)(v->Debugf[IDX_FOIL_SENSOR_VOLT]*10.0f);
			}else{
				lStatus = ID_NO_MATCH;
			}
			break;
		}
		case 0x705 :
		{
			if(v->DebugU8[IDX_LOG_ENABLE_FLAG] == 1){
				plContainer->ID0x705.Error29[0] =  v->DebugError[0];
				plContainer->ID0x705.Error29[1] =  v->DebugError[1];
				plContainer->ID0x705.Error29[2] =  v->DebugError[2];
				plContainer->ID0x705.Error29[3] =  v->DebugError[3];
				plContainer->ID0x705.Error29[4] =  v->DebugError[4];
				plContainer->ID0x705.Error29[5] =  v->DebugError[5];
				plContainer->ID0x705.Error29[6] =  v->DebugError[6];
				plContainer->ID0x705.Error29[7] =  v->DebugError[7];
//				plContainer->DataI16[0] = (int16_t)(v->Debugf[IDX_DC_LIMIT_CANRX_DC_CURR]);
//				plContainer->DataI16[1] = (int16_t)(v->Debugf[IDX_DC_LIMIT_CANRX_DC_REGEN]);
//				plContainer->DataI16[2] = v->IU0P1A;
//				plContainer->DataU8[6] = v->DebugU8[1];
				v->DebugU8[IDX_LOG_SAMPLE_FLAG] = 1;
			}else{
				lStatus = ID_NO_MATCH;
			}
			break;
		}
		default :
		{
			lStatus = ID_NO_MATCH;
			break;
		}
	}
	return lStatus;
}


StructBMSInform Test;
uint8_t BME060CAN_RxDataTranslate( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t )
{
	uint8_t	lStatus=ID_MATCH;

	UnionRxData *plContainer;
	plContainer = (UnionRxData*)pDataIn;

	int16_t wTemp=0;
	uint16_t uwTemp=0;

	memset( (void*)&Test, 0, sizeof(Test) );

	switch (pIdIn)
	{
		case 0x151:
		{
			Test._151hWarning = plContainer->ID0x151.BMSWarning;
			Test._151hSafetyFailure = plContainer->ID0x151.BMSSafetyFailure;
			Test._151hFailure= plContainer->ID0x151.BMSFailure;
			Test._151hChargeFinished = plContainer->ID0x151.BMSCHGFinished;
			Test._151hRecuperationAllowed = plContainer->ID0x151.BMSRecuperationAllow;

			wTemp =0;
			wTemp |= (plContainer->ID0x151.BMSCurrentNow_H8<<8);
			wTemp |= plContainer->ID0x151.BMSCurrentNow_L;
			Test._151hBatCurrentNow = ((float)wTemp)*0.2;

			uwTemp =0;
			uwTemp |= (plContainer->ID0x151.BMSVoltNow_H5 <<5);
			uwTemp |= plContainer->ID0x151.BMSVoltNow_L;
			Test._151hBatVoltNow = ((float)uwTemp)*0.01;
			break;

		}
		case 0x152:
		{
			Test._152hBMSState = plContainer->ID0x152.BMSstate_L+ (plContainer->ID0x152.BMSstate_H1<<1);
			Test._152hFETStatus = plContainer->ID0x152.BMSFETstate;
			Test._152hInterConnReady = plContainer->ID0x152.BMSMultiInterconReady;
			Test._152hPackQuantity = plContainer->ID0x152.BMSPackQuantity + 1 ;
			Test._152hPwrFactorCHG = plContainer->ID0x152.PwrFactorCHG;
			Test._152hPwrFactorDCHG = ( plContainer->ID0x152.PwrFactorDCHG_H6 <<6 ) + plContainer->ID0x152.PwrFactorDCHG_L;
			Test._152hWakeupMethod = plContainer->ID0x152.WakeupMethod;
			break;
		}
		case 0x361:
		{
			Test._361hCurrentCHGLimit = plContainer->ID0x361.BMSCurrentCHG;

			wTemp = 0;
			wTemp |= plContainer->ID0x361.BMSCurrentDCHG_H1 << 1;
			wTemp |= plContainer->ID0x361.BMSCurrentDCHG_L;
			Test._361hCurrentDCHGLimit = wTemp - 511;

			Test._361hSOC = plContainer->ID0x361.BMSSoc_L + ( plContainer->ID0x361.BMSSoc_H2 << 2 ) ;

			uwTemp = 0;
			uwTemp |= plContainer->ID0x361.BMSVoltCHG_H6 << 6;
			uwTemp |= plContainer->ID0x361.BMSVoltCHG_L;

			Test._361hVoltCHG = ((float)uwTemp)*0.01;

			uwTemp = 0;
			uwTemp |= plContainer->ID0x361.BMSVoltDCHG_H11;
			uwTemp<<=11;
			uwTemp |= plContainer->ID0x361.BMSVoltDCHG_H3<<3;
			uwTemp |= plContainer->ID0x361.BMSVoltDCHG_L;

			Test._361hVoltDCHG = ((float)uwTemp)*0.01;
			break;
		}
		default :
		{
			lStatus = ID_NO_MATCH;
			 break;
		}
	}

	return lStatus;
}


uint8_t BME060CAN_TxDataTranslate( uint32_t pIdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r )
{
	uint8_t	lStatus=ID_MATCH;
	UnionTxData *plContainer;
	uint8_t lIdx=0;

	for( lIdx=0; lIdx<8; lIdx++ )
	{
		*(pDataIn+lIdx)=0;	//clear input buffer
	}

	plContainer = (UnionTxData*)pDataIn;
	switch (pIdIn)
	{
		case 0x150:
		{
			plContainer->ID0x150.VcuShutdownReq = v->ShutDownReq;
			break;
		}
		default :
		{
			lStatus = ID_NO_MATCH;
			break;
		}
	}
	return lStatus;
}
#endif
