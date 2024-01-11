/*
 * Protocol.c
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */
#if E10
#include "Protocol.h"
#include "ICANInterface.h"
#include <BatCtrl.h>
#include "E10App.h"
//uint8_t VcuEnableFlag=0;
/*
 * "ExtranetInformInSystemTableExample" will be stored in system table bin
 * and the pointer will be loaded in drive_ini when memory control module is ready
 */

uint8_t EscFrameCnt = 0;
uint8_t CAN_RxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t );
uint8_t CAN_TxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r );



const CANProtocol ExtranetInformInSystemTableExample =
{
  50,
  10,
  {
	  CANTXID_ESC_LOG_INFO_0, CANTXID_ESC_LOG_INFO_1, CANTXID_ESC_LOG_INFO_2, CANTXID_ESC_LOG_INFO_3, CANTXID_ESC_LOG_INFO_4,
	  CANTXID_ESC_LOG_INFO_5, CANTXID_ESC_LOG_INFO_6, CANTXID_ESC_LOG_INFO_7, 0, 0
  },
  (pRxTranslate)CAN_RxDataTranslate,
  (pTxTranslate)CAN_TxDataTranslate,
};



/*
 * "LscCanIdTableExtra" will be stored in system table bin
 */
const CanIdConfig_t LscCanIdTableExtra[CAN_ID_CONFIG_ARRAY_SIZE] =
{
  //	Id1,	Id2,	{FilterType,IdType, ConfigUsage,Reserved}
  {CAN_ID_BMS_FILTER,	CAN_ID_BMS_MASK,	{{(uint8_t)FDCAN_FILTER_MASK,CAN_ID_CONIFG_TYPE_EXTENDED,CAN_ID_CONFIG_USED,0,0}}},
  {CAN_ID_DEV_CMD_START,	CAN_ID_DEV_CMD_END,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_EXTENDED,CAN_ID_CONFIG_USED,0,0}}},
  {0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
  {0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
  {0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
  {0x000,	0x000,	{{(uint8_t)FDCAN_FILTER_RANGE,CAN_ID_CONIFG_TYPE_STANDARD,CAN_ID_CONFIG_RESERVED,0,0}}},
};


void ByteOrderReverse(void *Dest, void *Src, uint8_t Size)
{
  uint8_t EndIdx = Size - 1;

  for(uint8_t i = 0; i < Size; i++)
  {
    *(uint8_t*)(Dest + i) = *(uint8_t*)(Src + EndIdx - i);
  }
}





uint8_t CAN_RxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t )
{
  uint8_t	lStatus=ID_MATCH;


  if(IdIn>=CAN_ID_DEV_CMD_START && IdIn <= CAN_ID_DEV_CMD_END)
  {
	  switch (IdIn)
	  {
	  	  case CAN_ID_DEV_CMD_START:

	  		  if(*(pDataIn + 0) == 0)
	  		  {
	  			  BatStation.PwrOnReq();
	  		  }
	  		  else
	  		  {
	  			  BatStation.PwrOffReq();
	  		  }
	  		  break;

	  	  default:
	  		  break;
	  }
  }
  else
  {
	  BatStation.CanMsgLoad(IdIn, pDataIn);
	  v->ReceivedCANID |= RECEIVED_BAT_ID_1;
  }
  return lStatus;
}

uint8_t CAN_TxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r )
{
  uint8_t	lStatus = ID_MATCH;
  EscCanTxCmd_t *p;
  uint8_t lIdx = 0;
  uint16_t u16Temp=0;
  int16_t i16Temp=0;


  for(lIdx=0; lIdx < 8; lIdx++)
  {
    *(pDataIn+lIdx) = 0;	//clear input buffer
  }

  p = (EscCanTxCmd_t*)pDataIn;

  switch (IdIn)
  {
    case CANTXID_ESC_LOG_INFO_0:
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 0)
      {
        lStatus = ID_NO_MATCH;
        break;
      }

      v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] = 0;
      p->EscLogInfo0.MotorTemp = (uint8_t)(v->NTCTemp[0]+40);
      p->EscLogInfo0.EscMos1Temp = (uint8_t)(v->NTCTemp[1]+40);
      p->EscLogInfo0.EscMos2Temp = (uint8_t)(v->NTCTemp[2]+40);
      p->EscLogInfo0.EscCapTemp = (uint8_t)(v->NTCTemp[3]+40);
      p->EscLogInfo0.FoilPosition = v->FoilPos;
      p->EscLogInfo0.TetherSensor = (uint8_t)HAL_GPIO_ReadPin(SAFTYSSR_GPIO_Port, SAFTYSSR_Pin);
      p->EscLogInfo0.FoilSensorVolt = (int8_t)(v->Debugf[IDX_FOIL_SENSOR_VOLT]*10.0f);
      p->EscLogInfo0.ThrottleRaw = r->ThrottleCmd;
      p->EscLogInfo0.ThrottleFinal = (uint8_t)( 100.0f * v->Debugf[IDX_THROTTLE_FINAL]);
      break;
    }
    case CANTXID_ESC_LOG_INFO_1 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        u16Temp = (uint16_t)(v->Debugf[IDX_DC_VOLT] * 10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo1.DcVoltU16, (void*)&u16Temp, 2);

        i16Temp = (int16_t)v->Debugf[IDX_MOTOR_RPM];
        ByteOrderReverse((void*)&p->EscLogInfo1.MotorRpmI16, (void*)&i16Temp, 2);

        p->EscLogInfo1.EscState = v->PcuStateReport;
        p->EscLogInfo1.AlarmFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_CRI_ALARM_MASK)==0) ? 0 : 1;
        p->EscLogInfo1.LimpHomeFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_NON_CRI_ALARM_MASK)==0) ? 0 : 1;
        p->EscLogInfo1.WarnFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_WARNING_MASK)==0) ? 0 : 1;
        p->EscLogInfo1.OutputMode = r->OutputModeCmd;
        p->EscLogInfo1.DeratingSrc = v->DeratingSrc;
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }

      break;
    }
    case CANTXID_ESC_LOG_INFO_2 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {

        i16Temp = (int16_t)(v->Id_cmd*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo2.IdCmdI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Iq_cmd*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo2.IqCmdI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Id_fbk*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo2.IdFbkI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Iq_fbk*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo2.IqFbkI16, (void*)&i16Temp, 2);

      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_ESC_LOG_INFO_3 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        i16Temp = (int16_t)(v->Debugf[IDX_AC_LIMIT_CMD]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo3.AcLimitCmd, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_AC_LIMIT_TQ]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo3.AcLimitTq, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_DC_LIMIT_CMD]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo3.DcLimitCmd, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_DC_LIMIT_TQ]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo3.DcLimitTq, (void*)&i16Temp, 2);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }

      break;
    }
    case CANTXID_ESC_LOG_INFO_4 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        i16Temp = (int16_t)(v->Debugf[IDX_VD_CMD]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo4.VdCmdI16 , (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_VQ_CMD]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo4.VqCmdI16 , (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_PERFROMANCE_TQ]*10.0f);
        ByteOrderReverse((void*)&p->EscLogInfo4.PerformanceTqI16 , (void*)&i16Temp, 2);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_ESC_LOG_INFO_5 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        memcpy(p->EscLogInfo5.AlarmCode, v->DebugError, 8);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_ESC_LOG_INFO_6 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        p->EscLogInfo6.PwrLv = r->PowerLevel;
        p->EscLogInfo6.RcConnStatus = r->RcConnStatus;

        /*todo: assign true value for signals*/
        i16Temp = (int16_t)(v->Debugf[IDX_AVERAGE_AC_POWER]);
        ByteOrderReverse((void*)&p->EscLogInfo6.AvgPwr , (void*)&i16Temp, 2);
        /*todo: assign true value for signals*/
        i16Temp = (int16_t)(v->Debugf[IDX_INSTANT_AC_POWER]);
        ByteOrderReverse((void*)&p->EscLogInfo6.InstPwr , (void*)&i16Temp, 2);
        /*todo: assign true value for signals*/
        u16Temp = (uint16_t)(v->Debugf[IDX_REMAIN_TIME]);
        ByteOrderReverse((void*)&p->EscLogInfo6.TimeRemain , (void*)&u16Temp, 2);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_ESC_LOG_INFO_7 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
    	  p->DataU8[0] = BatStation.MainSMGet();
    	  p->DataU8[1] = BatStation.PwrOffSMGet();
    	  p->DataU8[2] = BatStation.PwrOnSMGet();
        v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] = 1;
      }
      else
      {
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



#endif
