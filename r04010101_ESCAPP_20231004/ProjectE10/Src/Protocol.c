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

uint8_t InvFrameCnt = 0;
uint8_t CAN_RxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t );
uint8_t CAN_TxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r );



const CANProtocol ExtranetInformInSystemTableExample =
{
  50,
  10,
  {
	  CANTXID_INV_LOG_INFO_0, CANTXID_INV_LOG_INFO_1, CANTXID_INV_LOG_INFO_2, CANTXID_INV_LOG_INFO_3, CANTXID_INV_LOG_INFO_4,
	  CANTXID_INV_LOG_INFO_5, CANTXID_INV_LOG_INFO_6, CANTXID_INV_LOG_INFO_7, CANTXID_INV_LOG_INFO_8, CANTXID_INV_LOG_INFO_9
  },
  (pRxTranslate)CAN_RxDataTranslate,
  (pTxTranslate)CAN_TxDataTranslate,
};



/*
 * "LscCanIdTableExtra" will be stored in system table bin
 */
const CanIdConfig_t CanIdTableExtra[CAN_ID_CONFIG_ARRAY_SIZE] =
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
	  		  Btn_SignalWrite(BTN_IDX_KILL_SW, *pDataIn);
	  		  Btn_SignalWrite(BTN_IDX_BST_BTN, *(pDataIn + 1));
	  		  Btn_SignalWrite(BTN_IDX_REV_BTN, *(pDataIn + 2));
	  		  v->ThrottleCmd =  ((uint8_t)*(pDataIn + 3) > 100) ? 100 : (uint8_t)*(pDataIn + 3);
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
  InvCanTxCmd_t *p;
  uint8_t lIdx = 0;
  uint16_t u16Temp=0;
  int16_t i16Temp=0;


  for(lIdx=0; lIdx < 8; lIdx++)
  {
    *(pDataIn+lIdx) = 0;	//clear input buffer
  }

  p = (InvCanTxCmd_t*)pDataIn;

  switch (IdIn)
  {
    case CANTXID_INV_LOG_INFO_0:
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 0)
      {
        lStatus = ID_NO_MATCH;
        break;
      }

      v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] = 0;
      p->InvLogInfo0.Motor0Temp = (uint8_t)(v->Debugf[IDX_MOTOR0_TEMP]+40);
      p->InvLogInfo0.InvMos1Temp = (uint8_t)(v->Debugf[IDX_MOS1_TEMP]+40);
      p->InvLogInfo0.InvMos2Temp = (uint8_t)(v->Debugf[IDX_MOS2_TEMP]+40);
      p->InvLogInfo0.InvCapTemp = (uint8_t)(v->Debugf[IDX_CAP_TEMP]+40);
      p->InvLogInfo0.Motor1Temp = (uint8_t)(v->Debugf[IDX_MOTOR1_TEMP]+40);
      p->InvLogInfo0.Motor2Temp = (uint8_t)(v->Debugf[IDX_MOTOR2_TEMP]+40);
      p->InvLogInfo0.ThrottleRaw = (uint8_t)( 200.0f * v->Debugf[IDX_THROTTLE_RAW]);
      p->InvLogInfo0.ThrottleFinal = (uint8_t)( 200.0f * v->Debugf[IDX_THROTTLE_FINAL]);
      break;
    }
    case CANTXID_INV_LOG_INFO_1 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        u16Temp = (uint16_t)(v->Debugf[IDX_DC_VOLT] * 10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo1.DcVoltU16, (void*)&u16Temp, 2);

        i16Temp = (int16_t)v->Debugf[IDX_MOTOR_RPM];
        ByteOrderReverse((void*)&p->InvLogInfo1.MotorRpmI16, (void*)&i16Temp, 2);

//        p->InvLogInfo1.VehicleState = v->VehicleState Jeff
        p->InvLogInfo1.AlarmFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_CRI_ALARM_MASK)==0) ? 0 : 1;
        p->InvLogInfo1.LimpHomeFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_NON_CRI_ALARM_MASK)==0) ? 0 : 1;
        p->InvLogInfo1.WarnFlag = ((v->DebugU8[TX_INTERFACE_DBG_IDX_ERROR_FLAG]&CAN_TX_WARNING_MASK)==0) ? 0 : 1;
        p->InvLogInfo1.OutputMode = r->OutputModeCmd;
        p->InvLogInfo1.DeratingSrc = v->DeratingSrc;
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }

      break;
    }
    case CANTXID_INV_LOG_INFO_2 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {

        i16Temp = (int16_t)(v->Debugf[IDX_ID_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo2.IdCmdI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_IQ_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo2.IqCmdI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_ID_FBK]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo2.IdFbkI16, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_IQ_FBK]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo2.IqFbkI16, (void*)&i16Temp, 2);

      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_3 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        i16Temp = (int16_t)(v->Debugf[IDX_AC_LIMIT_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo3.AcLimitCmd, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_AC_LIMIT_TQ]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo3.AcLimitTq, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_DC_LIMIT_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo3.DcLimitCmd, (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_DC_LIMIT_TQ]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo3.DcLimitTq, (void*)&i16Temp, 2);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }

      break;
    }
    case CANTXID_INV_LOG_INFO_4 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        i16Temp = (int16_t)(v->Debugf[IDX_VD_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo4.VdCmdI16 , (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_VQ_CMD]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo4.VqCmdI16 , (void*)&i16Temp, 2);

        i16Temp = (int16_t)(v->Debugf[IDX_PERFROMANCE_TQ]*10.0f);
        ByteOrderReverse((void*)&p->InvLogInfo4.PerformanceTqI16 , (void*)&i16Temp, 2);

        p->InvLogInfo4.AccPedal1Volt = (uint8_t)( 50.0f * v->Debugf[IDX_ACC_PEDAL1_VOLT]);
        p->InvLogInfo4.EA5V = (uint8_t)( 40.0f * v->Debugf[IDX_EA5V]);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_5 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        memcpy(p->InvLogInfo5.AlarmCode, v->DebugError, 8);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_6 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
        /*todo: assign true value for signals*/
        i16Temp = (int16_t)(v->Debugf[IDX_AVERAGE_AC_POWER]);
        ByteOrderReverse((void*)&p->InvLogInfo6.AvgPwr , (void*)&i16Temp, 2);
        /*todo: assign true value for signals*/
        i16Temp = (int16_t)(v->Debugf[IDX_INSTANT_AC_POWER]);
        ByteOrderReverse((void*)&p->InvLogInfo6.InstPwr , (void*)&i16Temp, 2);
        /*todo: assign true value for signals*/
        u16Temp = (uint16_t)(v->Debugf[IDX_REMAIN_TIME]);
        ByteOrderReverse((void*)&p->InvLogInfo6.TimeRemain , (void*)&u16Temp, 2);

        p->InvLogInfo6.KillSwitchDI = (uint8_t)HAL_GPIO_ReadPin(Kill_Switch_DI_GPIO_Port, Kill_Switch_DI_Pin);
        p->InvLogInfo6.BoostDI = (uint8_t)HAL_GPIO_ReadPin(Boost_DI_GPIO_Port, Boost_DI_Pin);
        p->InvLogInfo6.ReverseDI = (uint8_t)HAL_GPIO_ReadPin(Reverse_DI_GPIO_Port, Reverse_DI_Pin);
        p->InvLogInfo6.BrakeDI = (uint8_t)HAL_GPIO_ReadPin(Brake_DI_GPIO_Port, Brake_DI_Pin);
        p->InvLogInfo6.RearLedFaultDI = (uint8_t)HAL_GPIO_ReadPin(Rear_Fault_DI_GPIO_Port, Rear_Fault_DI_Pin);
        p->InvLogInfo6.FrontLedFaultDI = (uint8_t)HAL_GPIO_ReadPin(Front_Fault_DI_GPIO_Port, Front_Fault_DI_Pin);
        p->InvLogInfo6.BufFbDI = (uint8_t)HAL_GPIO_ReadPin(BUF_FB_DI_GPIO_Port, BUF_FB_DI_Pin);
        p->InvLogInfo6.ISenUFaultDI = (uint8_t)HAL_GPIO_ReadPin(ISEN_UFault_DI_GPIO_Port, ISEN_UFault_DI_Pin);
        p->InvLogInfo6.ISenVFaultDI = (uint8_t)HAL_GPIO_ReadPin(ISEN_VFault_DI_GPIO_Port, ISEN_VFault_DI_Pin);
        p->InvLogInfo6.ISenWFaultDI = (uint8_t)HAL_GPIO_ReadPin(ISEN_WFault_DI_GPIO_Port, ISEN_WFault_DI_Pin);
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_7 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
//    	  p->InvLogInfo7.BatSoc =
    	  p->InvLogInfo7.BatMainSm = BatStation.MainSMGet();
    	  p->InvLogInfo7.BatPwrOffState = BatStation.PwrOffSMGet();
    	  p->InvLogInfo7.BatPwrOnState = BatStation.PwrOnSMGet();
    	  p->InvLogInfo7.E5V = (uint8_t)( 40.0f * v->Debugf[IDX_E5V] );
    	  p->InvLogInfo7.ES5V = (uint8_t)( 40.0f * v->Debugf[IDX_ES5V] );
    	  p->InvLogInfo7.HWID_H =  (uint8_t)( v->HWID[0] >> 4);
    	  p->InvLogInfo7.HWID_M = (uint8_t)((( v->HWID[0] & 0xF00 ) >> 4 ) | ( v->HWID[1] >> 8 ));
    	  p->InvLogInfo7.HWID_L = (uint8_t)( v->HWID[1] & 0xFF );
        v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] = 1;
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_8 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
          i16Temp = (int16_t)(v->Debugf[IDX_IU_FBK]*50.0f);
          ByteOrderReverse((void*)&p->InvLogInfo8.IuFbk, (void*)&i16Temp, 2);

          i16Temp = (int16_t)(v->Debugf[IDX_IV_FBK]*50.0f);
          ByteOrderReverse((void*)&p->InvLogInfo8.IvFbk, (void*)&i16Temp, 2);

          i16Temp = (int16_t)(v->Debugf[IDX_IW_FBK]*50.0f);
          ByteOrderReverse((void*)&p->InvLogInfo8.IwFbk, (void*)&i16Temp, 2);

          i16Temp = (int16_t)(v->Debugf[IDX_PREC]*10.0f);
          ByteOrderReverse((void*)&p->InvLogInfo8.PreC, (void*)&i16Temp, 2);
        v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_SAMPLE_FLAG] = 1;
      }
      else
      {
        lStatus = ID_NO_MATCH;
      }
      break;
    }
    case CANTXID_INV_LOG_INFO_9 :
    {
      if (v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] == 1)
      {
    	p->InvLogInfo9.AccPedal2Volt = (uint8_t)( 50.0f * v->Debugf[IDX_ACC_PEDAL2_VOLT] );
    	p->InvLogInfo9.S13V8 = (uint8_t)( 10.0f * v->Debugf[IDX_S13V8] );
#if MEASURE_CPU_LOAD
    	p->InvLogInfo9.Max10kHzLoopLoad = (uint8_t)( 2.0f * Max_ADC_Inj_Load_pct );
    	p->InvLogInfo9.AveCurrentLoopLoad = (uint8_t)( 2.0f * Ave_CurrentLoop_Load_pct );
    	p->InvLogInfo9.MaxPLCLoopLoad = (uint8_t)( 2.0f * Max_PLCLoop_Load_pct );
    	p->InvLogInfo9.AvePLCLoopLoad = (uint8_t)( 2.0f * Ave_PLCLoop_Load_pct );
    	p->InvLogInfo9.Max100HzLoopLoad = (uint8_t)( 2.0f * Max_100Hz_Load_pct );
#endif
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
