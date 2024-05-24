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
  1,
  10,
  {
	  CANTXID_INV_LOG_INFO_0
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

  v->DebugU8[TX_INTERFACE_DBG_IDX_LOG_ENABLE_FLAG] = 1;
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
      p->InvLogInfo0.IU_ADC = (uint16_t)(v->Debugf[IDX_IU_FBK]);
      p->InvLogInfo0.IV_ADC = (uint16_t)(v->Debugf[IDX_IV_FBK]);
      p->InvLogInfo0.IW_ADC = (uint16_t)(v->Debugf[IDX_IW_FBK]);
      p->InvLogInfo0.VDC_ADC = (uint16_t)(v->Debugf[IDX_PREC]);
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
