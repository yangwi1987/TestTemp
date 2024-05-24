/*
 * Protocol.c
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */
#if E10
#include "Protocol.h"
#include "ICANInterface.h"
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
{0};



uint8_t CAN_RxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANRxInterface *v, STRUCT_CANTxInterface *t )
{

}

uint8_t CAN_TxDataTranslate( uint32_t IdIn, uint8_t *pDataIn, STRUCT_CANTxInterface *v, STRUCT_CANRxInterface *r )
{
  uint8_t	lStatus = ID_MATCH;
  InvCanTxCmd_t *p;
  uint8_t lIdx = 0;

  for(lIdx=0; lIdx < 8; lIdx++)
  {
    *(pDataIn+lIdx) = 0;	//clear input buffer
  }

  p = (InvCanTxCmd_t*)pDataIn;

  switch (IdIn)
  {
    case CANTXID_INV_LOG_INFO_0:
    {
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
