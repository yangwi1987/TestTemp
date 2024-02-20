/*
 * CANDrive.c
 *
 *  Created on: Jan 10, 2020
 *      Author: Will
 */

#include "CANDrive.h"



void CANDrive_IdConfig(FDCAN_HandleTypeDef *p, const CanIdConfig_t *v)
{
	uint8_t i=0;
//	uint8_t FilterIndexStart=0;
	FDCAN_FilterTypeDef  lTemp;
	const CanIdConfig_t *pTemp;
	StdCanIdFilter_t *Container;
	uint8_t STDFilterIdxStart;	/* max 28 standard filters can be used */
	uint8_t EXDFilterIdxStart;	/* max 08 extended filters can be used */


	/* Get registered Standard filter number */
	Container = (StdCanIdFilter_t*)p->msgRam.StandardFilterSA;

	for( STDFilterIdxStart = 0; STDFilterIdxStart < 28; STDFilterIdxStart++)
	{
		if( (Container+STDFilterIdxStart)->SFEC == 0 )
		{
			break;
		}
	}

	/* Get registered Extended filter number */
	Container = (StdCanIdFilter_t*)p->msgRam.ExtendedFilterSA;

	for( EXDFilterIdxStart = 0; EXDFilterIdxStart < 8; EXDFilterIdxStart++)
	{
		if( (Container+EXDFilterIdxStart)->SFEC == 0 )
		{
			break;
		}
	}

	/* load standard and extended ID filters accordingly*/
	pTemp = v;

	for(i = 0; i < CAN_ID_CONFIG_ARRAY_SIZE ; i++)
	{
		if(pTemp->Setup.Bits.ConfigUsage==CAN_ID_CONFIG_USED)
		{
			lTemp.FilterID1 = pTemp->Id1;
			lTemp.FilterID2 = pTemp->Id2;

			if(pTemp->Setup.Bits.IdType == CAN_ID_CONIFG_TYPE_STANDARD)
			{
				lTemp.IdType = FDCAN_STANDARD_ID;
				lTemp.FilterIndex = i+STDFilterIdxStart;
				assert_param(lTemp.FilterIndex < 28);
			}
			else
			{
				lTemp.IdType = FDCAN_EXTENDED_ID;
				lTemp.FilterIndex = i+EXDFilterIdxStart;
				assert_param(lTemp.FilterIndex < 8);

			}

			lTemp.FilterConfig = (uint32_t)((pTemp->Setup.Bits.RxFifo == 0)?FDCAN_FILTER_TO_RXFIFO0:FDCAN_FILTER_TO_RXFIFO1);
			lTemp.FilterType = (uint32_t)(pTemp->Setup.Bits.FilterType);
			HAL_FDCAN_ConfigFilter(p,&lTemp);
		}
		else
		{
			// no id configuration is required , do nothing
		}
		pTemp += 1;
	}
}

void CANDrive_ModuleConfig( FDCAN_HandleTypeDef *p,const  CanModuleConfig_t *v )
{
	HAL_FDCAN_ConfigInterruptLines( p, v->ItSelect, v->ItLineNumber );
	HAL_FDCAN_ConfigRxFifoOverwrite( p, v->RxFifoSrc, FDCAN_RX_FIFO_OVERWRITE );
	HAL_FDCAN_ConfigGlobalFilter( p, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE );
	HAL_FDCAN_ActivateNotification( p, v->ItActivateItem, BUFFER_IDX_RESERVED );
}

void CANDrive_LoadParam(CanDriveSetup_t *v, const  CanModuleConfig_t *m, const CanIdConfig_t *p )
{
	v->pIdConfigTable = p;
	v->pModuleConfigData = m;
}

void CANDrive_Init(FDCAN_HandleTypeDef *p,  CanDriveSetup_t *v )
{
	v->ModuleConfig(p,v->pModuleConfigData);
	v->IdCondfig(p,v->pIdConfigTable);
}


void ByteSwap( uint32_t *Data, uint8_t Size)
{
	uint8_t lIdx=0;
	uint8_t* pData;
	uint32_t x=0;
	x=*Data;
	uint8_t *px=(uint8_t*)&x;
	pData = (uint8_t*)Data;
	for(lIdx=0;lIdx<Size;lIdx++)
	{
		*(pData+Size-lIdx-1)= *(px+lIdx);
	}
}
