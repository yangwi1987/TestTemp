/*
 * ExtranetCANStation.h
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_EXTRANETCANSTATION_H_
#define INC_EXTRANETCANSTATION_H_

#include "ICANInterface.h"
#include "CANQueue.h"
#include "Protocol.h"
#include "CANDrive.h"
#include "stm32g4xx_hal.h"

#define DISABLE_RST_TIMEOUT		4999

typedef void (*functypeExtranetCANStation_Init)( void *, const void *, void *);
typedef uint8_t (*functypeExtranetCANStation_ReadRxFIFO)( void *, uint32_t );
typedef uint8_t (*functypeExtranetCANStation_RxHandlePacket)( void * );
typedef uint8_t (*functypeExtranetCANStation_WriteTxFIFO)( void * );
typedef uint8_t (*functypeExtranetCANStation_TxHandlePacket)( void *, uint32_t );
typedef void (*functypeExtranetCANStation_DoPLCLoop)( void * );
typedef void (*functypeExtranetCANStation_DisableStateReset)( void *);

typedef struct {
	uint32_t Enable;
	uint32_t				ForceDisable;
	uint32_t 				PlcCnt1;
	uint16_t				DisableResetCnt;
	uint16_t				KeepDisableFlg;
	FDCAN_HandleTypeDef 	*phfdcan;
	FDCAN_TxHeaderTypeDef 	TxHead;
	STRUCT_CAN_QUEUE 		TxQ;
	STRUCT_CAN_QUEUE 		RxQ;
	STRUCT_CAN_DATA 		NowCANDataTx;
	STRUCT_CAN_DATA 		NowCANDataRx;
	const CANProtocol				*pProtocol;			//load pointer in drive_ini
	const CanIdConfig_t			*pIdConfigTable;	//load pointer in drive_ini
	const CanModuleConfig_t		*pModuleConfig;		//load pointer in drive_ini
	STRUCT_CANRxInterface	RxInfo;
	STRUCT_CANTxInterface	TxInfo;
	CanDriveSetup_t			DriveSetup;
	functypeExtranetCANStation_Init				 Init;
	functypeExtranetCANStation_ReadRxFIFO		 ReadRxFIFO;
	functypeExtranetCANStation_RxHandlePacket 	 RxHandlePacket;
	functypeExtranetCANStation_WriteTxFIFO		 WriteTxFIFO;
	functypeExtranetCANStation_TxHandlePacket	 TxHandlePacket;
	functypeExtranetCANStation_DoPLCLoop		 DoPlcLoop;
	functypeExtranetCANStation_DisableStateReset DisableRst;
} ExtranetCANStation_t;

void ExtranetCANStation_Init( ExtranetCANStation_t *v, const CANProtocol *p, FDCAN_HandleTypeDef *u);
uint8_t ExtranetCANStation_ReadRxFIFO( ExtranetCANStation_t *v, uint32_t lFifoN );
uint8_t ExtranetCANStation_RxHandlePacket( ExtranetCANStation_t *v );
uint8_t ExtranetCANStation_WriteTxFIFO( ExtranetCANStation_t *v );
uint8_t ExtranetCANStation_TxHandlePacket( ExtranetCANStation_t *v, uint32_t lDIn );
void ExtranetCANStation_DoPLCLoop( ExtranetCANStation_t *v );
void ExtranetCANStation_DisableStateReset( ExtranetCANStation_t *v );

#define CAN_TX_HEADER_DEFAULT { \
	0,\
	FDCAN_STANDARD_ID,\
	FDCAN_DATA_FRAME,\
	FDCAN_DLC_BYTES_8,\
	FDCAN_ESI_PASSIVE,\
	FDCAN_BRS_OFF,\
	FDCAN_CLASSIC_CAN,\
	FDCAN_NO_TX_EVENTS,\
	0\
}



#define EXTRANET_CAN_STATION_DEFAULT { \
		0,												\
		0,												\
		0,												\
		0,												\
		0,												\
		NULL,											\
		CAN_TX_HEADER_DEFAULT,							\
		QUEUE_DEFAULT,									\
		QUEUE_DEFAULT,									\
		CAN_DATA_DEFAULT,								\
		CAN_DATA_DEFAULT,								\
		0,												\
		0,												\
		0,												\
		CANRXINFO_DEFAULT,								\
		CANTXINFO_DEFAULT,								\
		CAN_DRIVE_SETUP_DEFAULT,						\
		(functypeExtranetCANStation_Init)ExtranetCANStation_Init,							\
		(functypeExtranetCANStation_ReadRxFIFO)ExtranetCANStation_ReadRxFIFO,				\
		(functypeExtranetCANStation_RxHandlePacket)ExtranetCANStation_RxHandlePacket,		\
		(functypeExtranetCANStation_WriteTxFIFO)ExtranetCANStation_WriteTxFIFO,				\
		(functypeExtranetCANStation_TxHandlePacket)ExtranetCANStation_TxHandlePacket,		\
		(functypeExtranetCANStation_DoPLCLoop)ExtranetCANStation_DoPLCLoop,					\
		(functypeExtranetCANStation_DisableStateReset)ExtranetCANStation_DisableStateReset	\
}


extern const CanModuleConfig_t CANModuleConfigExtra ;
#endif /* INC_EXTRANETCANSTATION_H_ */
