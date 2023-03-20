/*
 * UdsServiceCtrl.h
 *
 *  Created on: 2020年5月28日
 *      Author: Will.Yang.CYY
 */

#ifndef INC_UDSSERVICECTRL_H_
#define INC_UDSSERVICECTRL_H_

#include "UdsNetworkLayer.h"
#include "AxisFactory.h"
#include  "UdsSecurityAccess.h"

/*
 * Constant define  ===========================================================================================================================================================================================
 */

#define CAN_UDS_DATA_BUFFER_SIZE	600
#define POSITIVE_RESPONSE_OFFSET	0x40

#define DATA_UPDATE_INTERVAL_MS	100

/*
 * Enum define ===========================================================================================================================================================================================
 */


typedef enum
{
	SID_DIAGNOSTIC_SESSION_CONTROL 	= 0x10,
	SID_ECUReset					= 0x11,
	SID_READ_DATA_BY_ID 			= 0x22,
	SID_SECURITY_ACCESS				= 0x27,
	SID_COMMUNICATION_CTRL			= 0x28,
	SID_REQUEST_DOWNLOAD 			= 0x34,
	SID_TRANSFER_DATA 				= 0x36,
	SID_TEST_PRESENT				= 0x3E,
	SID_REQUEST_TRANSFER_EXIT 		= 0x37,
	SID_RESPONSE_ON_EVENT			= 0x86,
	SID_WRITE_DATA_BY_ID_LSC 		= 0xBA,

}EnumUdsServiceIdentifier;

typedef enum
{
	NRC_serviceNotSupported = 0x11,
	NRC_SubFunctionNotSupported = 0x12,
	NRC_IncorrectMessageLengthOrInvalidFormat = 0x13,
	NRC_ConditionsNotCorrect = 0x22,
	NRC_RequestSequenceError = 0x24,
	NRC_RequestOutOfRange = 0x31,
	NRC_SecurityAccessDenied = 0x33,
	NRC_InvalidKey = 0x35,
	NRC_TransferDataSuspended = 0x71,
	NRC_generalProgrammingFailure = 0x72,
	NRC_WrongBlockSequenceCounter = 0x73,
	NRC_VoltageTooHigh = 0x92,
	NRC_VoltageTooLow = 0x93,
}EnumUdsNRC;

typedef enum
{
	TransferCtrlOperation_Idle,
	TransferCtrlOperation_DownloadReq,
	TransferCtrlOperation_UploadReq,
}EnumTrasnferCtrlOperation;

typedef enum
{
	TransferCtrlLoad_Idle,
	TransferCtrlLoad_Processing,
}EnumTrasnferCtrlLoadFlag;

typedef enum
{
	Authority_EndUser = 0,
	Authority_VehicleDealer = 1,
	Authority_VehicleMf = 2,
	Authority_LscMf = 5,
	Authority_LscFAE = 6,
	Authority_LscRd = 7,
}EnumAuthority;

typedef enum
{
	Session_0x00_IsoSAEReserved_00 = 0,
	Session_0x01_Default,
	Session_0x02_Programming,
	Session_0x03_ExtendedDiagnostic,
	Session_0x04_SafetySystemDiagnostic,
	Session_0x05_IsoSAEReserved,						// 0x05 ~ 0x3F
	Session_0x40_VehicleManufacturerSpecific = 0x40,	// 0x40 ~ 0x5F
	Session_0x60_SystemSupplierSpecific = 0x60,			// 0x60 ~ 0x7E
	Session_0x7F_IsoSAEReserved = 0x7F,
}EnumSession;

typedef enum
{
	stopResponseOnEvent=0,
	onDTCStatusChange,
	onTimerInterrupt,
	onChangeOfDataIdentifier,
	reportActivatedEvents,
	startResponseOnEvent,
	clearResponseOnEvent,
	onComparisonOfValues,
}EventType_e;

typedef enum
{
	enableRxAndTx = 0x00,
	enableRxAndDisableTx = 0x01,
	disableRxAndEnableTx = 0x02,
	disableRxAndTx = 0x03
}CommunicationCtrlSubfunc_e;

typedef enum
{
	CommunicationDisable,
	CommunicationEnable
}CommunicationCtrlState_e;

typedef enum
{
	CommunicationRspOn = 0,
	CommunicationRspOff,
}CommunicationRspState_e;

typedef enum
{
	PARAM_READ = 0,
	PARAM_WRITE = 1,
}UDSAccessParamAction_e;

/*
 * Type define ===========================================================================================================================================================================================
 */
typedef	union
{
	struct
	{
		uint16_t	ParamID :12;
		uint16_t	TargetID :4; // 0x1~2: AxisID, 0xF: Flash
	}Bits;
	uint16_t All;
}Union_LscParamID;

typedef	union
{
	struct
	{
		uint8_t	FuncID :7;
		uint8_t	RspState :1;
	}Bits;
	uint8_t All;
}Union_CommunicationCtrlParamSunFunc;

typedef struct
{
	union
	{
		uint8_t All;
		struct {
			uint8_t EncryptCode:4;
			uint8_t CompressCode:4;
		};
	}FormatCode;
	union
	{
		uint8_t All;
		struct {
			uint8_t AddressSize:4;
			uint8_t LengthSize:4;
		};
	}SizeCode;
}DataTransferCtrlInform_t;

typedef void (*funcTypeDataTransferCtrl_init)( void*,  uint16_t);

typedef void (*funcTypeUdsServiceCtrl_Init) ( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_ServiceHandler)( void*, void* );
typedef void (*funcTypeUdsServiceCtrl_DoPLC)(void*);
typedef void (*funcTypeUdsServiceCtrl_NegativeRspReq)( void*, uint8_t, uint8_t );


typedef void (*funcTypeUdsServiceCtrl_ReadDataByID)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_WriteDataByIDLsc)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_SessionControl)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_SecurityAccess) ( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_ECUReset)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_RequestDownload)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_TransferData)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_RequestTransferExit)( void*, void*, void* );
typedef void (*funcTypeUdsServiceCtrl_SendDataPeriodically)( void*, void*, uint32_t );

typedef struct
{
	//	uint8_t *pBuffer;
	uint8_t Sid;
	uint16_t BuffSize;
	uint8_t *pStartMemAddr;
	uint8_t Operation;
	uint32_t LengthTotal;
	uint32_t LengthHandled;
	uint16_t MaxNumberOfBlockLength;
	uint8_t BlockSequenceCounter;
	uint8_t LoadStatus;
	DataTransferCtrlInform_t Inform;
	funcTypeDataTransferCtrl_init init;
	void (*WriteToFlash)(uint8_t *pSrc,uint8_t *pDest,uint16_t Length);
	void (*ReadFromFlash)(uint8_t *pSrc,uint8_t *pDest,uint16_t Length);
	uint16_t (*ChecksumCalculate)(uint8_t *pStart,uint32_t Length);		    //return checksum result
	uint8_t (*TargetAddressAvailableCheck)(uint8_t *pStart , uint32_t Length);				//return 1 if target address is Available
}DataTransferCtrl_t;

void TransferCtrl_Init(DataTransferCtrl_t *v, uint16_t BufferSize);

#define DataTransferCtrl_Default \
{\
	0,0,0,0,0,0,0,0,0,	\
	{{0},{0}},											\
	(funcTypeDataTransferCtrl_init)TransferCtrl_Init,	\
	0,	\
	0,	\
	0,	\
	0,	\
}\


typedef struct
{
	uint8_t		EnableFlag;
	uint8_t		StartFlag;
	uint16_t	IntervalInMs;
	uint32_t	ParamId;
}PeriodicUpdate_t;
#define PERIODIC_UPDATE_DEFAULT \
{\
	0, 0, DATA_UPDATE_INTERVAL_MS, 880\
}\

typedef struct
{
	NetworkCtrl_t		NetWork;
	DataTransferCtrl_t	TransferCtrl;
	PeriodicUpdate_t	PeriodUpdateCtrl;
	UdsSecurityAccessCtrl_t *pSecurityCtrl;
	ParamMgr_t 			*pParamMgr;
	uint8_t			EnableAutoSend;
	funcTypeUdsServiceCtrl_Init				Init;
	funcTypeUdsServiceCtrl_ServiceHandler	ServiceHandler;
	funcTypeUdsServiceCtrl_NegativeRspReq   NegativeRspReq;
	funcTypeUdsServiceCtrl_DoPLC  			DoPlcLoop;
	funcTypeUdsServiceCtrl_SendDataPeriodically SendDataPeriodically;
	int32_t (*AccessParam)(uint16_t, uint16_t, int32_t*, uint16_t, uint8_t*);
}NetWorkService_t;

#define NetWorkService_t_Default \
{ \
	NetworkCtrl_t_default,		\
	DataTransferCtrl_Default,	\
	PERIODIC_UPDATE_DEFAULT,	\
	0,							\
	0,							\
	0,							\
	(funcTypeUdsServiceCtrl_Init)UdsServiceCtrl_Init,\
	(funcTypeUdsServiceCtrl_ServiceHandler)UdsServiceCtrl_ServiceHandler,\
	(funcTypeUdsServiceCtrl_NegativeRspReq)UdsServiceCtrl_NegativeRspReq,\
	(funcTypeUdsServiceCtrl_DoPLC)UdsServiceCtrl_DoPLC,					\
	(funcTypeUdsServiceCtrl_SendDataPeriodically)UdsServiceCtrl_SendDataPeriodically,\
	0,\
} \



/*
 * function define ======================================================================================================================================================================================
 */
void UdsServiceCtrl_DoPLC (NetWorkService_t *v);
void UdsServiceCtrl_Init ( NetWorkService_t *v, FDCAN_HandleTypeDef *p, UdsSecurityAccessCtrl_t *u );
void UdsServiceCtrl_ServiceHandler( NetWorkService_t *p ,NetworkCtrl_t *v  );
void UdsServiceCtrl_NegativeRspReq( LinkLayerCtrlUnit_t *pTx, uint8_t Sid, uint8_t Nrc );

void UdsServiceCtrl_ReadDataByID( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
void UdsServiceCtrl_WriteDataByIDLsc( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
void UdsServiceCtrl_SessionControl( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
void UdsServiceCtrl_SecurityAccess( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
void UdsServiceCtrl_ECUReset(NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx);
void UdsServiceCtrl_CommunicationCtrl(LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ExtranetCANStation_t *pOp);
void UdsServiceCtrl_TesterPresent(LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx, ExtranetCANStation_t *pOp);
void UdsServiceCtrl_RequestDownload( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx );
void UdsServiceCtrl_TransferData( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx );
void UdsServiceCtrl_RequestTransferExit( NetWorkService_t *p, LinkLayerCtrlUnit_t *pRx, LinkLayerCtrlUnit_t *pTx );
void UdsServiceCtrl_SendDataPeriodically( NetWorkService_t *p, LinkLayerCtrlUnit_t *pTx, uint32_t ParamID );

/*
 * Extern Variable define
 */
extern uint16_t BootAppTrig;



#endif /* INC_UDSSERVICECTRL_H_ */
