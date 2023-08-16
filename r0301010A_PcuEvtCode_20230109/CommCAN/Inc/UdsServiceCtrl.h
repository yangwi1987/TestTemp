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
#include "UdsDID.h"
#include "DiagnosticTroubleCode.h"
/*
 * Constant define  ===========================================================================================================================================================================================
 */

#define CAN_UDS_DATA_BUFFER_SIZE	600
#define POSITIVE_RESPONSE_OFFSET	0x40

#define DATA_UPDATE_INTERVAL_MS	100
#define TRUE 1
#define FALSE 0
#define NONE 0

#define FALSE_ACCESS_EXCEED_LIMIT_DELAY_TIME_MS 10 * 60 * 1000 // 10 minutes
#define P2_SERVER_MAX              150       // Resolition: 1ms
#define P2_STAR_SERVER_MAX         505       // Resolition: 10ms
#define P2_STAR_SERVER_MAX_MS P2_STAR_SERVER_MAX * 10

/*
 * Enum define for BRP UDS ===========================================================================================================================================================================================
 */
typedef enum
{                                //
	SID_0x10_DSC_with_SF 	 = 0x10,     //  DiagnosticSessionControl (0x10) service   w/SF
	SID_0x11_ER_with_SF		 = 0x11,     //  ECUReset (0x11) service                   w/SF
	SID_0x27_SA_with_SF		 = 0x27,     //  SecurityAccess (0x27) service             w/SF
	SID_0x3E_TP_with_SF		 = 0x3E,     //  TesterPresent (0x3E) service              w/SF

	SID_0x22_RDBI_without_SF = 0x22,     //  ReadDataByIdentifier (0x22) service
	SID_0x2E_WDBI_without_SF = 0x2E,     //  WriteDataByIdentifier (0x2E) service

	SID_0x14_CDTCI_without_SF= 0x14,     //  ClearDiagnosticInformation (0x14) Service
	SID_0x19_RDTCI_with_SF   = 0x19,     //  ReadDTCInformation (0x19) Service         w/SF

	SID_0x2F_IOCBI_without_SF= 0x2F,     //  InputOutputControlByIdentifier (0x2F) service
	SID_0x31_RC_with_SF      = 0x31,     //  RoutineControl (0x31) service

	SID_0x34_RD_without_SF 	 = 0x34,     //  RequestDownload (0x34) service
	SID_0x36_TD_without_SF 	 = 0x36,     //  TransferData (0x36) service
	SID_0x37_RTE_without_SF  = 0x37,     //  RequestTransferExit (0x37) service


}EnumUdsServiceBRPIdentifier;

typedef enum
{
	NRC_0x00_PR      = 0x00,    //  Positive Response (0x00)
	NRC_0x10_GR      = 0x10,    //  General Reject (0x10) NRC
	NRC_0x11_SNS 	 = 0x11,    //  Service Not Supported (0x11) NRC
	NRC_0x12_SFNS    = 0x12,    //  Sub Function Not Supported - Invalid Format (0x12) NRC
	NRC_0x13_IMLOIF  = 0x13,    //  Incorrect Message Length Or Invalid Format (0x13) NRC
	NRC_0x14_RTL 	 = 0x14,    //  Response Too Long (0x14) NRC
	NRC_0x21_BRR 	 = 0x21,    //  Busy - Repeat Request (0x21) NRC
	NRC_0x22_CNC 	 = 0x22,    //  Conditions Not Correct (0x22) NRC
	NRC_0x24_RSE 	 = 0x24,    //  Request Sequence Error (0x24) NRC
	NRC_0x25_NRFSC 	 = 0x25,    //  No Response From Subnet Component (0x25) NRC
	NRC_0x26_FPEORA  = 0x26,    //  Failure Prevents Execution Of Requested Action (0x26) NRC
	NRC_0x31_ROOR    = 0x31,    //  Request Out Of Range (0x31) NRC
	NRC_0x33_SAD 	 = 0x33,    //  Security Access Denied - Security Access Requested (0x33) NRC
	NRC_0x35_IK 	 = 0x35,    //  Invalid Key (0x35) NRC
	NRC_0x36_ENOA 	 = 0x36,    //  Exceed Number Of Attempts (0x36) NRC
	NRC_0x37_RTDNE   = 0x37,    //  Required Time Delay Not Expired (0x37) NRC
	NRC_0x70_UDNA 	 = 0x70,    //  Upload Download Not Accepted (0x70) NRC
	NRC_0x71_TDS 	 = 0x71,    //  Transfer Data Suspended (0x71) NRC
	NRC_0x72_GPF     = 0x72,    //  General Programming Failure (0x72) NRC
	NRC_0x73_WBSC    = 0x73,    //  Wrong Block Sequence Counter (0x73) NRC
	NRC_0x78_RCRRP   = 0x78,    //  Request Correctly Received-Respnse Pending (0x78) NRC
	NRC_0x7E_SFNSIAS = 0x7E,    //  Sub-Function Not Supported In Active Session (0x7E) NRC
	NRC_0x7F_SNSIAS  = 0x7F,    //  Service Not Supported In Active Session (0x7F) NRC
	NRC_0x81_RPMTH   = 0x81,    //  RPM Too High (0x81) NRC
	NRC_0x82_RPMTL   = 0x82,    //  RPM Too Low (0x82) NRC
	NRC_0x83_EIR     = 0x83,    //  Engine Is Running (0x83) NRC
	NRC_0x84_EINR    = 0x84,    //  Engine Is Not Running (0x84) NRC
	NRC_0x85_ERTTL   = 0x85,    //  Engine Run Time Too Low (0x85) NRC
	NRC_0x86_TEMPTH  = 0x86,    //  Temperature Too High (0x86) NRC
}EnumUdsBRPNRC;

typedef enum
{
	Session_0x01_DS   = 0x01,
	Session_0x02_PRGS = 0x02,
	Session_0x03_EXTDS= 0x03,
}EnumDiagnosticSessionBRP;

typedef enum
{
	Soft_Reset = 0x03,
}EnumECUResetBRP;

typedef enum
{
	reportDTCByStatusMask = 0x02,
	reportDTCStoredDataByRecordNumber = 0x05
}EnumCDTCIreportTypeBRP;

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
	NRC_exceededNumberOfAttempts = 0x36,
	NRC_requiredTimeDelayNotExpired = 0x37,
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
	Security_Level_0,
	Security_Level_1,
	Security_Level_2,
	Security_Level_3,
	Security_Level_4,
}EnumSecurityLevel;

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
 * Type define for BRP UDS ===========================================================================================================================================================================================
 */

typedef	union
{
	struct
	{
		uint8_t	SunFunctionType :7;
		uint8_t	SuppressPosRspMsgIndicationBit :1;
	}Bits;
	uint8_t All;
}Union_UdsDataParameter;

typedef void (*funcTypeUdsServiceCtrlBRP_ServiceHandler_Functional)( void*, void* , void* );
typedef void (*funcTypeUdsServiceCtrlBRP_ServiceHandler_Physical)( void*, void* , void* );

typedef struct
{
	float Temperature_Max;
	float Temperature_Min;
}Temperature_Max_Min_t;

typedef struct
{
	uint8_t		test;
	int16_t     ServoOnOffState;
	uint8_t     SuppressPosRspMsgIndicationBit;
	uint8_t     BRPSessionCNTEnable;
	uint32_t    BRPSessionCNT;
	uint8_t     BRPECUSoftResetEnable;
	uint32_t    FalseAccessExceedLimitCNT;
	Temperature_Max_Min_t ESC_Mosfets_Center_Temp_Rec;
	Temperature_Max_Min_t ESC_Mosfets_Side_Temp_Rec;
	Temperature_Max_Min_t ESC_Capacitor_Temp_Rec;
	Temperature_Max_Min_t Motor_Temp_Rec;
	float                 Res_Max_Rec;
	EnumUdsBRPNRC           Response_Code;
	EnumDiagnosticSessionBRP DiagnosticSession;
	EnumUdsBRPNRC (*RDBI_Function) (UdsDIDParameter_e , LinkLayerCtrlUnit_t *, LinkLayerCtrlUnit_t *);
	EnumUdsBRPNRC (*WDBI_Function) (uint16_t Idx, LinkLayerCtrlUnit_t *, LinkLayerCtrlUnit_t *);
	DTCStation_t *pDTCStation;
	UdsSecurityAccessCtrl_t *pSecurityCtrl;
	funcTypeUdsServiceCtrlBRP_ServiceHandler_Functional ServiceHandler_Functional;
	funcTypeUdsServiceCtrlBRP_ServiceHandler_Physical   ServiceHandler_Physical;
}ServiceCtrlBRP_t;

#define ServiceCtrlBRP_Default \
{ \
	0,                     /**/\
	0,                     /*ServoOnOffState*/\
	FALSE,                 /*SuppressPosRspMsgIndicationBit*/\
	0,                     /*BRPSessionCNTEnable*/\
	0,                     /*BRPSessionCNT*/\
	0,                     /*BRPECUSoftResetEnable*/\
	0,                     /*FalseAccessExceedLimitCNT*/\
	{ -999,999 },               /*ESC_Mosfets_Center_Temp_Rec*/\
    { -999,999 },               /*ESC_Mosfets_Side_Temp_Rec; */\
    { -999,999 },               /*ESC_Capacitor_Temp_Rec;    */\
    { -999,999 },               /*Motor_Temp_Rec;            */\
	0,                     /*Res_Max_Rec;               */\
	NRC_0x00_PR,           /*Response_Code*/\
	Session_0x01_DS,      /*DiagnosticSession*/\
	0,                     /*RDBI_Function*/\
	0,                     /*WDBI_Function*/\
	0,                     /*pDTCStation*/\
	0,                     /*pSecurityCtrl*/\
	(funcTypeUdsServiceCtrlBRP_ServiceHandler_Functional)UdsServiceCtrlBRP_ServiceHandler_Functional,\
	(funcTypeUdsServiceCtrlBRP_ServiceHandler_Physical)UdsServiceCtrlBRP_ServiceHandler_Physical,\
} \
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
	ServiceCtrlBRP_t    ServiceCtrlBRP;
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
	ServiceCtrlBRP_Default,     \
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
 * Function define for BRP UDS ===========================================================================================================================================================================================
 */
void UdsServiceCtrlBRP_ServiceHandler_Functional( NetWorkService_t *p ,NetworkCtrl_t *v,  ServiceCtrlBRP_t *m );
void UdsServiceCtrlBRP_ServiceHandler_Physical( NetWorkService_t *p ,NetworkCtrl_t *v,   ServiceCtrlBRP_t *m );

/*
 * Extern Variable define
 */
extern uint16_t BootAppTrig;



#endif /* INC_UDSSERVICECTRL_H_ */
