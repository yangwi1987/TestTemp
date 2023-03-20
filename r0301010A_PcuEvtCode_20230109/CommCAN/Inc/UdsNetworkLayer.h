/*
 * UdsNetworkLayer.h
 *
 *  Created on: 2020年6月3日
 *      Author: Will.Yang.CYY
 */

#ifndef INC_UDSNETWORKLAYER_H_
#define INC_UDSNETWORKLAYER_H_

#include "stdint.h"
#include "stm32g4xx_hal.h"
#include "CANDrive.h"
#include "ParamMgr.h"
//Will @ 20200603



/*
 * Constants define ===========================================================================================================================================================================================
 */
#define MAX_BUFFER_SIZE 			256
#define PDU_PCI_SHIFT 				4
#define PDU_DLC 					0x0F
#define BS							10
#define ST_MIN_MS					1
#define FLOW_CTRL_STM_MIN_LIMIT 	100
#define FLOW_CTRL_BS_LIMIT 			100
#define TX_REQUEST_CLEAR 			0
#define TX_REQUEST_SET 				1

#define UDS_RX_ID_LSC_START			0x300
#define UDS_RX_ID_LSC_END			0x307
#define UDS_RX_ID_LSC_MASK			0x300
#define UDS_TX_ID_LSC				0x308

#define UDS_RX_ID_CUS_START			0x7D1
#define UDS_RX_ID_CUS_END			0x7D1
#define UDS_RX_ID_CUS_MASK 			0x7D0
#define UDS_TX_ID_CUS				0x7D8
#define UDS_RX_ID_CUS_BOOT_START 	0x7DF
#define UDS_RX_ID_CUS_BOOT_END		0x7DF
/*
 * Enum define ===========================================================================================================================================================================================
 */

typedef enum
{
	Diagnostic,
	RemoteDiagnostic,
}EnumMType;


typedef enum
{
	Physical,
	Functional,
}EnumN_TAtype;

typedef enum
{
	STmin,
	BlockSize,
}EnumParameter;

//@EnumN_Result
typedef enum
{
	N_OK,
	N_TIMEOUT_A,
	N_TIMEOUT_Bs,
	N_TIMEOUT_Cr,
	N_WRONG_SN,
	N_INVALID_FS,	//FS = Flow status in PCI_FC
	N_UNEXP_PDU,
	N_WFT_OVERN,	// wait frame time
	N_BUFFER_OVFLW,
	N_ERROR,
	N_RX_ON,
	N_WRONG_PARAMETER,
	N_WRONG_VALUE,
}EnumN_Result;

typedef enum
{
	ChangeParam_N_OK,
	ChangeParam_RX_ON,
	ChangeParam_Wrong_param,
	ChangeParam_Wrong_Value,
}EnumChangeParamResult;

typedef enum
{
	Tx_Idle=0,
	Tx_Request = 0x01,
	Tx_Processing,
	Tx_Complete,
	Tx_Error,

	Rx_Idle = 0x80,
	Rx_Processing = 0x81,
	Rx_Complete,
	Rx_Reading,

	Rx_Error,
}EnumNLService;


typedef enum
{
	FlowStatus_ContinueToSend,
	FlowStatus_Wait,
	FlowStatus_Overflow,
}EnumFlowStatus;

typedef enum
{
	PCI_SF,	//Single Frame
	PCI_FF,		//First Frame
	PCI_CF,		//Consequence Frame
	PCI_FC,		//Flow Control
}EnumPCIType;

//@EnumFlowCtrl
typedef enum
{
	FlowCtrl_Idle 		= 0,
	FlowCtrl_WaitforFC 	= 1,
	FlowCtrl_FCReceived,
	FlowCtrl_RxTimeout,
}EnumFlowCtrl;

/*
 * type define ===========================================================================================================================================================================================
 */
typedef struct
{
	union
	{
		struct {
			uint8_t DLC :4;
			uint8_t PCIType :4;
		}Bits;
		uint8_t All;
	}PCI;
	uint8_t Datas[7];
}PduDefault_t;

typedef struct
{
	union
	{
		struct {
			uint8_t DLC :4;
			uint8_t PCIType :4;
		}Bits;
		uint8_t All;
	}PCI;
	uint8_t Datas[7];
}PduSF_t;

typedef struct
{
	union
	{
		struct {
			uint16_t DLC_H :4;
			uint16_t PCIType :4;
			uint16_t DLC_L :8;
		}Bits;
		uint16_t All;
	}PCI;
	uint8_t Datas[6];
}PduFF_t;

typedef struct
{
	union
	{
		struct {
			uint8_t SN :4;
			uint8_t PCIType :4;
		}Bits;
		uint8_t All;
	}PCI;
	uint8_t Datas[7];
}PduCF_t;

typedef struct
{
	union
	{
		struct {
			uint8_t FS :4;
			uint8_t PCIType :4;
		}Bits;
		uint8_t All;
	}PCI;
	uint8_t BlockSize;
	uint8_t STMin; // Sent Time minimum
	uint8_t Reserved[5];
}PduFC_t;

typedef union
{
	PduDefault_t Default;
	PduSF_t SF; // Single Frame
	PduFF_t FF; // First Frame
	PduCF_t CF; // Consequence Frame
	PduFC_t FC; // Flow Control
}PDU_u;


typedef void (*funcTypeNetWorkLayer_TxDataHandle)(void *);
typedef void (*funcTypeNetWorkLayer_RxDataHandle)(void *);
typedef void (*funcTypeNetworkLayer_FlowCtrlSendReq)( void *, uint8_t );
typedef void (*funcTypeNetWorkLayer_Init)(void* , void* );
typedef uint8_t (*funcTypeNetWorkLayer_ChangeParamConfirm)(void *);



typedef struct
{
	int16_t 	LengthTotal;	//total byte number to be received/send, max = 4095, the number is excluding PCI item.
	int16_t 	LengthHandled;  //received/ transmitted byte number
	uint8_t 	SnNow;			//Serial Number received/transmit now, range from 0x00~0x0F
	uint8_t 	SnBefore;		//Serial Number received/transmit before,range from 0x00~0x0F
	uint8_t 	Result;			//handling result ,refer @EnumN_Result
	uint8_t 	Status;
	uint8_t 	Data[MAX_BUFFER_SIZE];	// messages TX/RX buffer
	uint16_t 	DataSize;		//TX/RX buffer size
	uint16_t 	TimeoutCnt;		//flow control timeout counter
	uint8_t 	CFCnt;			//Consequence frame counter	,range from 0x00~0xFF
	uint8_t 	FlowCtrlFlag;	//flow control flag,refer to @EnumFlowCtrl
	uint32_t 	CanId;			//id handled
}LinkLayerCtrlUnit_t;


#define LinkLayerCtrlUnit_t_default \
{\
	0,0,0,0,0,0,\
	{	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	\
	},	\
	0,0,0,0,0\
}\

typedef struct
{
	FDCAN_HandleTypeDef *pCanHandle;
	FDCAN_TxHeaderTypeDef TxHeader;
	const CanModuleConfig_t *pModuleConfig;
	const CanIdConfig_t *pIdConfigTable;
	LinkLayerCtrlUnit_t Tx;
	LinkLayerCtrlUnit_t Rx;
	CanDriveSetup_t	DriveSetup;
	uint8_t STminCmd;	// Sent Time minimum Command
	uint8_t BsCmd;		// Block Size command
	uint8_t STminReq;	// Sent Time minimum Request
	uint8_t BsReq;		// Block Size Request
	uint8_t STCounter;	// Sent Time Counter
	funcTypeNetWorkLayer_Init Init;
	funcTypeNetWorkLayer_TxDataHandle TxDataHandle;
	funcTypeNetWorkLayer_RxDataHandle RxDataHandle;
	funcTypeNetworkLayer_FlowCtrlSendReq FlowCtrlSendReq ;
	funcTypeNetWorkLayer_ChangeParamConfirm ChangeParamRequest;
}NetworkCtrl_t;




/*
 * Function define ===========================================================================================================================================================================================
 */

void NetworkLayer_FlowCtrlSendReq( NetworkCtrl_t *v,uint8_t status );
void NetworkLayer_Init(NetworkCtrl_t *v , FDCAN_HandleTypeDef *p );
void NetworkLayer_TxDataHandle( NetworkCtrl_t *v);
void NetworkLayer_RxDataHandle( NetworkCtrl_t *v );
uint8_t NetworkLayer_ChangeParamRequest(NetworkCtrl_t *v ,uint8_t lStminCmd, uint8_t lBsCmd  );


#define NetworkCtrl_t_default 	\
{	\
	0,{0,0,0,0,0,0,0,0,0},0,0,			\
	LinkLayerCtrlUnit_t_default,	\
	LinkLayerCtrlUnit_t_default,	\
	CAN_DRIVE_SETUP_DEFAULT,		\
	0,0,0,0,0,						\
	(funcTypeNetWorkLayer_Init)NetworkLayer_Init,								\
	(funcTypeNetWorkLayer_TxDataHandle)NetworkLayer_TxDataHandle,				\
	(funcTypeNetWorkLayer_RxDataHandle)NetworkLayer_RxDataHandle,				\
	(funcTypeNetworkLayer_FlowCtrlSendReq)NetworkLayer_FlowCtrlSendReq,			\
	(funcTypeNetWorkLayer_ChangeParamConfirm)NetworkLayer_ChangeParamRequest,	\
}	\

extern const CanIdConfig_t LscCanIdTableIntra[];
extern const CanModuleConfig_t CANModuleConfigIntra;




//typedef struct
//{
//	uint8_t MType;
//	struct
//	{
//		uint8_t N_SA;		//Network Source Address 0~FF
//		uint8_t N_TA;		//Network Target Address 0~FF
//		uint8_t N_TAtype;	//Networt Target Address type  0 = physical , 1 = functional
//		uint8_t N_AE;		//Network Address Extension
//	}N_AI; 				//Address Information
//	uint16_t Length;	//0~4095
//	uint8_t	*pData;		//pointer of data received/to be send
//	uint8_t ParamType;	//0 = STmin,1=BS(Block size)
//	uint8_t ParamValue;	//0~FF param value for STmin or BS
//	uint8_t N_Result;	//
//	uint8_t Result_changeparam;
//	uint8_t NLService;
//}NetworkService_t;
//typedef uint8_t (*funcTypeNetWorkService_DataIndication)(void *);
//typedef uint8_t (*funcTypeNetWorkService_DataFFIndication)(void *);
//typedef uint8_t (*funcTypeNetWorkService_Request)(void *);
//typedef uint8_t (*funcTypeNetWorkService_Confirm)(void *);
//typedef uint8_t (*funcTypeNetWorkService_ChangeParamConfirm)(void *);




#endif /* INC_UDSNETWORKLAYER_H_ */
