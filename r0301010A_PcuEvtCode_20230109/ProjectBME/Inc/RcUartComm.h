/*
 * RcUartComm.h
 *
 *  Created on: 2022年7月21日
 *      Author: Will.Yang.CYY
 */

#ifndef INC_RCUARTCOMM_H_
#define INC_RCUARTCOMM_H_

#include "string.h"
#include "stm32g4xx_hal.h"
#include "ICANInterface.h"

typedef enum RcCommError_e
{
	RC_COMM_ERROR = -1,
	RC_COMM_NO_ERROR = 0,	
	RC_COMM_ERROR_UNSUPPORT_DATA_ID = 1,
}RcCommError_t;

typedef enum RcCommDataId_e
{
	RC_COMM_DATA_ID_RC_FW_VERSION 	= 0x0100,
	RC_COMM_DATA_ID_RC_SN 			= 0x0101,	
	RC_COMM_DATA_ID_RF_FW_VERSION 	= 0x0200,
	RC_COMM_DATA_ID_RF_SN 			= 0x0201,
	RC_COMM_DATA_ID_ESC_FW_VERSION = 0x0300,
	RC_COMM_DATA_ID_ESC_SN 		= 0x0301,
	RC_COMM_DATA_ID_BMS_FW_VERSION = 0x0400,
	RC_COMM_DATA_ID_BMS_SN 		= 0x0401,
}RcCommDataId_t;

#define RC_COMM_QUERY_RF_INFO_INTERVAL_MS	100
#define RC_COMM_QUERY_RF_INFO_INTERVAL_PRESCALER_CNT	1	/* 1 = RC_COMM_QUERY_RF_INFO_INTERVAL_MS / 100 */
#define RC_COMM_QUERY_RF_INFO_TIMEOUT_THRESHOLD		1500	/* 1500ms = 1.5 sec */
#define RC_COMM_QUERY_RF_INFO_TIMEOUT_CNT  15				/* 15 = RC_COMM_QUERY_RF_INFO_TIMEOUT_THRESHOLD / 100 */

#define RC_COMM_RC_FW_VER_SIZE	3	/* RC firmware version array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RC_SN_SIZE		32	/* RC serial number array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RF_FW_VER_SIZE	3	/* RF firmware version array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RF_SN_SIZE		32	/* RF serial number array size in byte. Todo: clarify the exact nuber with TBS */

#define RC_TEIMOUT_TEST 0
#define RC_COMM_DMA_USAGE 1
#define RC_COMM_BYPASS_CRC 0

#define RC_COMM_MSG_DLC_MAX  60
#define RC_COMM_RX_BUFF_SIZE 60
#define RC_COMM_TX_BUFF_SIZE 60
#define RC_COMM_HEAD_VALUE 0x03
#define RC_COMM_END_VALUE 0xFF
#define RC_COMM_TIMEOUT_THRESHOLD_100MS 11	// 11*100ms = 1100ms

typedef void (*functypeRcComm_Init)( void *,void *,void *,void*,void* );
typedef uint16_t (*functypeRcComm_CalCrc)( void *, uint8_t*, uint8_t);
typedef void (*functypeRcComm_StartScan)(void*);
typedef void (*functypeRcComm_GetUartMsgFromIsr)(void*);
typedef void (*functypeRcComm_MsgHandler)(void*,uint8_t*);
typedef void (*functypeRcComm_MsgDecoder)(void*);
typedef RcCommError_t (*functypeRcComm_QueryInfoFromRF)(void*, RcCommDataId_t);
typedef void (*functypeRcComm_10HzLoop)(void*);
typedef void (*functypeRcComm_Reset)(void*);

typedef struct{
	UART_HandleTypeDef *pTarget;
	CRC_HandleTypeDef *phcrc;
	STRUCT_CANTxInterface *pTxInterface;
	STRUCT_CANRxInterface *pRxInterface;
	uint8_t RxPutIdx;
	uint8_t RxReadIdx;
	uint8_t RxFlag;
	uint8_t TxFlag;
	uint8_t RcHaveConnectedFlag;
	uint8_t RxUnit[1];
	uint8_t RxBuff[RC_COMM_RX_BUFF_SIZE];
	uint8_t TxBuff[RC_COMM_TX_BUFF_SIZE];
	uint16_t RxDlc;
	uint16_t TxDlc;
	uint16_t TimeoutCnt;
	uint16_t VerConfig;
	uint8_t RCFwVer[RC_COMM_RC_FW_VER_SIZE];
	uint8_t RCSN[RC_COMM_RC_SN_SIZE];
	uint8_t RFFwVer[RC_COMM_RF_FW_VER_SIZE];
	uint8_t RFSN[RC_COMM_RF_SN_SIZE];
	functypeRcComm_Init Init;
	functypeRcComm_CalCrc CalCrc;
	functypeRcComm_StartScan StartScan;
	functypeRcComm_GetUartMsgFromIsr GetMsgFromIsr;
	functypeRcComm_MsgHandler MsgHandler;
	functypeRcComm_MsgDecoder MsgDecoder;
	functypeRcComm_QueryInfoFromRF QueryInfoFromRF;
	functypeRcComm_10HzLoop _10HzLoop;
	functypeRcComm_Reset	Reset;
}StructUartCtrl;



typedef enum{
	IDX_RC_COMM_HEAD = 0,
	IDX_RC_COMM_DLC_L,
	IDX_RC_COMM_DLC_H,
	IDX_RC_COMM_CMD_ID,
	IDX_RC_COMM_DATA_START,
}EnumRCCommIdx;

typedef enum{
	RC_COMM_RX_STATE_IDLE,
	RC_COMM_RX_STATE_SCAN_FOR_HEADER,
	RC_COMM_RX_STATE_PROCESSING,
	RC_COMM_RX_STATE_COMPLETE,
}EnumRCCommRxState;

typedef enum{
	RC_COMM_TX_STATE_IDLE,			/* UART Tx buffer is able to be accessed */
	RC_COMM_TX_STATE_FILLING,		/* UART Tx buffer is filling by other functions*/
	RC_COMM_TX_STATE_TXREQ,			/* UART TX Buffer is filled, waiting for transmitting */
	RC_COMM_TX_STATE_TRANSFERING,	/* UART TX Buffer is transmitting via hal functions*/
}EnumRCCommTxState;

#define RC_CMD_DATA_IDX_THROTTLE_CMD 	0 + IDX_RC_COMM_DATA_START
#define RC_CMD_DATA_IDX_TURN_OFF_REQ 	1 + IDX_RC_COMM_DATA_START
#define RC_CMD_DATA_IDX_RC_CONN_STATUS 	2 + IDX_RC_COMM_DATA_START
#define RC_CMD_DATA_IDX_ERROR_CODE 		3 + IDX_RC_COMM_DATA_START
#define RC_CMD_DATA_IDX_GATE_MODE 		7 + IDX_RC_COMM_DATA_START
#define RC_CMD_DATA_IDX_PWR_LEVEL 		8 + IDX_RC_COMM_DATA_START

typedef enum{
	RC_CMD_ID_ROUTINE_INFO_REQ = 40,
	RC_CMD_ID_GET_SYS_INFO_REQ = 41,
	RC_CMD_ID_QUERY_RF_INFO	= 42,
	RC_CMD_ID_RC_COMMAND = 50
}EnumRCCommCmdID;

typedef enum RCConnStatus_e
{
	RC_CONN_STATUS_NO_VALID_RC = 0x00,			/* No Valid RC is connected to RF */
	RC_CONN_STATUS_RC_THROTTLE_LOCKED,			/* One Valid RC is connected and the throttle is locked */
	RC_CONN_STATUS_RC_THROTTLE_UNLOCKING,		/* One Valid RC is connected and user is unlocking the throttle */
	RC_CONN_STATUS_RC_THROTTLE_UNLOCKED,		/* One Valid RC is connected and the throttle is unlocked */
} RCConnStatus_t;


void RcComm_Init(StructUartCtrl*p, UART_HandleTypeDef *huart, CRC_HandleTypeDef *pHcrc, STRUCT_CANTxInterface *t, STRUCT_CANRxInterface *r);
uint32_t RcComm_CalCrc(StructUartCtrl *p,uint8_t *pDataStart, uint8_t size);
void RcComm_StartScan(StructUartCtrl*p);
void RcComm_GetUartMsgFromIsr( StructUartCtrl *p);
void RcComm_MsgHandler(StructUartCtrl*p,uint8_t *pData);
void RcComm_MsgHandlerVP3(StructUartCtrl*p,uint8_t *pData);
void RcComm_LoadRxDataFromIsr(StructUartCtrl*p);
void RcComm_MsgDecoder(StructUartCtrl*p);
void RcComm_Reset(StructUartCtrl*p);
RcCommError_t RcComm_QueryInfoFromRF (StructUartCtrl *p, RcCommDataId_t IdxIn);
void RcComm_10HzLoop(StructUartCtrl*p);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_txCpltCallback(UART_HandleTypeDef *huart);

#define RC_COMM_CTRL_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	{0},\
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},\
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},\
	0,\
	0,\
	0,\
	0,\
	{0x01,0x02,0x03},/* RCFwVer */ \
	{33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,48,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64},/* RCSN */ \
	{0x01,0x02,0x03,},/* RFFwVer */ \
	{33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,48,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64},/* RFSN */ \
	(functypeRcComm_Init)&RcComm_Init,\
	(functypeRcComm_CalCrc)&RcComm_CalCrc,\
	(functypeRcComm_StartScan)&RcComm_StartScan,\
	(functypeRcComm_GetUartMsgFromIsr)&RcComm_LoadRxDataFromIsr,\
	(functypeRcComm_MsgHandler)&RcComm_MsgHandlerVP3,\
	(functypeRcComm_MsgDecoder)&RcComm_MsgDecoder,\
	(functypeRcComm_QueryInfoFromRF)&RcComm_QueryInfoFromRF,\
	(functypeRcComm_10HzLoop)&RcComm_10HzLoop,\
	(functypeRcComm_Reset)&RcComm_Reset,\
}\


extern StructUartCtrl RCCommCtrl;

#endif /* INC_RCUARTCOMM_H_ */
