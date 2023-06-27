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

#define RC_COMM_RC_FW_VER_SIZE	20	/* RC firmware version array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RC_SN_SIZE		20	/* RC serial number array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RF_FW_VER_SIZE	20	/* RF firmware version array size in byte. Todo: clarify the exact nuber with TBS */
#define RC_COMM_RF_SN_SIZE		20	/* RF serial number array size in byte. Todo: clarify the exact nuber with TBS */

#define RC_TEIMOUT_TEST 0
#define RC_COMM_DMA_USAGE 1
#define RC_COMM_BYPASS_CRC 0

#define RC_COMM_MSG_DLC_MAX  20
#define RC_COMM_RX_BUFF_SIZE 40
#define RC_COMM_TX_BUFF_SIZE 40
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
	uint8_t RcEnable;
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
	RC_COMM_TX_STATE_FILLING,		/* UART Tx buffer is filling bt other functions*/
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
	{0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x01,0x02,0x03,0x04,0x05},/* RCFwVer */ \
	{0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x11,0x12,0x13,0x14,0x15},/* RCSN */ \
	{0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x21,0x22,0x23,0x24,0x25},/* RFFwVer */ \
	{0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x31,0x32,0x33,0x34,0x35},/* RFSN */ \
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
