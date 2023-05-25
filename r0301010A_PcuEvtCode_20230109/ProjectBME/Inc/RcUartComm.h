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
	functypeRcComm_Init Init;
	functypeRcComm_CalCrc CalCrc;
	functypeRcComm_StartScan StartScan;
	functypeRcComm_GetUartMsgFromIsr GetMsgFromIsr;
	functypeRcComm_MsgHandler MsgHandler;
	functypeRcComm_MsgDecoder MsgDecoder;
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
	RC_COMM_TX_STATE_IDLE,
	RC_COMM_TX_STATE_TXREQ,
	RC_COMM_TX_STATE_TRANSFERING,
	RC_COMM_TX_STATE_COMPLETE,
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
	(functypeRcComm_Init)&RcComm_Init,\
	(functypeRcComm_CalCrc)&RcComm_CalCrc,\
	(functypeRcComm_StartScan)&RcComm_StartScan,\
	(functypeRcComm_GetUartMsgFromIsr)&RcComm_LoadRxDataFromIsr,\
	(functypeRcComm_MsgHandler)&RcComm_MsgHandlerVP3,\
	(functypeRcComm_MsgDecoder)&RcComm_MsgDecoder,\
	(functypeRcComm_10HzLoop)&RcComm_10HzLoop,\
	(functypeRcComm_Reset)&RcComm_Reset,\
}\


extern StructUartCtrl RCCommCtrl;

#endif /* INC_RCUARTCOMM_H_ */
