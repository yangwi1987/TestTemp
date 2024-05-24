/*
 * ICANInterface.h
 *
 *  Created on: 2020年5月27日
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_ICANINTERFACE_H_
#define INC_ICANINTERFACE_H_

#include "stdio.h"
#include "Protocol.h"

#define ID_MATCH	0
#define ID_NO_MATCH	1
/*=========================================
 * Interface
 =========================================*/
typedef struct
{
	uint8_t Reserved;
} STRUCT_CANRxInterface;

typedef enum DebugFloatIdx_e
{
  IDX_MOTOR0_TEMP = 0,
  IDX_IU_FBK,
  IDX_IV_FBK,
  IDX_IW_FBK,
  IDX_PREC,
  IDC_DEBUG_FLOAT_MAX = 64, /* no more than this value*/
} DebugFloatIdx_t;

typedef struct
{
	// debug
	float Debugf[IDC_DEBUG_FLOAT_MAX];
} STRUCT_CANTxInterface;

typedef uint8_t (*pRxTranslate)( uint32_t, uint8_t*, void *, void *);
typedef uint8_t (*pTxTranslate)( uint32_t, uint8_t*, void *, void *);

typedef struct {
	uint16_t				TxPeriodMs;
	uint16_t				PeriodicTxIDNumber;
	uint32_t				PeriodicTxIDTable[10];
	pRxTranslate			RxTranslate;
	pTxTranslate			TxTranslate;
} CANProtocol;

#define CANRXINFO_DEFAULT { \
	0,      /* OutputModeCmd */\
}\

#define CANTXINFO_DEFAULT { \
	{0.0f}, /*Debugf*/\
}\

#endif /* INC_ICANINTERFACE_H_ */
