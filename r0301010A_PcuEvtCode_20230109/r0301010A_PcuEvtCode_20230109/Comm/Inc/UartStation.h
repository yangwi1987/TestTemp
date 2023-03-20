/*
 * UartStation.h
 *
 *  Created on: 2019年12月20日
 *      Author: MikeSFWen
 */

#ifndef INC_UARTSTATION_H_
#define INC_UARTSTATION_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "stdio.h"
#include "ConstantParamAndUseFunction.h"

#if USE_UART_COMMUNICATION
extern UART_HandleTypeDef huart3;

#define UART_FIFO_16BIT_SIZE 	32

// ========= Packet Header ========
// Header Mask
#define H_MASK_ALARM	 0x8000	// 15 bit
#define H_MASK_WDT		 0x7000	// 12-14 bit
#define H_MASK_PACKETCT	 0x0F00 // 8-11 bit
#define H_MASK_CMDMODE	 0x00E0	// 5-7 bit
#define H_MASK_SIZE		 0x0010	// 4 bit
#define H_MASK_STATION	 0x000F	// 0-3 bit
// =====================================

typedef enum {
	STATION_NONE = 0x0000,
	STATION_BROADCAST = 0x0001,
	STATION_AXIS1 = 0x0010,
	STATION_AXIS2 = 0x0011
} UART_STATION;

typedef enum {
	CMD_MODE_READ_PARAM = 0x0000,
	CMD_MODE_WRITE_PARAM = 0x0020,
	CMD_MODE_READ_SCOPE = 0x0080
} UART_CMD_MODE;

typedef enum {
	SIZE_16BIT = 0x0000,
	SIZE_32BIT = 0x0010
} UART_SIZE;

typedef struct {
	UART_HandleTypeDef* pUARTx;
	uint16_t HeaderMode;	// 1: Recv header
	uint16_t RxIndex;		// Received Buffer Size in 16-bit
	uint16_t RxPacketCount;	// Packet Count in 32-bit
	uint16_t Station;
	uint16_t DataSize;
	uint16_t CmdMode;
	uint16_t WDT;
	uint16_t AlarmFlag;
	uint8_t  RawRxBuff[4];	// each Receive Buffer is 4 x 8bytes (1 packets)
	uint16_t RxBuff[32];
	void (*RxHandleRecv)(void*);
	int32_t (*RxAccessParam)(uint16_t, uint16_t, int32_t*, uint16_t, uint8_t*);
} UART_Module;

#define UART_MODULE_DEFAULT {\
	&huart3,  \
	1,			\
	0,			\
	0,			\
	0,			\
	0,			\
	0,			\
	0,			\
	0,			\
	{0},		\
	{0},		\
	(void(*)(void*))RxHandleRecv,	\
	0 \
}

// ========= User Interface =============
// Handle Receive Packet
void RxHandleRecv(UART_Module* v);
// ======================================

extern unsigned int UART_RxCount;
extern unsigned int UART_TxCount;

//void CRC16(Uint16* CRC16, Uint16 *pWord, Uint16 Len);

void UU_PutChar(UART_HandleTypeDef* USARTx, uint8_t ch);
void UU_PutUint16(UART_HandleTypeDef* USARTx, uint16_t x);
void UU_PutUint32(UART_HandleTypeDef* USARTx, uint32_t x);

extern void HMI_SendData_Two32Bit(UART_HandleTypeDef* USARTx, uint32_t timestamp, uint32_t x1, uint32_t x2);
extern void HMI_SendData_Four16Bit(UART_HandleTypeDef* USARTx, uint32_t timestamp, uint16_t x1, uint16_t x2, uint16_t x3, uint16_t x4);

#endif

#endif /* INC_UARTSTATION_H_ */
