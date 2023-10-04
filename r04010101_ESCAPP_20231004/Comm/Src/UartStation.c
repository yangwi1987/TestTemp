/*
 * UartStation.c
 *
 *  Created on: 2019年12月20日
 *      Author: MikeSFWen
 */
#include "UartStation.h"
#if USE_UART_COMMUNICATION

#define MIN(a,b)	((a)<(b)? (a): (b))
#define CIRCLE(a,b)	((a)<(b)? (a): 0)

unsigned int UART_RxCount = 0;
unsigned int UART_TxCount = 0;

void HandlePacket(UART_Module* v);
void RecvHeaderMode(UART_Module* v);
void RecvDataMode(UART_Module* v);
void PushRxBuff(UART_Module* v);

// =============== Handle Receive & Packet ====================
void CleanBuff(UART_Module* v)
{
	for( int i = 0; i < 32; i++ )
		v->RxBuff[i] = 0;
	v->RxIndex = 0;
	v->RxPacketCount = 0;
}

void PushRxBuff(UART_Module* v)
{
	v->RxBuff[v->RxIndex++] = v->RawRxBuff[0] | v->RawRxBuff[1] << 8;
	v->RxBuff[v->RxIndex++] = v->RawRxBuff[2] | v->RawRxBuff[3] << 8;
}

void RxHandleRecv(UART_Module* v)
{
	// Enter Each 4 bytes (1 packets received)
	if( v->HeaderMode ) {
		CleanBuff(v);
		PushRxBuff(v);
		RecvHeaderMode(v);
	} else {
		PushRxBuff(v);
	}

	v->RxPacketCount--;
	if( v->RxPacketCount == 0 ) {
		RecvDataMode(v);
	}
}

void RecvHeaderMode(UART_Module* v)
{
	uint16_t Header = v->RxBuff[0];
//	uint16_t TargerID;

	v->RxPacketCount = (Header & H_MASK_PACKETCT) >> 8;
	v->Station = Header & H_MASK_STATION;
	v->CmdMode = Header & H_MASK_CMDMODE;
	v->DataSize = Header & H_MASK_SIZE;
	v->WDT = Header & H_MASK_WDT;

//	if( v->WDT != Header & H_MASK_WDT ) {
//		v->WDT = Header & H_MASK_WDT;
//	} else {
//		return 0;
//	}

	v->HeaderMode = 0;
}

void RecvDataMode(UART_Module* v)
{
//	uint16_t crc = 0;

// Finish Receive packet and then Check CRC16
//	CRC16(&crc, v->RxBuff, v->RxBuffPos);
//	if(v->RxBuff[v->RxBuffPos] == crc)
	HandlePacket(v);

	// Reset Receive Machine for next packet
	for( int i = 0; i < UART_FIFO_16BIT_SIZE; i++ )
	{
		v->RawRxBuff[i] = 0;
		v->RawRxBuff[2*i] = 0;
	}
	v->HeaderMode = 1;
	v->RxIndex = 0;
}

void HandlePacket(UART_Module* v)
{
	int i = 0;

//	uint16_t TargetID = v->Station;
	uint16_t TargetID = 1; // Axis 1
	uint16_t DataNum = (v->DataSize == SIZE_32BIT) ? (v->RxIndex - 2) / 3 : (v->RxIndex - 2) / 2;

	switch( v->CmdMode ) {
	case CMD_MODE_READ_PARAM:
		if( v->DataSize == SIZE_32BIT ) {
			for(i = 0; i < DataNum; i++)
			{
				int  IdxPos = 3 * i + 1;
				uint16_t Index = v->RxBuff[IdxPos];
				int32_t Data;
				Data = v->RxAccessParam(TargetID, Index, 0, 0, NULL);
				v->RxBuff[IdxPos + 1] = Data;
				v->RxBuff[IdxPos + 2] = Data >> 16;
			}
		} else if( v->DataSize == SIZE_16BIT ) {
			for(i = 0; i < DataNum; i++)
			{
				int  IdxPos = 2 * i + 1;
				uint16_t Index = v->RxBuff[IdxPos];
				v->RxBuff[IdxPos + 1] = v->RxAccessParam(TargetID, Index, 0, 0, NULL);
			}
		}
		break;

	case CMD_MODE_WRITE_PARAM:
		if( v->DataSize == SIZE_32BIT ) {
			for(i = 0; i < DataNum; i++)
			{
				int  IdxPos = 3 * i + 1;
				uint16_t Index = v->RxBuff[IdxPos];
				if( Index == 0 ) {
					break;
				} else {
					int32_t Data = v->RxBuff[IdxPos + 1] | ((int32_t)v->RxBuff[IdxPos + 2] << 16);
					Data = v->RxAccessParam(TargetID, Index, &Data, 1, NULL);
					v->RxBuff[IdxPos + 1] = Data;
					v->RxBuff[IdxPos + 2] = Data >> 16;
				}
			}
		} else if( v->DataSize == SIZE_16BIT ) {
			for(i = 0; i < DataNum; i++)
			{
				int  IdxPos = 2 * i + 1;
				uint16_t Index = v->RxBuff[IdxPos];
				if( Index == 0 ) {
					break;
				} else {
					int32_t Data = (int32_t)v->RxBuff[IdxPos + 1];
					v->RxBuff[IdxPos + 1] = v->RxAccessParam(TargetID, Index, &Data, 1, NULL);
				}
			}
		}
		break;

	case CMD_MODE_READ_SCOPE:
		break;
	}
	UART_RxCount++;

	HAL_UART_Transmit(v->pUARTx, (uint8_t *)v->RxBuff, v->RxIndex * 2, 0xFFFF);
}

void UU_PutChar(UART_HandleTypeDef* USARTx, uint8_t ch)
{
	HAL_UART_Transmit(USARTx, &ch, 1, 0xFFFF);
}

void UU_PutUint16(UART_HandleTypeDef* USARTx, uint16_t x)
{
  char value[5]; //a temp array to hold results of conversion
  int i = 0; //loop index

  do
  {
    value[i++] = (char)(x % 10) + '0'; //convert integer to character
    x /= 10;
  } while(x);

  while(i) //send data
  {
    UU_PutChar(USARTx, value[--i]);
  }
}

void UU_PutUint32(UART_HandleTypeDef* USARTx, uint32_t x)
{
  char value[10]; //a temp array to hold results of conversion
  int i = 0; //loop index

  do
  {
    value[i++] = (char)(x % 10) + '0'; //convert integer to character
    x /= 10;
  } while(x);

  while(i) //send data
  {
    UU_PutChar(USARTx, value[--i]);
  }
}

void HMI_SendData_Two32Bit(UART_HandleTypeDef* USARTx, uint32_t timestamp, uint32_t x1, uint32_t x2)
{
	UU_PutUint32(USARTx, timestamp);
	UU_PutChar(USARTx, '\t');
	UU_PutUint32(USARTx, x1);
	UU_PutChar(USARTx, '\t');
	UU_PutUint32(USARTx, x2);
	UU_PutChar(USARTx, '\n');
	UU_PutChar(USARTx, '\r');
}

void HMI_SendData_Four16Bit(UART_HandleTypeDef* USARTx, uint32_t timestamp, uint16_t x1, uint16_t x2, uint16_t x3, uint16_t x4)
{
	UU_PutUint32(USARTx, timestamp);
	UU_PutChar(USARTx, '\t');
	UU_PutUint16(USARTx, x1);
	UU_PutChar(USARTx, '\t');
	UU_PutUint16(USARTx, x2);
	UU_PutChar(USARTx, '\t');
	UU_PutUint16(USARTx, x3);
	UU_PutChar(USARTx, '\t');
	UU_PutUint16(USARTx, x4);
	UU_PutChar(USARTx, '\n');
	UU_PutChar(USARTx, '\r');
}
#endif