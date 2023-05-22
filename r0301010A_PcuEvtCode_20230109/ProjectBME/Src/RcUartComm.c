/*
 * RcUartComm.c
 *
 *  Created on: 2022年7月21日
 *      Author: Will.Yang.CYY
 */

#include "RcUartComm.h"
#define ABS(x) 	( (x) > 0 ? (x) : -(x) )


__IO uint32_t uwCRCValue = 0;


uint8_t CrcBuff[40];

StructUartCtrl RCCommCtrl = RC_COMM_CTRL_DEFAULT;

uint32_t RcComm_CalCrc(StructUartCtrl *p,uint8_t *pDataStart, uint8_t size){
	uint8_t i=0;
	for( i = 0; i < size; i++ ){
		CrcBuff[i] = *(pDataStart+i);
	}
	uwCRCValue = HAL_CRC_Calculate(p->phcrc, (uint32_t*)CrcBuff, size);

#if RC_COMM_BYPASS_CRC
	uwCRCValue = 0;
#else
#endif
	return uwCRCValue;
}



void RcComm_MsgHandler(StructUartCtrl*p, uint8_t *pData ){
	int16_t lTempI16 = 0;
	uint32_t CrcResultU32 = 0;
//	uint16_t ulTemp16 = 0;


	if(p->RxFlag == RC_COMM_RX_STATE_COMPLETE){
		switch (*(pData+IDX_RC_COMM_CMD_ID)){
			case  RC_CMD_ID_ROUTINE_INFO_REQ:{
				//send information to RC
				p->TxBuff[0] = RC_COMM_HEAD_VALUE;
				p->TxBuff[1] = 21;
				p->TxBuff[2] = 0;
				p->TxBuff[3] = RC_CMD_ID_ROUTINE_INFO_REQ;
				//DC BUS Voltage
				lTempI16 = p->pTxInterface->VoltDcBu0P1V;
				p->TxBuff[4] = lTempI16 & 0xFF;
				p->TxBuff[5] = lTempI16 >>8;
				p->TxBuff[6] = p->pTxInterface->PcuStateReport;
				//Motor RPM
				lTempI16 = ABS(p->pTxInterface->MotorRpm);
				p->TxBuff[7] = lTempI16 & 0xFF;
				p->TxBuff[8] = lTempI16 >>8;
				//T MOS-1
				lTempI16 = p->pTxInterface->NTCTemp[1]*10;
				p->TxBuff[9] = lTempI16 & 0xFF;
				p->TxBuff[10] = lTempI16 >>8;
				//TCap
				lTempI16 = p->pTxInterface->NTCTemp[3]*10;
				p->TxBuff[11] = lTempI16 & 0xFF;
				p->TxBuff[12] = lTempI16 >>8;
				//TBat
				lTempI16 = p->pRxInterface->BmsReportInfo.MaxCellTemp;
				p->TxBuff[13] = lTempI16 & 0xFF;
				p->TxBuff[14] = lTempI16 >>8;
				//SOC
				p->TxBuff[15] = 78;
				//Error Code
				p->TxBuff[16] = p->pTxInterface->DebugError[0];
				p->TxBuff[17] = p->pTxInterface->DebugError[1];
				p->TxBuff[18] = p->pTxInterface->DebugError[2];
				p->TxBuff[19] = p->pTxInterface->DebugError[3];
				//Applied PWR power
				p->TxBuff[20] = 0;
				//FOIL MODE
				p->TxBuff[21] = p->pRxInterface->OutputModeCmd;
				CrcResultU32 = p->CalCrc(p,&p->TxBuff[IDX_RC_COMM_DLC_L],p->TxBuff[IDX_RC_COMM_DLC_L]);
				p->TxBuff[22] = CrcResultU32 & 0x00FF;
				p->TxBuff[23] = (CrcResultU32 &0xFF00) >>8;
				p->TxBuff[24] = RC_COMM_END_VALUE;
				p->TxDlc = 25;
				p->TxFlag = RC_COMM_TX_STATE_TXREQ;

				break;
			}
			case  RC_CMD_ID_GET_SYS_INFO_REQ:{
				break;
			}
			case  RC_CMD_ID_RC_COMMAND:{

				if(*(pData+6)==0x01)
				{
					p->pRxInterface->ThrottleCmd = ((uint16_t)(*(pData+4))>100)? 100 : (uint16_t)(*(pData+4));
					p->pRxInterface->PowerLevel = (uint16_t)(*(pData+7));
					p->TimeoutCnt=0;
					p->RcEnable = 1;
					p->pRxInterface->RcConnStatus = 1;
				}
				else
				{
          p->pRxInterface->RcConnStatus = 0;
				}
				break;
			}
			default : {
				break;
			}

		}
		p->RxFlag = RC_COMM_RX_STATE_IDLE;
	}


}


void RcComm_LoadRxDataFromIsr(StructUartCtrl*p){
	p->RxBuff[p->RxPutIdx]=p->RxUnit[0];
	p->RxPutIdx++;
	if(p->RxPutIdx>=RC_COMM_RX_BUFF_SIZE){
		p->RxPutIdx = 0;
	}
}


void RcComm_MsgDecoder(StructUartCtrl*p){
	uint16_t DLCtemp = 0;
	uint16_t CrcResultTemp=0x01;
	uint8_t PutIdxTemp=0;
	uint8_t ReadIdxTemp=0;
	uint8_t BufferForCrc[RC_COMM_RX_BUFF_SIZE];


#if !RC_COMM_DMA_USAGE
	PutIdxTemp = p->RxPutIdx;
#else
	PutIdxTemp =RC_COMM_RX_BUFF_SIZE- p->pTarget->hdmarx->Instance->CNDTR;
#endif
	if(PutIdxTemp < p->RxReadIdx){
		PutIdxTemp+=RC_COMM_RX_BUFF_SIZE;
	}

	for(ReadIdxTemp = p->RxReadIdx; ReadIdxTemp < PutIdxTemp; ReadIdxTemp++){
		if(p->RxBuff[((ReadIdxTemp)%RC_COMM_RX_BUFF_SIZE)] == RC_COMM_HEAD_VALUE){
			//possible header of msg, see if DLC is received
			if((ReadIdxTemp+IDX_RC_COMM_DLC_H) < PutIdxTemp){
				DLCtemp = p->RxBuff[(ReadIdxTemp + IDX_RC_COMM_DLC_H)%RC_COMM_RX_BUFF_SIZE];
				DLCtemp <<= 8;
				DLCtemp += p->RxBuff[(ReadIdxTemp + IDX_RC_COMM_DLC_L)%RC_COMM_RX_BUFF_SIZE];
				if(DLCtemp < RC_COMM_MSG_DLC_MAX){
					if((ReadIdxTemp+DLCtemp+3) < PutIdxTemp){
						if(p->RxBuff[(ReadIdxTemp+DLCtemp+3)%RC_COMM_RX_BUFF_SIZE] == RC_COMM_END_VALUE){
							for(uint8_t j=0; j <= DLCtemp+3; j++){
								BufferForCrc[j] = p->RxBuff[(ReadIdxTemp+j)%RC_COMM_RX_BUFF_SIZE];
							}

							CrcResultTemp = p->CalCrc(p,&BufferForCrc[IDX_RC_COMM_DLC_L],DLCtemp+2);
							if(CrcResultTemp == 0){
								p->RxReadIdx = (ReadIdxTemp+DLCtemp+3)%RC_COMM_RX_BUFF_SIZE;
								p->RxFlag = RC_COMM_RX_STATE_COMPLETE;
								p->MsgHandler(p,BufferForCrc);
								break;
								//finish search & record read index
							}else{
								//wrong CRC,search for next header
							}
						}else{
							//not correct header & end ,keep searching
						}
					}else{
						//End of data is not received yet,  record read index & wait for left data
						p->RxReadIdx = ReadIdxTemp%RC_COMM_RX_BUFF_SIZE;
						break;
					}

				}else{
					//wrong DLC, keep search for next header
				}
			}else{
				//DLC is not received yet,  record read index & wait for left data
				p->RxReadIdx = ReadIdxTemp%RC_COMM_RX_BUFF_SIZE;
				break;
			}
		}
	}



	if(p->TxFlag == RC_COMM_TX_STATE_TXREQ){
			HAL_UART_Transmit_DMA(p->pTarget, p->TxBuff, p->TxDlc);
			p->TxFlag = RC_COMM_TX_STATE_TRANSFERING;
	}


}

void RcComm_StartScan(StructUartCtrl*p){
#if !RC_COMM_DMA_USAGE
	HAL_UART_Receive_IT(p->pTarget, p->RxUnit, 1);
#endif
}


void RcComm_Init(StructUartCtrl*p, UART_HandleTypeDef *huart, CRC_HandleTypeDef *pHcrc, STRUCT_CANTxInterface *t, STRUCT_CANRxInterface *r){
	p->pTarget = huart;
	p->phcrc = pHcrc;
	p->pTxInterface = t;

	p->pRxInterface =r;
#if RC_COMM_DMA_USAGE
	HAL_UART_AbortReceive(p->pTarget);
	HAL_UART_DMAStop(p->pTarget);
	HAL_UART_Receive_DMA(p->pTarget,p->RxBuff,RC_COMM_RX_BUFF_SIZE);
#endif
}



void RcComm_10HzLoop(StructUartCtrl*p){

	if(p->TimeoutCnt < RC_COMM_TIMEOUT_THRESHOLD_100MS){
		p->TimeoutCnt++;
	}else{

	}

	if(p->TimeoutCnt == RC_COMM_TIMEOUT_THRESHOLD_100MS){
		p->pRxInterface->ThrottleCmd =0;
		p->Reset(p);
#if RC_TEIMOUT_TEST
		p->TxBuff[0] = 0x77;
		p->TxBuff[2] = 0x88;
		p->TxBuff[3] = 0x55;
		p->TxBuff[3] = 0x66;
		p->TxFlag = RC_COMM_TX_STATE_TXREQ;
		HAL_UART_Transmit_DMA(p->pTarget, p->TxBuff, 4);
#endif
	}


}

void RcComm_Reset(StructUartCtrl*p){
#if !RC_COMM_DMA_USAGE
	HAL_UART_AbortReceive_IT(p->pTarget);
	HAL_UART_AbortTransmit_IT(p->pTarget);
#endif
	memset(p->TxBuff,0,RC_COMM_TX_BUFF_SIZE);
	memset(p->RxBuff,0,RC_COMM_RX_BUFF_SIZE);
	p->RxReadIdx=0;
	p->RxPutIdx=0;
	p->TimeoutCnt=0; // Reset time out counter after "time out warning" is triggered.
	p->RxDlc=0;
//	p->RcEnable=0;
	p->RxFlag = RC_COMM_RX_STATE_IDLE;
	p->TxFlag = RC_COMM_TX_STATE_IDLE;
#if RC_COMM_DMA_USAGE
	HAL_UART_AbortReceive(p->pTarget);
	HAL_UART_DMAStop(p->pTarget);
	HAL_UART_Receive_DMA(p->pTarget,p->RxBuff,RC_COMM_RX_BUFF_SIZE);
#else
	p->StartScan(p);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if !RC_COMM_DMA_USAGE
	RCCommCtrl.GetMsgFromIsr(&RCCommCtrl);
#else
#endif
}

void HAL_UART_txCpltCallback(UART_HandleTypeDef *huart)
{
	RCCommCtrl.TxFlag = RC_COMM_TX_STATE_IDLE;
}
