/*
 * ExtFlash.c
 *
 *  Created on: Jul 17, 2020
 *      Author: Mike.Wen.SFW
 */

#include "UtilityBase.h"
#include "string.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"
#include "stdio.h"
#include "AlarmMgr.h"
#include "ParamTable.h"
#include "ExtFlash.h"

#define MAX_NUMBER_Of_TABLE 7

const uint8_t ParaReadEnableTable1p4[PAGE_NUM][ROW_NUM] =
{
	// ParaReadEnableTable2p4 is created for External Flash 3.0.1.1.
	/* page1 SysParamTable P0-00 ~ P1-99*/
	{	/* Row000 */	0x06,
		/* Row001 */	0x00,
		/* Row002 */	0x00,
		/* Row003 */	0x00,
		/* Row004 */	0x00,
		/* Row005 */	0x00,
		/* Row006 */	0x80,
		/* Row007 */	0xFF,
		/* Row010 */	0xFF,
		/* Row011 */	0xFF,
		/* Row012 */	0x7F,
		/* Row013 */	0xF0,
		/* Row014 */	0xF3,
		/* Row015 */	0xFF,
		/* Row016 */	0xFF,
		/* Row017 */	0x3F,
		/* Row020 */	0xF0,
		/* Row021 */	0x00,
		/* Row022 */	0x00,
		/* Row023 */	0x00,
		/* Row024 */	0x00,
		/* Row025 */	0x00,
		/* Row026 */	0x00,
		/* Row027 */	0x00,
		/* Row030 */	0x00	},

	/* page2 PcuParamTable P2-00 ~ P3-99 , P3-00 ~ P3-99 are alarm threshold*/
	{	/* Row000 */	0x00,
		/* Row001 */	0x04,
		/* Row002 */	0x00,
		/* Row003 */	0x00,
		/* Row004 */	0x00,
		/* Row005 */	0xFF,
		/* Row006 */	0xFF,
		/* Row007 */	0xF3,
		/* Row010 */	0xFF,
		/* Row011 */	0x3F,
		/* Row012 */	0xFF,
		/* Row013 */	0x0F,
		/* Row014 */	0xF0,
		/* Row015 */	0xFF,
		/* Row016 */	0xFF,
		/* Row017 */	0xFF,
		/* Row020 */	0xFF,
		/* Row021 */	0xFF,
		/* Row022 */	0xFF,
		/* Row023 */	0xFF,
		/* Row024 */	0xFF,
		/* Row025 */	0xFF,
		/* Row026 */	0xFF,
		/* Row027 */	0xFF,
		/* Row030 */	0xFF,	}
};

static inline void ExtFlash_Write_DTC_Data_by_ADDR( ExtFlash_t *v, uint32_t Address, uint8_t *pData );

/*
 * External flash communication or basic function
 */

void ExtFlash_NORD( ExtFlash_t *v, uint32_t Address, uint16_t TotalLen )
{
	uint16_t i;

	// Clear Buffer
	for( i = 0; i < SPI_BUF_LEN; i++ )
	{
		v->TxBuff[i] = v->RxBuff[i] = 0;
	}

	// set cmd and Address
	v->TxBuff[0] = NORD;
	v->TxBuff[1] = (Address & 0x00FF0000) >> 16;
	v->TxBuff[2] = (Address & 0x0000FF00) >> 8;
	v->TxBuff[3] = (Address & 0x000000FF);

	// send cmd
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, v->TxBuff, v->RxBuff, TotalLen, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );
}

void ExtFlash_PP( ExtFlash_t *v, uint32_t Address, uint8_t *pData, uint16_t TotalLen )
{
	uint16_t i;

	// Clear Buffer
	for( i = 0; i < SPI_BUF_LEN; i++ )
	{
		v->TxBuff[i] = v->RxBuff[i] = 0;
	}

	// set cmd and Address
	v->TxBuff[0] = PP;
	v->TxBuff[1] = (Address & 0x00FF0000) >> 16;
	v->TxBuff[2] = (Address & 0x0000FF00) >> 8;
	v->TxBuff[3] = (Address & 0x000000FF);

	for( i = PP_LEN; i < TotalLen; i++ )
	{
		v->TxBuff[i] = *(pData + i - PP_LEN);
	}

	// send cmd
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, v->TxBuff, v->RxBuff, TotalLen, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );
}

void ExtFlash_WREN( void )
{
	uint8_t CMD = WREN;

	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_Transmit(&hspi1, &CMD, WREN_LEN, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );
}

void ExtFlash_RDSR( ExtFlash_t *v )
{
	uint8_t TxTemp[RDSR_LEN] = {RDSR, 0};
	uint8_t RxTemp[RDSR_LEN] = {0, 0};

	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, TxTemp, RxTemp, RDSR_LEN, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );

	v->StatusRegister = RxTemp[1];
}

void ExtFlash_SER( ExtFlash_t *v, uint32_t Address )
{
	uint16_t i;

	ExtFlash_WREN();

	// Clear Buffer
	for( i = 0; i < SPI_BUF_LEN; i++ )
	{
		v->TxBuff[i] = v->RxBuff[i] = 0;
	}

	// set cmd and Address
	v->TxBuff[0] = SER;
	v->TxBuff[1] = (Address & 0x00FF0000) >> 16;
	v->TxBuff[2] = (Address & 0x0000FF00) >> 8;
	v->TxBuff[3] = (Address & 0x000000FF);

	// send cmd
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, v->TxBuff, v->RxBuff, SER_LEN, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );

	// wait until Process done
	v->RetryCount = 0;

	do
	{
		ExtFlash_RDSR(v);
		v->RetryCount++;
		if( v->RetryCount == SER_TIMEOUT )
		{
			v->AlarmStatus |= FLASHERROR_DAMAGED;
			break;
		}
	} while( (v->StatusRegister & SR_WIP) == SR_WIP );
}

void ExtFlash_BER64( ExtFlash_t *v, uint32_t Address )
{
	uint16_t i;

	ExtFlash_WREN();

	// Clear Buffer
	for( i = 0; i < SPI_BUF_LEN; i++ )
	{
		v->TxBuff[i] = v->RxBuff[i] = 0;
	}

	// set cmd and Address
	v->TxBuff[0] = BER64;
	v->TxBuff[1] = (Address & 0x00FF0000) >> 16;
	v->TxBuff[2] = (Address & 0x0000FF00) >> 8;
	v->TxBuff[3] = (Address & 0x000000FF);

	// send cmd
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, v->TxBuff, v->RxBuff, SER_LEN, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );

	// wait until Process done
	v->RetryCount = 0;

	do
	{
		ExtFlash_RDSR(v);
		v->RetryCount++;
		if( v->RetryCount == SER_TIMEOUT * 4 )
		{
			v->AlarmStatus |= FLASHERROR_DAMAGED;
			break;
		}
	} while( (v->StatusRegister & SR_WIP) == SR_WIP );
}

void ExtFlash_CER( ExtFlash_t *v )
{
	uint16_t i;

	ExtFlash_WREN();

	// Clear Buffer
	for( i = 0; i < SPI_BUF_LEN; i++ )
	{
		v->TxBuff[i] = v->RxBuff[i] = 0;
	}

	// set cmd and Address
	v->TxBuff[0] = CER;

	// send cmd
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive(&hspi1, v->TxBuff, v->RxBuff, CER_LEN, TIMEOUT );
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );

	// wait until Process done
	v->RetryCount = 0;

	do
	{
		ExtFlash_RDSR(v);
		v->RetryCount++;
		if( v->RetryCount == (SER_TIMEOUT*4) )
		{
			v->AlarmStatus |= FLASHERROR_DAMAGED;
			break;
		}
	} while( (v->StatusRegister & SR_WIP) == SR_WIP );
}

//QW: QWord = 64 bit
static uint32_t ExtFlash_SearchQW( ExtFlash_t *v, uint32_t MinQWAddr, uint32_t MaxQWAddr, uint32_t TotalQWSize, uint32_t AlarmCode )
{
	uint8_t NextQWReadyFlag = 0, ThisQWReadyFlag = 0;
	uint32_t TempQWAddr = MinQWAddr;
	uint16_t i;

	// Search for the total time Address between MinQWAddr ~ the address before MaxQWAddr
	for( i = 1; i < TotalQWSize; i++ )
	{
		TempQWAddr = MinQWAddr + FLASH_QW_SIZE * (i - 1);

		ExtFlash_NORD( v, TempQWAddr + QW_READY_OFFSET, NORD_LEN + 1 );
		ThisQWReadyFlag = v->RxBuff[NORD_LEN];
		ExtFlash_NORD( v, (TempQWAddr + FLASH_QW_SIZE) + QW_READY_OFFSET, NORD_LEN + 1 );
		NextQWReadyFlag = v->RxBuff[NORD_LEN];

		if( ThisQWReadyFlag == CHECK_WORD && NextQWReadyFlag == 0xFF )
		{
			// Search Success and return.
			return TempQWAddr;
		}
	}

	// If the above search is fail, and then search for the QW Address at MaxQWAddr
	ThisQWReadyFlag = NextQWReadyFlag; // This flag is in MaxQWAddr.
	ExtFlash_NORD( v, MinQWAddr + QW_READY_OFFSET, NORD_LEN + 1 );
	NextQWReadyFlag = v->RxBuff[NORD_LEN];

	if( ThisQWReadyFlag == CHECK_WORD && NextQWReadyFlag == 0xFF )
	{
		return MaxQWAddr;
	}
	else
	{
		// search fail or uninitialized, so register warning
		v->AlarmStatus |= AlarmCode;

		// use the first pack
		return MinQWAddr;
	}
}

void ExtFlash_Init( ExtFlash_t *v )
{
	uint32_t TempAddr = 0;

	// Pull High SPI Hold Pin (Low-Active) to enable external flash memory
	HAL_GPIO_WritePin( F_SPI_HOLD_GPIO_Port, F_SPI_HOLD_Pin, GPIO_PIN_SET );

	// Pull High SPI CS Pin
	HAL_GPIO_WritePin( F_SPI_CS_GPIO_Port, F_SPI_CS_Pin, GPIO_PIN_SET );

	TempAddr = ExtFlash_SearchQW( v, TOTAL_TIME_FIRST_ADDR, TOTAL_TIME_END_ADDR, TOTAL_TotalTime_QW, FLASHERROR_NULL_TOTAL_TIME );
	if( v->AlarmStatus & FLASHERROR_NULL_TOTAL_TIME )
	{
		ExtFlash_SER( v, SECTOR2_PACK4_ADDR );
		v->TotalTimeQWAddress = TOTAL_TIME_END_ADDR;
	}
	else
	{
		v->TotalTimeQWAddress = TempAddr;
	}
}

/*
 * parameter backup function
 */
void ExtFlash_ReadParamPack( ExtFlash_t *v, uint32_t PACK_ADDR )
{
	// Read Pack 0
	uint16_t i, j = 0;

	for( i = 0; i < PP_COUNT_PER_PACK; i++ )
	{
		ExtFlash_NORD( v, PACK_ADDR + i * FLASH_PP_SIZE, NORD_LEN + FLASH_PP_SIZE );
		for( j = 0; j < FLASH_PP_SIZE; j++ )
		{
			*((uint8_t *)(&v->ParamPack) + i * FLASH_PP_SIZE + j) = v->RxBuff[NORD_LEN + j];
		}
	}
}

void ExtFlash_WriteParamPack( ExtFlash_t *v, uint32_t PACK_ADDR )
{
	uint16_t i = 0;

	for( i = 0; i < PP_COUNT_PER_PACK; i++ )
	{
		// Write Enable
		ExtFlash_WREN();

		// Page 4 + 256 bytes
		ExtFlash_PP( v, PACK_ADDR + i * FLASH_PP_SIZE, (uint8_t *)(&v->ParamPack) + i * FLASH_PP_SIZE, PP_LEN + FLASH_PP_SIZE );

		// Check done
		ExtFlash_RDSR(v);
		do
		{
			ExtFlash_RDSR(v);
		}
		while( (v->StatusRegister & SR_WIP) == SR_WIP );

		// Read Back
		ExtFlash_NORD( v, SECTOR0_PACK0_ADDR + i * FLASH_PP_SIZE, NORD_LEN + FLASH_PP_SIZE );

		ExtFlash_RDSR(v);
	}
}

void ExtFlash_LoadParam( ExtFlash_t *v )
{
	uint8_t NextPackReadyFlag = 0, ThisPackReadyFlag = 0; // avoid to be used before initialization.
	uint32_t PackAddr = 1;
	uint16_t i;
	uint8_t tempCheckSum = 0;

	// Search for the ParamPack Address Pack 0 ~ 2
	for( i = 1; i < TOTAL_PARA_PACK_SIZE; i++ )
	{
		ExtFlash_NORD( v, SECTOR0_PACK0_ADDR + FLASH_PACK_SIZE * ( i - 1 ) + READY_OFFSET, NORD_LEN + 1 );
		ThisPackReadyFlag = v->RxBuff[NORD_LEN];

		ExtFlash_NORD( v, SECTOR0_PACK0_ADDR + FLASH_PACK_SIZE * i + READY_OFFSET, NORD_LEN + 1 );
		NextPackReadyFlag = v->RxBuff[NORD_LEN];

		if( ThisPackReadyFlag == CHECK_WORD && NextPackReadyFlag == 0xFF )
		{
			PackAddr = SECTOR0_PACK0_ADDR + FLASH_PACK_SIZE * ( i - 1 );
			break;
		}
	}

	// Search for the ParamPack Address Pack 3
	if( PackAddr == 1 )
	{
		ThisPackReadyFlag = NextPackReadyFlag;

		ExtFlash_NORD( v, SECTOR0_PACK0_ADDR + READY_OFFSET, NORD_LEN + 1 );
		NextPackReadyFlag = v->RxBuff[NORD_LEN];

		if( ThisPackReadyFlag == CHECK_WORD && NextPackReadyFlag == 0xFF )
		{
			PackAddr = SECTOR1_PACK3_ADDR;
		}
		else
		{
			// search fail or uninitialized. Return LoadParam function.
			v->AlarmStatus |= FLASHERROR_UNINITIALIZED;
			return;
		}
	}

	// Check if version is illegal.
	ExtFlash_CheckExtFlashVersion ( v, PackAddr );
	v->RetryCount = 0;

	do
	{
		// Write data to ParamPack from RxBuffer.
		ExtFlash_ReadParamPack( v, PackAddr );

		// Check CRC
		tempCheckSum = 0;
		for( i = 0; i < sizeof(ParamPack_t) / sizeof(uint8_t) - 1; i++ )
		{
			tempCheckSum += *(((uint8_t *)&v->ParamPack) + i);
		}

		v->RetryCount++;
		if( v->RetryCount == RETRY_COUNT )
		{
			v->AlarmStatus |= FLASHERROR_CHECKSUM_FAIL;
			break;
		}
	} while ( v->ParamPack.CheckSum != tempCheckSum );
}

void ExtFlash_ParamBackup( ExtFlash_t *v, DriveParams_t *pDriveParams )
{
	uint16_t i;
	uint8_t ReadyFlag;
	uint16_t TempFlashVersion[EXT_FLASH_VER_NUM] = EXT_FLASH_VERSION;

	// Update to new external flash version
	for( i = 0; i < EXT_FLASH_VER_NUM; i++ )
	{
		v->ParamPack.Header.ExtFlashVersion[i] = TempFlashVersion[i];
	}

	// Ready the Param Pack
	v->ParamPack.DriveParamsExtMemory = *pDriveParams;
	v->ParamPack.ReadyFlag = CHECK_WORD;
	v->ParamPack.CheckSum = 0;
	for( i = 0; i < sizeof(ParamPack_t) / sizeof(uint8_t) - 1; i++ )
	{
		v->ParamPack.CheckSum += *(((uint8_t *)&v->ParamPack) + i);
	}

	ExtFlash_NORD( v, SECTOR0_PACK0_ADDR + READY_OFFSET, NORD_LEN + 1 );
	ReadyFlag = v->RxBuff[NORD_LEN];

	if( ReadyFlag == 0xFF )
	{
		// PP Pack0
		ExtFlash_WriteParamPack( v, SECTOR0_PACK0_ADDR );
	}
	else
	{
		ExtFlash_NORD( v, SECTOR0_PACK1_ADDR + READY_OFFSET, NORD_LEN + 1 );
		ReadyFlag = v->RxBuff[NORD_LEN];
		if( ReadyFlag == 0xFF )
		{
			ExtFlash_WriteParamPack( v, SECTOR0_PACK1_ADDR );
			ExtFlash_SER( v, SECTOR1_ADDR );
		}
		else
		{
			ExtFlash_NORD( v, SECTOR1_PACK2_ADDR + READY_OFFSET, NORD_LEN + 1 );
			ReadyFlag = v->RxBuff[NORD_LEN];
			if( ReadyFlag == 0xFF )
			{
				// PP Pack 2
				ExtFlash_WriteParamPack( v, SECTOR1_PACK2_ADDR );
			}
			else
			{
				ExtFlash_NORD( v, SECTOR1_PACK3_ADDR + READY_OFFSET, NORD_LEN + 1 );
				ReadyFlag = v->RxBuff[NORD_LEN];
				if( ReadyFlag == 0xFF )
				{
					// PP Pack 3 and Erase Sector 0
					ExtFlash_WriteParamPack( v, SECTOR1_PACK3_ADDR );
					ExtFlash_SER( v, SECTOR0_ADDR );
				}
				else
				{
					v->AlarmStatus |= FLASHERROR_DAMAGED;
				}
			}
		}
	}
}

void ExtFlash_CheckExtFlashVersion ( ExtFlash_t *v, uint32_t PACK_ADDR )
{
	uint16_t TempExtFlashVersion[EXT_FLASH_VER_NUM] = { 0 }; // X1.X2.nn.mm
	uint8_t i = 0;

	// read version from ext. Flash
	ExtFlash_NORD( v, PACK_ADDR, NORD_LEN + EXT_FLASH_VER_SIZE );
	for( i = 0; i < EXT_FLASH_VER_SIZE; i++ )
	{
		*((uint8_t*)TempExtFlashVersion + i) = v->RxBuff[NORD_LEN + i];
	}

	// normal operation
	v->pParaReadEnableTable = (uint8_t *)ParaReadEnableTable1p4;
}

void ExtFlash_LogTotalTime( ExtFlash_t *v )
{

	uint16_t i;
	uint32_t NextSector, NextQW;

	// Search next pack and sector.
	NextSector = GET_SECTOR_NUMBER( v->TotalTimeQWAddress ) + 1;
	if( NextSector > GET_SECTOR_NUMBER( TOTAL_TIME_END_ADDR ) )
	{
		NextSector = GET_SECTOR_NUMBER( TOTAL_TIME_FIRST_ADDR );
	}

	NextQW = GET_QW_NUMBER( v->TotalTimeQWAddress ) + 1;
	if( NextQW > GET_QW_NUMBER( TOTAL_TIME_END_ADDR ) )
	{
		NextQW = GET_QW_NUMBER( TOTAL_TIME_FIRST_ADDR );
	}

	v->pBufferTotalTimeQW->CheckWord = CHECK_WORD;
	v->pBufferTotalTimeQW->CheckSum = 0;

	// write check sum, note: checksum including CHECK_WORD.
	for( i = 0; i < sizeof(TotalTimeQW_t) / sizeof(uint8_t) - 1; i++ )
	{
		v->pBufferTotalTimeQW->CheckSum += *(((uint8_t*)v->pBufferTotalTimeQW) + i);
	}

	// Erase Next total time Sector, when next QW is the final QW in this sector.
	if( (NextQW & 0x1ff) == 0x1ff )
	{
		ExtFlash_SER( v, GET_SECTOR_ADDR( NextSector ) );
	}

	// Write Enable
	ExtFlash_WREN();

	// Page Programming. Write data into QWord
	ExtFlash_PP( v, GET_QW_ADDR( NextQW ), (uint8_t*)(v->pBufferTotalTimeQW), PP_LEN + TOTAL_TIME_LEN );
	// Check done
	ExtFlash_RDSR( v );
	do
	{
		ExtFlash_RDSR( v );
	}
	while( (v->StatusRegister & SR_WIP) == SR_WIP );

	v->TotalTimeQWAddress =  GET_QW_ADDR( NextQW );
}

void ExtFlash_ReadLastOPTotalTime( ExtFlash_t *v )
{
	uint32_t i;
	uint8_t TempCheckSum = 0;
	TotalTimeQW_t TempTTQW = { 0 };

	if( v->AlarmStatus & FLASHERROR_NULL_TOTAL_TIME )
	{
		// stop read data from external flash.
		return;
	}

	if( v->TotalTimeQWAddress > TOTAL_TIME_END_ADDR || v->TotalTimeQWAddress < TOTAL_TIME_FIRST_ADDR )
	{
		// stop read data from external flash
		return;
	}

	// Reset CheckWord warning
	v->WarningStatus &= ~FLASHWARNING_TT_CHECK_WORD_ERR;

	// check CheckWord
	ExtFlash_NORD( v, v->TotalTimeQWAddress + QW_READY_OFFSET, NORD_LEN + 1 );

	if( v->RxBuff[NORD_LEN] != CHECK_WORD )
	{
		v->WarningStatus |= FLASHWARNING_TT_CHECK_WORD_ERR;
		return;
	}
	v->RetryCount = 0;
	do
	{

		ExtFlash_NORD( v, v->TotalTimeQWAddress, NORD_LEN + FLASH_QW_SIZE );
		memcpy( ((uint8_t*)&TempTTQW), &v->RxBuff[NORD_LEN], FLASH_QW_SIZE );

		// Check checksum
		TempCheckSum = 0;
		for( i = 0; i < sizeof(TotalTimeQW_t) / sizeof(uint8_t) - 1; i++ )
		{
			TempCheckSum += *(((uint8_t*)&TempTTQW) + i);
		}
		v->RetryCount++;
		if( v->RetryCount == RETRY_COUNT )
		{
			v->AlarmStatus |= FLASHERROR_CHECKSUM_FAIL_TOTAL_TIME;
			break;
		}
	}
	while( TempTTQW.CheckSum != TempCheckSum );

	if( v->AlarmStatus & FLASHERROR_CHECKSUM_FAIL_TOTAL_TIME )
	{
		*v->pBufferTotalTimeQW = (TotalTimeQW_t){0, 0, 0, 0};
	}
	else
	{
		*v->pBufferTotalTimeQW = TempTTQW;
	}
}

void ExtFlash_EraseTotalTime( ExtFlash_t *v )
{
	uint16_t i = 0;
	uint16_t TotalSectorNum = TOTAL_TotalTime_QW >> 9;

	for( i = 0; i < TotalSectorNum; i++ )
	{
		ExtFlash_SER( v, TOTAL_TIME_FIRST_ADDR + GET_SECTOR_ADDR( i ) );
	}
}

void ExtFlash_Write_DTC_Data( ExtFlash_t *v , uint16_t DTC_Record_Num, uint8_t *pDataIn )
{
    uint8_t row_number = 0;
	//read the DTC record number to find empty row. 0xFF means empty row
    for ( row_number = 0; row_number < 32; row_number++ )
    {
	    ExtFlash_NORD( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num] + (row_number * FLASH_HALFPAGE_SIZE) + DTCChecksumOffset , NORD_LEN + 1 );

	    if ( v->RxBuff[NORD_LEN] == 0xFF )
	    {
	    	break;
	    }
    }
    if ( row_number < 32 )
    {
    	//If there is any empty row, program directly
    	uint8_t tempData[DATA_LENGTH_EACH_DTC_STORE] = {0};
    	for (uint32_t j = 0; j < ( DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset ); j++)
    	{
    		tempData[0] += pDataIn[j];
    	}
    	memcpy( &tempData[1], pDataIn, DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
		ExtFlash_Write_DTC_Data_by_ADDR( v,  v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num] + ( row_number * FLASH_HALFPAGE_SIZE ), &tempData[0] );
    }
    else
    {
    	/*If there is no empty row, follow the steps below:
    	 * 1. Read the first row data to backup the first Store DTC
    	 * 2. Erase this sector
    	 * 3. Write the first data into the first row of Flash
    	 * 4. Then , write the new data into the second row of Flash
    	 */
    	uint8_t tempData[DATA_LENGTH_EACH_DTC_STORE] = {0};
    	uint8_t tempData1[DATA_LENGTH_EACH_DTC_STORE] = {0};
        ExtFlash_NORD( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num], NORD_LEN + DATA_LENGTH_EACH_DTC_STORE );
        memcpy( &tempData[0], &v->RxBuff[NORD_LEN], DATA_LENGTH_EACH_DTC_STORE );
    	ExtFlash_SER( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num]);
		ExtFlash_Write_DTC_Data_by_ADDR( v,  v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num], &tempData[0] );

    	for (uint32_t j = 0; j < ( DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset ); j++)
    	{
    		tempData1[0] += pDataIn[j];
    	}
    	memcpy( &tempData1[1], pDataIn, DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
		ExtFlash_Write_DTC_Data_by_ADDR( v,  v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num] + FLASH_HALFPAGE_SIZE, &tempData1[0] );
    }
}
void ExtFlash_Read_DTC_Data( ExtFlash_t *v , uint16_t DTC_Record_Num, uint8_t *pDataOut0, uint8_t *pDataOut1 )
{
	//read the DTC record number to check if the first row is empty. 0xFF means empty row
    ExtFlash_NORD( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num], NORD_LEN + DATA_LENGTH_EACH_DTC_STORE );
    if ( v->RxBuff[NORD_LEN + 1] == 0xFF )
    {
    	return;
    }
    else
    {
    	//Check data valid or invalid by checksum
    	uint8_t tempChecksum = 0;
    	for (uint8_t j = DTCChecksumOffset; j < ( DATA_LENGTH_EACH_DTC_STORE ); j++)
    	{
    		tempChecksum += v->RxBuff[NORD_LEN + j];
    	}
    	if ( tempChecksum == v->RxBuff[NORD_LEN] )
    	{
    	    memcpy( pDataOut0, &(v->RxBuff[NORD_LEN + DTCChecksumOffset]), DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
    	}
    	else
    	{
    		// If the data from first row invalid, erase this sector.
        	ExtFlash_SER( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num]);
        	return;
    	}

    }
    /* Read the last stored DTC data.
     * read the DTC record number to find empty row. 0xFF means empty row,
     * the row just above the empty row is the last stored row
     */
	for (uint8_t row_number = 1; row_number < 32; row_number++ )
	{
	    ExtFlash_NORD( v, v->DTC_Store.ADDR_by_DTC_Record_Number[DTC_Record_Num] + (row_number * FLASH_HALFPAGE_SIZE) , NORD_LEN + DATA_LENGTH_EACH_DTC_STORE );
	    if ( v->RxBuff[NORD_LEN + 1] == 0xFF )
	    {
	    	return;
	    }
	    else
	    {
	    	uint8_t tempChecksum = 0;
	    	for (uint8_t j = DTCChecksumOffset; j < ( DATA_LENGTH_EACH_DTC_STORE ); j++)
	    	{
	    		tempChecksum += v->RxBuff[NORD_LEN + j];
	    	}
	    	if ( tempChecksum == v->RxBuff[NORD_LEN] )
	    	{
	    	    memcpy( pDataOut1, &v->RxBuff[NORD_LEN + DTCChecksumOffset], DATA_LENGTH_EACH_DTC_STORE - DTCChecksumOffset );
	    	}
	    }
	}
}
void ExtFlash_Clear_DTC_Data( ExtFlash_t *v )
{

	ExtFlash_BER64( v, v->DTC_Store.ADDR_by_DTC_Record_Number[0] );
	ExtFlash_BER64( v, v->DTC_Store.ADDR_by_DTC_Record_Number[16] );

}

static inline void ExtFlash_Write_DTC_Data_by_ADDR( ExtFlash_t *v, uint32_t Address, uint8_t *pData )
{
	// Write Enable
	ExtFlash_WREN();

	// Page 4 + data bytes
	ExtFlash_PP( v,  Address, pData, PP_LEN + DATA_LENGTH_EACH_DTC_STORE );

	// Check done
	ExtFlash_RDSR(v);
	do
	{
		ExtFlash_RDSR(v);
	}
	while( (v->StatusRegister & SR_WIP) == SR_WIP );
}
