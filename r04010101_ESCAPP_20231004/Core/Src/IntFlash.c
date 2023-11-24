/*
 * IntFlash.c
 *
 *  Created on: Mar 10, 2021
 *      Author: Will.Yang.CYY
 */

#include "IntFlash.h"

const IntFlashSetup_t IntFlashSetupTable[INT_FLASH_DATA_GROUP_NUMBER] =
{
/*
 * element template
 {
 .pStart = TBD,
 .ElementByteNumber = TBD,
 .ElementNumber = TBD,
 .DataByteNumber = TBD,
 .Reserved = 0
 }
 */
		//Active Code data
		{
				.pStart = ACTIVE_CODE_SPACE_START,
				.ElementByteNumber = ACTIVE_CODE_ELEMENT_BYTE_NUMBER,
				.ElementNumber = ACTIVE_CODE_ELEMENT_NUMBER,
				.DataByteNumber = ACTIVE_CODE_DATA_BYTE_NUMBER,
				.Reserved = 0
		},
		//PU ID data
		{
				.pStart = PCU_ID_SPACE_START,
				.ElementByteNumber = PCU_ID_ELEMENT_BYTE_NUMBER,
				.ElementNumber = PCU_ID_ELEMENT_NUMBER,
				.DataByteNumber = PCU_ID_DATA_BYTE_NUMBER,
				.Reserved = 0
		},
};

IntFlashInform_t IntFlashINformTable[INT_FLASH_DATA_GROUP_NUMBER];

void IntFlash_Init ( IntFlashCtrl_t *p )
{
	uint8_t i = 0;
	for ( i = 0; i < p->InformNum; i++ )
	{
		(p->pInformTable + i)->pSetup = p->pSetupTable + i;
		(p->pInformTable + i)->NextIndex = 0;
		(p->pInformTable + i)->pLast = (p->pInformTable + i)->pSetup->pStart;
		p->Read ( p, i, NULL );
	}
}

uint16_t IntFlash_Read ( IntFlashCtrl_t *v, uint16_t Idx, uint8_t *pDataOut )
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t temp = 0;
	const uint8_t *pData;
	IntFlashInform_t *p;
	p = (v->pInformTable + Idx);
	for ( i = 0; i < p->pSetup->ElementNumber; i++ )
	{
		if ( *(p->pSetup->pStart + (i * (p->pSetup->ElementByteNumber) + INT_FLASH_IDX_HEAD)) == 0xFF )
		{
			break;
		}
	}

	p->NextIndex = i;
	if ( i == 0 )
	{
		p->Status = INT_FLASH_READ_EMPTY;
	}
	else
	{
		p->pLast = p->pSetup->pStart + (p->NextIndex - 1) * p->pSetup->ElementByteNumber;
		if ( pDataOut != NULL )
		{
			memcpy ( pDataOut, p->pLast + INT_FLASH_IDX_DATA, p->pSetup->DataByteNumber );
		}

		if ( (*(p->pLast + INT_FLASH_IDX_HEAD) & INT_FLASH_HEAD_MASK) == 0 )
		{
			p->Status = INT_FLASH_CLEARED;
		}
		else
		{
			//checksum calculate
			pData = (p->pLast + INT_FLASH_IDX_DATA);
			temp = 0;
			for ( j = 0; j < p->pSetup->DataByteNumber; j++ )
			{
				temp += *(pData + j);
			}
			temp += *(p->pLast + INT_FLASH_IDX_CHECKSUM);
			if ( temp != 0 )
			{
				p->Status = INT_FLASH_READ_WRONG_CHECKSUM;
			}
			else
			{
				p->Status = INT_FLASH_READ_SUCCESS;
			}
		}
	}
	return p->Status;
}

uint16_t IntFlash_Write ( IntFlashCtrl_t *v, uint16_t Idx, uint8_t *pDataIn, uint8_t *pDataOut )
{
	uint16_t i = 0;
	uint16_t j = 0;
	uint32_t AddrValue = 0;

	uint8_t CompareResult = 0;
	uint8_t Head = 0;
	uint8_t WriteFlag = 0;
	uint8_t Checksum = 0;
	uint16_t DataHandled = 0;
	uint64_t Buffer = 0;
	uint32_t OffSet = 0;
	IntFlashInform_t *p;
	p = (v->pInformTable + Idx);

	WriteFlag = 0;
	IntFlash_Read ( v, Idx, pDataOut );
	switch ( p->Status )
	{
		case INT_FLASH_READ_EMPTY:
			WriteFlag = 1;
			break;
		case INT_FLASH_READ_SUCCESS:
			CompareResult = memcmp ( pDataIn, (p->pLast + INT_FLASH_IDX_DATA), p->pSetup->DataByteNumber );
			if ( CompareResult != 0 )
			{
				WriteFlag = 1;
			}
			else
			{
				WriteFlag = 0;
				p->Status = INT_FLASH_WRITE_DUMMY_DATA;
			}
			break;
		case INT_FLASH_READ_WRONG_CHECKSUM:
			WriteFlag = 1;
			break;
		case INT_FLASH_CLEARED:
			WriteFlag = 1;
			break;
		default:
			p->Status = INT_FLASH_NONE;
			break;
	}

	if ( WriteFlag == 1 )
	{
		if ( p->NextIndex == p->pSetup->ElementNumber )
		{
			//no room for new active code ,return full
			p->Status = INT_FLASH_WRITE_FULL;
		}
		else
		{
			DataHandled = 0;
			Head = (uint8_t)((0x000F) & p->NextIndex) + INT_FLASH_HEAD_MASK;
			AddrValue = (uint32_t)(p->pSetup->pStart + (p->NextIndex * p->pSetup->ElementByteNumber));
			for ( i = 0; i < p->pSetup->DataByteNumber; i++ )
			{
				Checksum -= *(pDataIn + i);
			}

			for ( i = 0; i < (p->pSetup->ElementByteNumber >> 3); i++ )
			{
				if ( i == 0 )
				{
					*((uint8_t*)&Buffer) = Head;
					*((uint8_t*)&Buffer + 1) = Checksum;
					OffSet = 2;
				}
				else
				{
					OffSet = 0;
				}

				for ( j = 0; j < (INT_FLASH_BASE - OffSet); j++ )
				{

					if ( DataHandled < p->pSetup->DataByteNumber )
					{
						*((uint8_t*)&Buffer + OffSet + j) = *(pDataIn + DataHandled);
					}
					else
					{
						*((uint8_t*)&Buffer + OffSet + j) = 0;
					}

					DataHandled++;
				}
				IntFlash_FlashProgram ( AddrValue, Buffer );
				HAL_Delay ( 1 );
				AddrValue += INT_FLASH_BASE;
			}
			IntFlash_Read ( v, Idx, pDataOut );
		}
	}
	return p->Status;
}

uint16_t IntFlash_Clear ( IntFlashCtrl_t *v, uint16_t Idx )
{
	uint16_t i = 0;
	uint64_t Buffer = 0;
	uint32_t AddrValue = 0;
	IntFlashInform_t *p;
	p = (v->pInformTable + Idx);
	
	IntFlash_Read ( v, Idx, NULL );
	AddrValue = (uint32_t)(p->pLast);

	if ( (p->Status == INT_FLASH_READ_SUCCESS) || (p->Status == INT_FLASH_READ_WRONG_CHECKSUM) )
	{
		for ( i = 0; i < (p->pSetup->ElementByteNumber >> 3); i++ )
		{
			Buffer = 0;
			IntFlash_FlashProgram ( AddrValue, Buffer );
			HAL_Delay ( 1 );
			AddrValue += INT_FLASH_BASE;
		}
	}
	IntFlash_Read ( v, Idx, NULL );
	return p->Status;
}

void IntFlash_FlashProgram ( uint32_t AddrValueIn, uint64_t DataIn )
{
	__disable_irq ();
	HAL_FLASH_Unlock ();
	__HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_ALL_ERRORS );
	HAL_FLASH_Program ( FLASH_TYPEPROGRAM_DOUBLEWORD, AddrValueIn, DataIn );
	HAL_FLASH_Lock ();
	__enable_irq ();
}

void IntFlash_IDSectionErase( IntFlashCtrl_t *v, uint32_t *FnCmd, uint16_t SecurityLv )
{
	uint32_t CmdState = 0;
	CmdState = *FnCmd;
	if( ( CmdState != 1 ) || ( SecurityLv < 5 ) )
		return;

	if( CmdState > 0 )
	{
		v->EraseInfo.Banks = FLASH_BANK_1;
		v->EraseInfo.Page = 0x8000 >> 11;
		v->EraseInfo.NbPages = 0x800 >> 11;
		v->EraseInfo.TypeErase = FLASH_TYPEERASE_PAGES;
		__disable_irq ();
		HAL_FLASH_Unlock ();
		__HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_ALL_ERRORS );
		HAL_FLASHEx_Erase( &v->EraseInfo, &v->PageError);
		HAL_FLASH_Lock ();
		__enable_irq ();
	}
	*FnCmd = 0;
}
