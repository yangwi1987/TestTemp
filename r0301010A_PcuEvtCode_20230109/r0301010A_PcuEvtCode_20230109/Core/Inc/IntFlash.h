
/*
 * IntFlash.h
 *
 *  Created on: Mar 10, 2021
 *      Author: Will.Yang.CYY
 */

#ifndef INC_INTFLASH_H_
#define INC_INTFLASH_H_

#include "stm32g4xx_hal.h"
#include "string.h"

#define INT_FLASH_BASE	8		//DOUBLE WORD = 2*4=8 BYTES
#define INT_FLASH_IDX_HEAD			0
#define INT_FLASH_IDX_CHECKSUM		1
#define INT_FLASH_IDX_DATA			2
#define INT_FLASH_HEAD_MASK  		0x10

#define INT_FLASH_SPACE_START		(uint8_t*)0x8008000
#define INT_FLASH_SPACE_SIZE		0x800
#define INT_FLASH_SPACE_END			(uint8_t*)(INT_FLASH_SPACE_START+INT_FLASH_SPACE_SIZE-1)


typedef enum
{
	//Index template : INT_FLASH_DATA_GROUP_INDEX_
	INT_FLASH_DATA_GROUP_INDEX_ACTIVE_CODE,
	INT_FLASH_DATA_GROUP_INDEX_PCU_ID,
	
	INT_FLASH_DATA_GROUP_NUMBER,	//add new group name above this line!!!
}INTFlashGroupIndex_e;

//Erase structure define
#define ERASE_STRUCT_DEF_DEFAULT { \
		0, \
		0, \
		0, \
		0 }

//Active Code setup
#define ACTIVE_CODE_SPACE_START INT_FLASH_SPACE_START
#define ACTIVE_CODE_DATA_BYTE_NUMBER 16
#define ACTIVE_CODE_ELEMENT_BYTE_NUMBER INT_FLASH_BASE*3
#define ACTIVE_CODE_ELEMENT_NUMBER 20
#define ACTIVE_CODE_TOTAL_SPACE_SIZE	ACTIVE_CODE_ELEMENT_BYTE_NUMBER*ACTIVE_CODE_ELEMENT_NUMBER

//PCU ID setup
#define PCU_ID_SPACE_START ACTIVE_CODE_SPACE_START+ACTIVE_CODE_TOTAL_SPACE_SIZE
#define PCU_ID_DATA_BYTE_NUMBER 10
#define PCU_ID_ELEMENT_BYTE_NUMBER INT_FLASH_BASE*2
#define PCU_ID_ELEMENT_NUMBER 20
#define PCU_ID_TOTAL_SPACE_SIZE	PCU_ID_ELEMENT_BYTE_NUMBER*PCU_ID_ELEMENT_NUMBER

typedef void (*funcTypeIntFlash_Init) ( const void* );
typedef uint16_t (*funcTypeIntFlash_Read) ( void*, uint16_t, uint8_t* );
typedef uint16_t (*funcTypeIntFlash_Write) ( void*, uint16_t, uint8_t*, uint8_t* );
typedef uint16_t (*funcTypeIntFlash_Clear) ( void*, uint16_t );
typedef void (*funcTypeIntFlash_FlashProgram) ( uint32_t, uint64_t );
typedef void (*funcTypeIntFlash_IDsectionErase) ( void*, uint32_t*, uint16_t );

typedef enum
{
	INT_FLASH_NONE,
	INT_FLASH_READ_SUCCESS,
	INT_FLASH_READ_WRONG_CHECKSUM,
	INT_FLASH_READ_EMPTY,
	INT_FLASH_CLEARED,
	INT_FLASH_WRITE_SUCCESS,
	INT_FLASH_WRITE_FULL,
	INT_FLASH_WRITE_DUMMY_DATA,
	INT_FLASH_WRITE_FAIL,
	INT_FLASH_INIT_WRONG_PARAM,
	INT_FLASH_INIT_SUCCESS
} IntFlashStatus1_e;

typedef struct
{
	uint8_t const *const pStart;
	const uint16_t ElementByteNumber;
	const uint16_t ElementNumber;
	const uint16_t DataByteNumber;
	const uint16_t Reserved;
} IntFlashSetup_t;

typedef struct
{
	const IntFlashSetup_t *pSetup;
	uint8_t *pLast;
	uint16_t Status;
	uint16_t NextIndex;
} IntFlashInform_t;

typedef struct
{
	IntFlashSetup_t const *const pSetupTable;
	IntFlashInform_t *const pInformTable;
	const uint16_t InformNum;
	uint16_t Reserved;
	FLASH_EraseInitTypeDef EraseInfo;
	uint32_t PageError;
	funcTypeIntFlash_Init Init;
	funcTypeIntFlash_Read Read;
	funcTypeIntFlash_Write Write;
	funcTypeIntFlash_Clear Clear;
	funcTypeIntFlash_IDsectionErase IDSectionErase;

} IntFlashCtrl_t;

void IntFlash_Init ( IntFlashCtrl_t *p );
uint16_t IntFlash_Read ( IntFlashCtrl_t *v, uint16_t Idx, uint8_t *pDataOut );
uint16_t IntFlash_Write ( IntFlashCtrl_t *v, uint16_t Idx, uint8_t *pDataIn, uint8_t *pDataOut );
uint16_t IntFlash_Clear ( IntFlashCtrl_t *v, uint16_t Idx );
void IntFlash_FlashProgram ( uint32_t AddrValueIn, uint64_t DataIn );
void IntFlash_IDSectionErase( IntFlashCtrl_t *v, uint32_t *FnCmd, uint16_t SecurityLv );

#define INT_FLASH_INFORM_DEFAULT	\
{	\
	0,0,0,0,\
}	\

extern const IntFlashSetup_t IntFlashSetupTable[INT_FLASH_DATA_GROUP_NUMBER];
extern IntFlashInform_t IntFlashINformTable[INT_FLASH_DATA_GROUP_NUMBER];

#define INT_FLASH_CTRL_DEFAULT	\
{	\
	IntFlashSetupTable,\
	IntFlashINformTable,\
	INT_FLASH_DATA_GROUP_NUMBER,\
	0,\
	ERASE_STRUCT_DEF_DEFAULT,\
	0,\
	(funcTypeIntFlash_Init)IntFlash_Init,\
	(funcTypeIntFlash_Read)IntFlash_Read,\
	(funcTypeIntFlash_Write)IntFlash_Write,\
	(funcTypeIntFlash_Clear)IntFlash_Clear,\
	(funcTypeIntFlash_IDsectionErase)IntFlash_IDSectionErase,\
}	\

#endif /* INC_INTFLASH_H_ */
