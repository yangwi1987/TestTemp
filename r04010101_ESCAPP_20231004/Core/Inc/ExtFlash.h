/*
 * ExtFlash.h
 *
 *  Created on: Jul 17, 2020
 *      Author: Mike.Wen.SFW
 */

#ifndef INC_EXTFLASH_H_
#define INC_EXTFLASH_H_


#include "ParamTable.h"
#include "TotalTimeQW.h"

/*
 * Memory address definition
 */
#define NULL_ADDRESS 0

// allocate sector0 ~ sector1 for parameter system
#define TOTAL_PARA_PACK_SIZE		4
#define SECTOR0_ADDR				0x00000000
#define SECTOR0_PACK0_ADDR			0x00000000
#define SECTOR0_PACK1_ADDR			0x00000800
#define SECTOR1_ADDR				0x00001000
#define SECTOR1_PACK2_ADDR			0x00001000
#define SECTOR1_PACK3_ADDR			0x00001800

// allocate sector2 ~ sector3 for total time
#define	SECTOR2_ADDR				0x00002000
#define	SECTOR2_PACK4_ADDR			0x00002000 // first total time QW ADDR
#define	SECTOR2_PACK5_ADDR			0x00002800

#define	SECTOR3_ADDR				0x00003000
#define	SECTOR3_PACK6_ADDR			0x00003000
#define	SECTOR3_PACK7_ADDR			0x00003800
#define	SECTOR3_QW511_ADDR			0x00003FF8 // end total time QW ADDR

// allocate sector4 ~ sector5 for current calibration
#define	SECTOR4_ADDR				0x00004000
#define	SECTOR4_PACK8_ADDR			0x00004000 // first Current Calibration 8QW ADDR
#define	SECTOR4_PACK9_ADDR			0x00004800

#define	SECTOR5_ADDR				0x00005000
#define	SECTOR5_PACK10_ADDR			0x00005000
#define	SECTOR5_PACK11_ADDR			0x00005800
#define	SECTOR5_8QW63_ADDR			0x00005FC0 // end Current Calibration 8QW ADDR

// reserve sector6 ~ sector7
#define	SECTOR6_ADDR				0x00006000
#define SECTOR7_ADDR				0x00007000
#define SECTOR7_PACK15_ADDR			0x00007800

#define TOTAL_TIME_FIRST_ADDR	SECTOR2_PACK4_ADDR
#define TOTAL_TIME_END_ADDR		SECTOR3_QW511_ADDR

#define TOTAL_TIME_SIZE_IN_QW       1
#define TOTAL_TotalTime_QW 			1024 // 2 sector = 1024 QWord

#define CURRENT_CALIB_FIRST_ADDR	SECTOR4_PACK8_ADDR
#define CURRENT_CALIB_END_ADDR		SECTOR5_8QW63_ADDR

#define CURRENT_CALIB_SIZE_IN_QW    8
#define TOTAL_CURRENT_CALIB_8QW		128  // 2 sector = 128 * 8QWord

/* Macro for DTC store*/
#define	SECTOR16_ADDR				0x00010000
#define	SECTOR17_ADDR				0x00011000
#define	SECTOR18_ADDR				0x00012000
#define SECTOR19_ADDR				0x00013000
#define SECTOR20_ADDR				0x00014000
#define SECTOR21_ADDR				0x00015000
#define SECTOR22_ADDR				0x00016000
#define SECTOR23_ADDR				0x00017000
#define SECTOR24_ADDR				0x00018000
#define SECTOR25_ADDR				0x00019000
#define SECTOR26_ADDR				0x0001A000
#define SECTOR27_ADDR				0x0001B000
#define SECTOR28_ADDR				0x0001C000
#define SECTOR29_ADDR				0x0001D000
#define SECTOR30_ADDR				0x0001E000
#define SECTOR31_ADDR				0x0001F000
#define SECTOR32_ADDR				0x00020000
#define SECTOR33_ADDR				0x00021000
#define SECTOR34_ADDR				0x00022000
#define SECTOR35_ADDR				0x00023000
#define SECTOR36_ADDR				0x00024000
#define SECTOR37_ADDR				0x00025000
#define SECTOR38_ADDR				0x00026000
#define SECTOR39_ADDR				0x00027000
#define SECTOR40_ADDR				0x00028000
#define SECTOR41_ADDR				0x00029000
#define SECTOR42_ADDR				0x0002A000
#define SECTOR43_ADDR				0x0002B000
#define SECTOR44_ADDR				0x0002C000
#define SECTOR45_ADDR				0x0002D000
#define SECTOR46_ADDR				0x0002E000
#define SECTOR47_ADDR				0x0002F000

#define DTCChecksumOffset 1

#define DATA_LENGTH_EACH_DTC_STORE  (116 + DTCChecksumOffset)

// Macro function
#define GET_SECTOR_NUMBER(address) 	(address >> 12)
#define GET_PACK_NUMBER(address) 	(address >> 11)
#define GET_QW_NUMBER(address) 		(address >> 3)
#define GET_8QW_NUMBER(address)     (address >> 6)
#define GET_SECTOR_ADDR(number) 	(number << 12)
#define GET_PACK_ADDR(number) 		(number << 11)
#define GET_QW_ADDR(number) 		(number << 3)
#define GET_8QW_ADDR(number)        (number << 6)

/*
 * External flash constant definition
 */
#define SPI_BUF_LEN			260 // page size + NORD length
#define TIMEOUT				1000
#define RETRY_COUNT			3
#define SER_TIMEOUT			1000

#define EXT_FLASH_VERSION	{ \
	PROJECT_CODE_X1, \
	(PRODUCT_AND_BIN_TYPE_X2 & PRODUCT_TYPE_MASK), \
	EXT_FLASH_MAIN_RELEASE_NN, \
	EXT_FLASH_MINOR_RELEASE_MM \
}

#define NULL_READ_TABLE_INDEX 	7U
#define RESCUE_TABLE_INDEX 		0x0AU

#define PP			0x02	// Page Programming
#define NORD		0x03	// Normal Read
#define WRDI		0x04	// Write Disable
#define RDSR		0x05	// Read status register
#define WREN		0x06	// Write Enable
#define SER			0xD7	// Sector Erase
#define BER64       0xD8    // Block Erase 64Kbyte
#define CER			0xC7	// Chip Erase
#define JEDEC		0x9F

#define PP_LEN		4
#define NORD_LEN	4
#define WRDI_LEN	1
#define RDSR_LEN	2
#define WREN_LEN	1
#define	SER_LEN		4
#define	CER_LEN		1
#define JEDEC_LEN	4
#define TOTAL_TIME_LEN 8
#define CURRENT_CALIB_LEN 64

#define SR_WIP		0x01	// Write In Progress
#define SR_WEL		0x02	// Write Enable Latch

#define FLASH_QW_SIZE				8		// bytes, QW: QWord = 64 bit
#define FLASH_8QW_SIZE              64      // bytes
#define FLASH_HALFPAGE_SIZE         128     // byte
#define FLASH_PP_SIZE				256		// bytes
#define FLASH_PACK_SIZE				2048	// bytes
#define PP_COUNT_PER_PACK			(FLASH_PACK_SIZE / FLASH_PP_SIZE)

#define CHECK_WORD					0xAA

#define EXT_FLASH_VER_NUM			4 		// 16 bits
#define EXT_FLASH_HEADER_RSVD_NUM	12		// 16 bits

// define size(Bytes) of each data region
#define EXT_FLASH_VER_SIZE			(EXT_FLASH_VER_NUM * 2)
#define EXT_FLASH_HEADER_RSVD_SIZE	(EXT_FLASH_HEADER_RSVD_NUM * 2)			// header reserved
#define EXT_FLASH_HEADER_SIZE		sizeof(ParamPackHeader) 				// version + header reserved
#define EXT_FLASH_PARAM_SIZE		sizeof(DriveParams_t)
// EXT_FLASH_PARAM_RSVD_SIZE =       pack - header - param - check word - check sum
#define EXT_FLASH_PARAM_RSVD_SIZE	(FLASH_PACK_SIZE - EXT_FLASH_HEADER_SIZE - EXT_FLASH_PARAM_SIZE - 1 - 1)

// define offset of memory address
#define HEADER_OFFSET				0
#define PARAM_OFFSET				HEADER_OFFSET + EXT_FLASH_HEADER_SIZE	// parameter offset
#define RSVD_OFFSET					PARAM_OFFSET + EXT_FLASH_PARAM_SIZE		// Reserved parameter offset
#define READY_OFFSET				2046									// check word offset
#define CHECK_SUM_OFFSET			2047
#define QW_READY_OFFSET				6										// QWord check word offset
#define QW_CHECK_SUM_OFFSET			7										// QWord checksum offset

// define read enable table size and structure
// Assume that: each page in different parameter regions has the same size.
#define PAGE_SIZE 					SYS_PARAM_SIZE										// 200 parameters(uint16_t)
#define ROW_SIZE 					8 													// 8 parameters(uint16_t)
#define PAGE_NUM 					((SYS_PARAM_SIZE + PCU_PARAM_SIZE ) / PAGE_SIZE)	// 2
#define ROW_NUM 					(PAGE_SIZE  / ROW_SIZE) 							// 25
#define R_SHIFT_TO_ROW 				3 													// R_SHIFT_TO_ROW = log2( ROW_SIZE )

// Bit 0~3: parameters backup error
typedef enum
{
	FLASHERROR_NONE				= 0b00000000,
	FLASHERROR_UNINITIALIZED	= 0b00000001,
	FLASHERROR_CHECKSUM_FAIL	= 0b00000010,
	FLASHERROR_DAMAGED			= 0b00001000,
	FLASHERROR_NULL_CURR_CAL_BACKUP 	= 0b00010000, // no current calibration backup in another section of external flash
	FLASHERROR_NULL_TOTAL_TIME			= 0b00100000, // no total time exist
	FLASHERROR_CHECKSUM_FAIL_TOTAL_TIME	= 0b01000000,
	FLASHERROR_CHECKSUM_FAIL_CURR_CAL_BACKUP = 0b10000000,
} E_FLASH_ERROR; // note: AlarmStatus is 8 bits;

typedef enum
{
	FLASHWARNING_NONE				    = 0b00000000,
	FLASHWARNING_CURRCLB_CHECK_WORD_ERR = 0b00010000, // check word is error in current calibration
	FLASHWARNING_TT_CHECK_WORD_ERR	    = 0b00100000, // check word is error in total time
} E_FLASH_WARNING; // note: WarningStatus is 8 bits;

typedef enum
{
	FLASH_TOTAL_TIME_IDLE = 0,
	FLASH_TOTAL_TIME_START = 1,
	FLASH_TOTAL_TIME_FINISH = 2,
} E_FLASH_TOTAL_TIME_LOG_STATE;


/*
 * External flash data structure
 */
extern SPI_HandleTypeDef hspi1;

typedef void (*functypeExtFlash_Init)( void* );
typedef void (*functypeExtFlash_LoadParam)(void*);
typedef void (*functypeExtFlash_ParamBackup)(void*, void*);

typedef void (*functypeExtFlash_CurrentCalibrationBackup)(void*, void*);
typedef void (*functypeExtFlash_ReadCurrentCalibration)( void*, void*);

/*define function for DTC*/

typedef void (*functypeExtFlash_Write_DTC_Data)( void*, uint16_t, uint8_t* );
typedef void (*functypeExtFlash_Read_DTC_Data)( void*, uint16_t, uint8_t*, uint8_t* );
typedef void (*functypeExtFlash_Clear_DTC_Data)( void* );

/*
 *   DTC related data structure
 */

typedef struct
{
	uint32_t ADDR_by_DTC_Record_Number[32];
	uint8_t DTC_Record_Row_Number[32];
	functypeExtFlash_Write_DTC_Data Write_DTC_Data;
	functypeExtFlash_Read_DTC_Data  Read_DTC_Data;
	functypeExtFlash_Clear_DTC_Data Clear_DTC_Data;
}ExtFlash_DTC_Store_t;


typedef struct
{
	uint16_t Axis1_Iu_Scale[2];						// 4 Byte
	uint16_t Axis1_Iv_Scale[2];						// 4 Byte
	uint16_t Axis1_Iw_Scale[2];						// 4 Byte
	uint16_t Axis2_Iu_Scale[2];						// 4 Byte
	uint16_t Axis2_Iv_Scale[2];						// 4 Byte
	uint16_t Axis2_Iw_Scale[2];						// 4 Byte
	uint16_t Axis1_Iu_ZeroPoint;					// 2 Byte
	uint16_t Axis1_Iv_ZeroPoint;					// 2 Byte
	uint16_t Axis1_Iw_ZeroPoint;					// 2 Byte
	uint16_t Axis2_Iu_ZeroPoint;					// 2 Byte
	uint16_t Axis2_Iv_ZeroPoint;					// 2 Byte
	uint16_t Axis2_Iw_ZeroPoint;					// 2 Byte
	uint16_t Reserved[13];							// 26 Byte
	uint8_t CheckWord;								// 1 Byte
	uint8_t CheckSum;								// 1 Byte
} ExtFlash_Current_Calibration_t;

typedef struct
{
	uint8_t ReadDoneFlag;
	uint8_t LoadCurrCalibRequest;
	functypeExtFlash_CurrentCalibrationBackup 	CurrentCalibrationBackup;
	functypeExtFlash_ReadCurrentCalibration		ReadCurrentCalibration;
} ExtFlash_Curr_Calib_Store_t;

typedef struct
{
	uint16_t ExtFlashVersion[EXT_FLASH_VER_NUM];
	uint16_t Reserved[EXT_FLASH_HEADER_RSVD_NUM];
} ParamPackHeader;

typedef struct
{
	ParamPackHeader Header;							// 32 Bytes
	DriveParams_t DriveParamsExtMemory; 			// 1600 Bytes
	uint8_t Reserved[EXT_FLASH_PARAM_RSVD_SIZE];	// 414 Bytes
	uint8_t ReadyFlag;								// 1 Byte (check word)
	uint8_t CheckSum;								// 1 Byte
} ParamPack_t;

typedef struct
{
	uint16_t DataLength;
	uint8_t ParamBackupRequest;
	uint8_t AlarmStatus;
	uint8_t WarningStatus;
	uint8_t RetryCount;
	uint8_t	StatusRegister;
	uint8_t RxBuff[SPI_BUF_LEN];
	uint8_t TxBuff[SPI_BUF_LEN];
	uint8_t *pParaReadEnableTable;
	TotalTimeQW_t *pBufferTotalTimeQW;
	uint32_t TotalTimeQWAddress; // address of last total time QWord
	uint32_t CurrentCalib8QWAddress; // address of last current calibration 8QWord
	ParamPack_t ParamPack;
	ExtFlash_DTC_Store_t DTC_Store;
	ExtFlash_Curr_Calib_Store_t Curr_Calib_Store;
	functypeExtFlash_Init Init;
	functypeExtFlash_LoadParam LoadParam;
	functypeExtFlash_ParamBackup ParamBackup;
} ExtFlash_t;

/*
 *  function declare
 */

void ExtFlash_NORD( ExtFlash_t *v, uint32_t Address, uint16_t TotalLen );
void ExtFlash_PP( ExtFlash_t *v, uint32_t Address, uint8_t *pData, uint16_t TotalLen );
void ExtFlash_WREN( void );
void ExtFlash_RDSR( ExtFlash_t *v );
void ExtFlash_SER( ExtFlash_t *v, uint32_t Address );
void ExtFlash_BER64( ExtFlash_t *v, uint32_t Address );
void ExtFlash_CER( ExtFlash_t *v );
void ExtFlash_CheckExtFlashVersion( ExtFlash_t *v, uint32_t PACK_ADDR );
void ExtFlash_ReadParamPack( ExtFlash_t *v, uint32_t PACK_ADDR );
void ExtFlash_WriteParamPack( ExtFlash_t *v, uint32_t PACK_ADDR );
void ExtFlash_LoadParam( ExtFlash_t *v );
void ExtFlash_ParamBackup( ExtFlash_t *v, DriveParams_t *pDriveParams );

void ExtFlash_Init( ExtFlash_t *v );
void ExtFlash_ReadTotalTime( ExtFlash_t *v );
void ExtFlash_ReadLastOPTotalTime( ExtFlash_t *v );
void ExtFlash_LogTotalTime( ExtFlash_t *v);
void ExtFlash_EraseTotalTime( ExtFlash_t *v);

void ExtFlash_LoadBufferCurrentCalibration( ExtFlash_t *v, ExtFlash_Current_Calibration_t* pBufferCurrentCalib, DriveParams_t *pDriveParams );
void ExtFlash_CurrentCalibrationBackup( ExtFlash_t *v, DriveParams_t *pDriveParams );
void ExtFlash_ReadCurrentCalibration( ExtFlash_t *v, ExtFlash_Current_Calibration_t *pTempCurrentCalib );

void ExtFlash_Write_DTC_Data( ExtFlash_t *v , uint16_t DTC_Record_Num, uint8_t *pDataIn );
void ExtFlash_Read_DTC_Data( ExtFlash_t *v , uint16_t DTC_Record_Num, uint8_t *pDataOut0, uint8_t *pDataOut1 );
void ExtFlash_Clear_DTC_Data( ExtFlash_t *v );

#define PARAM_PACK_HEADER_DEFAULT { \
	{0}, /* ExtFlashVersion */ \
	{0} /* Reserved */ }

#define PARAM_PACK_DEFAULT { \
	PARAM_PACK_HEADER_DEFAULT, \
	{ {0}, {0}, {{0}, {0}} }, \
	{0}, \
	0, \
	0 }

#define EXT_FLASH_DEFAULT {   \
	0, /* DataLength */ \
	0, /* ParamBackupRequest */ \
	FLASHERROR_NONE, /* AlarmStatus */ \
	FLASHWARNING_NONE, /* WarningStatus */ \
	0, /* RetryCount */ \
	0, /* StatusRegister */ \
	{0}, /* RxBuff[SPI_BUF_LEN] */ \
	{0}, /* TxBuff[SPI_BUF_LEN] */ \
	0, /* pParaReadEnableTable*/ \
	0, /* pBufferTotalTimeQW */\
	0, /* TotalTimeQWAddress */\
	0, /* CurrentCalib8QWAddress */\
	PARAM_PACK_DEFAULT, \
	DTC_STORE_DEFAULT, /*ExtFlash_DTC_Store_t*/\
	CURR_CALIB_STORE_DEFAULT, /*ExtFlash_Curr_Calib_Store_t*/\
	(functypeExtFlash_Init)ExtFlash_Init, \
	(functypeExtFlash_LoadParam)ExtFlash_LoadParam, \
	(functypeExtFlash_ParamBackup)ExtFlash_ParamBackup, \
}

#define DTC_STORE_DEFAULT {  \
		ADDR_BY_DTC_RECORD_NUM_DEFAULT,\
		{0},\
		(functypeExtFlash_Write_DTC_Data) ExtFlash_Write_DTC_Data,\
		(functypeExtFlash_Read_DTC_Data)  ExtFlash_Read_DTC_Data,\
		(functypeExtFlash_Clear_DTC_Data) ExtFlash_Clear_DTC_Data,\
	}

#define CURR_CALIB_STORE_DEFAULT {  \
		0, /* ReadDoneFlag */\
		0, /* LoadCurrCalibRequest */\
		(functypeExtFlash_CurrentCalibrationBackup) ExtFlash_CurrentCalibrationBackup,\
		(functypeExtFlash_ReadCurrentCalibration)  ExtFlash_ReadCurrentCalibration,\
	}

#define ADDR_BY_DTC_RECORD_NUM_DEFAULT {\
		SECTOR16_ADDR,   /* Num0   */ \
		SECTOR17_ADDR,   /* Num1   */ \
		SECTOR18_ADDR,   /* Num2   */ \
		SECTOR19_ADDR,   /* Num3   */ \
		SECTOR20_ADDR,   /* Num4   */ \
		SECTOR21_ADDR,   /* Num5   */ \
		SECTOR22_ADDR,   /* Num6   */ \
		SECTOR23_ADDR,   /* Num7   */ \
		SECTOR24_ADDR,   /* Num8   */ \
		SECTOR25_ADDR,   /* Num9   */ \
		SECTOR26_ADDR,   /* Num10  */ \
		SECTOR27_ADDR,   /* Num11  */ \
		SECTOR28_ADDR,   /* Num12  */ \
		SECTOR29_ADDR,   /* Num13  */ \
		SECTOR30_ADDR,   /* Num14  */ \
		SECTOR31_ADDR,   /* Num15  */ \
		SECTOR32_ADDR,   /* Num16  */ \
		SECTOR33_ADDR,   /* Num17  */ \
		SECTOR34_ADDR,   /* Num18  */ \
		SECTOR35_ADDR,   /* Num19  */ \
		SECTOR36_ADDR,   /* Num20  */ \
        SECTOR37_ADDR,   /* Num21  */ \
        SECTOR38_ADDR,   /* Num22  */ \
        SECTOR39_ADDR,   /* Num23  */ \
        SECTOR40_ADDR,   /* Num24  */ \
        SECTOR41_ADDR,   /* Num25  */ \
        SECTOR42_ADDR,   /* Num26  */ \
        SECTOR43_ADDR,   /* Num27  */ \
        SECTOR44_ADDR,   /* Num28  */ \
        SECTOR45_ADDR,   /* Num29  */ \
        SECTOR46_ADDR,   /* Num30  */ \
        SECTOR47_ADDR,   /* Num31  */ \
        }


#endif /* INC_EXTFLASH_H_ */
